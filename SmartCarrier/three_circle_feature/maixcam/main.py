"""MaixCam Pro detector for the numbered three-circle board.

Deploy this file together with three_circle_geometry.py. It is a dedicated
feature build; the existing repository MaixCam scripts remain unchanged.

Request:  AA 00 31 BB
Response: fixed 30-byte packet documented in ../PROTOCOL.md
"""

from maix import app, camera, display, gpio, image, pinmap, time, uart
import struct

from three_circle_geometry import (
    SOURCE_FUSED,
    SOURCE_INFERRED,
    SOURCE_MEASURED,
    solve_three_targets,
)


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
COMMAND_REQUEST_TARGETS = 0x31
COMMAND_SELECT_MODE = 0xF2

CIRCLE_ROI = [0, 0, IMAGE_WIDTH, IMAGE_HEIGHT]
HOUGH_THRESHOLD = 1350
HOUGH_X_STRIDE = 4
HOUGH_Y_STRIDE = 4
HOUGH_X_MARGIN = 9
HOUGH_Y_MARGIN = 9
HOUGH_R_MARGIN = 4
HOUGH_R_MIN = 16
HOUGH_R_MAX = 72
HOUGH_R_STEP = 3
DETECT_EVERY_N_FRAMES = 4
MAX_HOUGH_CANDIDATES = 100
CENTER_CLUSTER_DISTANCE_PX = 8
MAX_OBSERVATIONS = 5
MAX_SOLUTION_MISSES = 5
SOLUTION_SMOOTH_ALPHA = 0.25

DIGIT_BLACK_THRESHOLD = 125
DIGIT_ROI_RADIUS_RATIO = 0.43

UART_RX_PIN = "A17"
UART_TX_PIN = "A16"
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200


# Thick sans-serif templates matching the supplied board. A global assignment
# later guarantees that one frame cannot contain duplicate target numbers.
DIGIT_TEMPLATES = {
    1: (
        "00111",
        "11111",
        "00111",
        "00111",
        "00011",
        "00011",
        "00011",
    ),
    2: (
        "01111",
        "11001",
        "10001",
        "00111",
        "01110",
        "11000",
        "11111",
    ),
    3: (
        "01111",
        "11001",
        "00001",
        "00111",
        "00001",
        "10001",
        "11111",
    ),
}

DIGIT_EXPECTED_ASPECT = {1: 0.34, 2: 0.66, 3: 0.58}


frame_sequence = 0
rx_buffer = bytearray()
latest_solution = None
solution_misses = 0
smoothed_targets = {}
responding = False


def median(values):
    values = sorted(values)
    middle = len(values) // 2
    if len(values) & 1:
        return values[middle]
    return (values[middle - 1] + values[middle]) / 2


def clamp_i16(value):
    return max(-32768, min(32767, int(round(value))))


def clamp_u8(value):
    return max(0, min(255, int(round(value))))


def crc16_modbus(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def make_response(solution):
    valid = solution is not None and solution.get("valid", False)
    targets = solution["targets"] if valid else []
    flags = 0x01 if valid else 0
    if any(item["source"] == SOURCE_INFERRED for item in targets):
        flags |= 0x02
    if any(item["source"] == SOURCE_FUSED for item in targets):
        flags |= 0x04
    quality = solution.get("quality", 0) if valid else 0

    result_sequence = solution.get("sequence", frame_sequence) if valid else frame_sequence
    body = bytearray(struct.pack(
        "<BHBB", COMMAND_REQUEST_TARGETS, result_sequence & 0xFFFF,
        flags, clamp_u8(quality),
    ))
    by_number = {item["number"]: item for item in targets}
    for number in (1, 2, 3):
        target = by_number.get(number)
        if target is None:
            body.extend(struct.pack("<BBhhB", number, 0, 0, 0, 0))
        else:
            body.extend(struct.pack(
                "<BBhhB", number, target["source"],
                clamp_i16(target["x"]), clamp_i16(target["y"]),
                clamp_u8(target["confidence"]),
            ))
    checksum = crc16_modbus(body)
    return bytes([0xAA]) + bytes(body) + struct.pack("<H", checksum) + bytes([0xBB])


def on_received(serial, data):
    global latest_solution, responding, solution_misses
    if not data:
        return
    rx_buffer.extend(data)
    while True:
        header = rx_buffer.find(b"\xAA")
        if header < 0:
            rx_buffer.clear()
            return
        if header:
            del rx_buffer[:header]
        if len(rx_buffer) < 4:
            return
        if rx_buffer[3] != 0xBB:
            del rx_buffer[0]
            continue
        command = rx_buffer[2]
        del rx_buffer[:4]
        if command == COMMAND_SELECT_MODE:
            # Do not let the controller accept a frame captured before the
            # chassis reached its final work-zone pose.
            latest_solution = None
            solution_misses = 0
            smoothed_targets.clear()
            print("Three-circle mode selected")
            continue
        if command != COMMAND_REQUEST_TARGETS or responding:
            continue
        responding = True
        try:
            serial.write(make_response(latest_solution))
        except Exception as error:
            print("UART response error:", error)
        finally:
            responding = False


def _gray_value(gray, x, y):
    if x < 0 or y < 0 or x >= IMAGE_WIDTH or y >= IMAGE_HEIGHT:
        return None
    value = gray.get_pixel(int(x), int(y))
    if isinstance(value, int):
        return value
    if isinstance(value, (tuple, list)):
        return int(value[0]) if value else None
    for name in ("luma", "gray", "value", "r"):
        try:
            field = getattr(value, name)
            return int(field() if callable(field) else field)
        except Exception:
            pass
    try:
        return int(value)
    except Exception:
        return None


def digit_scores(gray, center_x, center_y, radius):
    """Find the digit ink box, then compare its normalized 5x7 occupancy."""
    half = max(8, int(round(radius * DIGIT_ROI_RADIUS_RATIO)))
    minimum_x = IMAGE_WIDTH
    maximum_x = -1
    minimum_y = IMAGE_HEIGHT
    maximum_y = -1
    dark_samples = 0
    for y in range(int(center_y) - half, int(center_y) + half + 1, 2):
        for x in range(int(center_x) - half, int(center_x) + half + 1, 2):
            dx = x - center_x
            dy = y - center_y
            if dx * dx + dy * dy > half * half:
                continue
            pixel = _gray_value(gray, x, y)
            if pixel is None or pixel >= DIGIT_BLACK_THRESHOLD:
                continue
            minimum_x = min(minimum_x, x)
            maximum_x = max(maximum_x, x)
            minimum_y = min(minimum_y, y)
            maximum_y = max(maximum_y, y)
            dark_samples += 1

    box_width = maximum_x - minimum_x + 1
    box_height = maximum_y - minimum_y + 1
    if (dark_samples < 8 or box_width < 3 or
            box_height < max(8, int(radius * 0.32))):
        return {1: 0, 2: 0, 3: 0}

    occupancy = []
    valid_samples = 0
    total_samples = 0
    for row in range(7):
        row_values = []
        for column in range(5):
            x0 = minimum_x + box_width * column // 5
            x1 = minimum_x + box_width * (column + 1) // 5
            y0 = minimum_y + box_height * row // 7
            y1 = minimum_y + box_height * (row + 1) // 7
            dark = 0
            valid = 0
            for y in range(y0, max(y0 + 1, y1)):
                for x in range(x0, max(x0 + 1, x1)):
                    total_samples += 1
                    pixel = _gray_value(gray, x, y)
                    if pixel is None:
                        continue
                    valid += 1
                    valid_samples += 1
                    if pixel < DIGIT_BLACK_THRESHOLD:
                        dark += 1
            row_values.append(float(dark) / valid if valid else 0.0)
        occupancy.append(row_values)

    visible_ratio = float(valid_samples) / max(1, total_samples)
    aspect = float(box_width) / max(1, box_height)
    scores = {}
    for number, template in DIGIT_TEMPLATES.items():
        error = 0.0
        weight_sum = 0.0
        for row in range(7):
            for column in range(5):
                expected = 1.0 if template[row][column] == "1" else 0.0
                weight = 1.55 if expected else 1.0
                error += abs(occupancy[row][column] - expected) * weight
                weight_sum += weight
        similarity = max(0.0, 1.0 - error / weight_sum)
        aspect_penalty = min(
            35.0, abs(aspect - DIGIT_EXPECTED_ASPECT[number]) * 55.0
        )
        scores[number] = max(0, int(round(
            100.0 * similarity * visible_ratio - aspect_penalty
        )))
    return scores


def assign_unique_digits(observations):
    """Choose the best unique digit assignment over all observations."""
    if not observations:
        return
    # Only the three strongest circle observations can represent the board.
    selected = sorted(
        observations,
        key=lambda item: item["detection_confidence"],
        reverse=True,
    )[:3]
    assignments = []

    def search(index, used, labels, total):
        if index == len(selected):
            assignments.append((total, tuple(labels)))
            return
        for number in (1, 2, 3):
            if number in used:
                continue
            search(
                index + 1,
                used | {number},
                labels + [number],
                total + selected[index]["digit_scores"][number],
            )

    search(0, set(), [], 0)
    assignments.sort(key=lambda item: item[0], reverse=True)
    if not assignments:
        return
    best_total, labels = assignments[0]
    second_total = assignments[1][0] if len(assignments) > 1 else 0
    global_margin = max(0, best_total - second_total) / max(1, len(selected))
    for observation, number in zip(selected, labels):
        scores = observation["digit_scores"]
        alternatives = [scores[value] for value in (1, 2, 3) if value != number]
        local_margin = scores[number] - max(alternatives)
        confidence = (
            scores[number] * 0.68
            + max(0, local_margin) * 1.20
            + global_margin * 0.75
        )
        observation["label"] = number
        observation["label_confidence"] = clamp_u8(confidence)


def detect_circle_observations(gray):
    try:
        circles = gray.find_circles(
            roi=CIRCLE_ROI,
            x_stride=HOUGH_X_STRIDE,
            y_stride=HOUGH_Y_STRIDE,
            threshold=HOUGH_THRESHOLD,
            x_margin=HOUGH_X_MARGIN,
            y_margin=HOUGH_Y_MARGIN,
            r_margin=HOUGH_R_MARGIN,
            r_min=HOUGH_R_MIN,
            r_max=HOUGH_R_MAX,
            r_step=HOUGH_R_STEP,
        )
    except Exception as error:
        print("find_circles error:", error)
        return []

    candidates = []
    for circle in circles:
        try:
            candidates.append({
                "x": int(circle.x()),
                "y": int(circle.y()),
                "r": int(circle.r()),
                "magnitude": max(1, int(circle.magnitude())),
            })
        except Exception:
            pass
    candidates.sort(key=lambda item: item["magnitude"], reverse=True)
    candidates = candidates[:MAX_HOUGH_CANDIDATES]

    groups = []
    maximum_distance_squared = CENTER_CLUSTER_DISTANCE_PX ** 2
    for candidate in candidates:
        best_group = None
        best_distance = maximum_distance_squared + 1
        for group in groups:
            center_x = median([item["x"] for item in group])
            center_y = median([item["y"] for item in group])
            dx = candidate["x"] - center_x
            dy = candidate["y"] - center_y
            distance = dx * dx + dy * dy
            if distance <= maximum_distance_squared and distance < best_distance:
                best_group = group
                best_distance = distance
        if best_group is None:
            groups.append([candidate])
        else:
            best_group.append(candidate)

    observations = []
    for group in groups:
        center_x = median([item["x"] for item in group])
        center_y = median([item["y"] for item in group])
        radii = sorted(item["r"] for item in group)
        distinct_radii = []
        for radius in radii:
            if not distinct_radii or radius - distinct_radii[-1] >= 4:
                distinct_radii.append(radius)
        outer_radius = max(radii)
        average_magnitude = sum(item["magnitude"] for item in group) / len(group)
        confidence = min(
            100,
            18 + len(group) * 11 + len(distinct_radii) * 10
            + int(20 * average_magnitude / max(1, HOUGH_THRESHOLD)),
        )
        observation = {
            "x": center_x,
            "y": center_y,
            "radius": outer_radius,
            "detection_confidence": confidence,
            "label": 0,
            "label_confidence": 0,
        }
        observation["digit_scores"] = digit_scores(
            gray, center_x, center_y, outer_radius,
        )
        observations.append(observation)

    observations.sort(
        key=lambda item: (
            item["detection_confidence"],
            max(item["digit_scores"].values()),
        ),
        reverse=True,
    )
    observations = observations[:MAX_OBSERVATIONS]
    assign_unique_digits(observations)
    return observations


def smooth_solution(solution):
    if not solution.get("valid", False):
        return solution
    for target in solution["targets"]:
        number = target["number"]
        old = smoothed_targets.get(number)
        if old is None:
            smoothed_targets[number] = [target["x"], target["y"]]
        else:
            dx = target["x"] - old[0]
            dy = target["y"] - old[1]
            if abs(dx) > 35 or abs(dy) > 35:
                old[0] = target["x"]
                old[1] = target["y"]
            else:
                old[0] += SOLUTION_SMOOTH_ALPHA * dx
                old[1] += SOLUTION_SMOOTH_ALPHA * dy
        target["x"] = smoothed_targets[number][0]
        target["y"] = smoothed_targets[number][1]
    return solution


def draw_result(img, observations, solution):
    for observation in observations:
        cx = int(round(observation["x"]))
        cy = int(round(observation["y"]))
        radius = int(round(observation["radius"]))
        img.draw_circle(cx, cy, radius, image.COLOR_YELLOW, 1)
        if observation["label"]:
            img.draw_string(
                cx - 4, cy - 6, str(observation["label"]),
                scale=1.0, color=image.COLOR_YELLOW,
            )
    if solution is None or not solution.get("valid", False):
        return
    for target in solution["targets"]:
        cx = int(round(target["x"]))
        cy = int(round(target["y"]))
        if not (0 <= cx < IMAGE_WIDTH and 0 <= cy < IMAGE_HEIGHT):
            continue
        color = image.COLOR_GREEN
        if target["source"] == SOURCE_INFERRED:
            color = image.COLOR_BLUE
        elif target["source"] == SOURCE_FUSED:
            color = image.COLOR_RED
        img.draw_cross(cx, cy, color, size=8, thickness=2)
        img.draw_string(cx + 5, cy + 5, str(target["number"]), scale=1.0, color=color)


try:
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    disp = display.Display()
    pinmap.set_pin_function(UART_RX_PIN, "UART0_RX")
    pinmap.set_pin_function(UART_TX_PIN, "UART0_TX")
    stm32_uart = uart.UART(UART_DEVICE, UART_BAUD)
    stm32_uart.set_received_callback(on_received)
    pinmap.set_pin_function("B3", "GPIOB3")
    led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
    led.value(1)
    print("Three-circle detector ready")
except Exception as error:
    print("Initialization failed:", error)
    raise


try:
    observations = []
    while not app.need_exit():
        img = cam.read()
        frame_sequence = (frame_sequence + 1) & 0xFFFF
        if frame_sequence == 0:
            frame_sequence = 1

        if frame_sequence % DETECT_EVERY_N_FRAMES == 0:
            gray = img.to_format(image.Format.FMT_GRAYSCALE)
            observations = detect_circle_observations(gray)
            solution = smooth_solution(solve_three_targets(observations))
            if solution.get("valid", False):
                solution["sequence"] = frame_sequence
                latest_solution = solution
                solution_misses = 0
                led.value(0)
            else:
                solution_misses += 1
                if solution_misses >= MAX_SOLUTION_MISSES:
                    latest_solution = None
                    led.value(1)

        draw_result(img, observations, latest_solution)
        disp.show(img)
        time.sleep_ms(1)
except Exception as error:
    print("Main loop error:", error)
    raise
finally:
    print("Exiting three-circle detector")
