"""Combined MaixCam Pro colour and outer dashed-circle detector.

UART input is AA | reserved | command | BB.
Default mode is the original colour-block detector.

Commands:
  F0: switch to optimized outer dashed-circle mode
  F1: switch to original colour-block mode
  10: request the circle result (16-byte response from ColorRecognize)
  01..06: request the corresponding colour result (original 7-byte response)
"""

from maix import app, camera, display, gpio, image, pinmap, time, uart
import struct


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

MODE_COLOR = 1                 # Default: original main.py behaviour.
MODE_CIRCLE = 2
CMD_SET_CIRCLE_MODE = 0xF0
CMD_SET_COLOR_MODE = 0xF1
CMD_REQUEST_CIRCLE = 0x10

COLOR_THRESHOLDS = [
    ([0, 90, 30, 90, 0, 90], 1, "Red Block", image.COLOR_RED),
    ([30, 100, -64, -30, -32, 45], 2, "Green Block", image.COLOR_GREEN),
    ([5, 80, 10, 35, -60, -20], 3, "Blue Block", image.COLOR_BLUE),
    ([25, 55, -5, 15, -35, -5], 4, "LightBlue Block", image.Color(120, 180, 255)),
    ([50, 90, -15, 10, 40, 90], 5, "Yellow Block", image.COLOR_YELLOW),
    ([0, 45, -10, 10, -10, 10], 6, "Black Block", image.COLOR_BLACK),
]

# Optimized Hough settings for the largest dashed ring.
CIRCLE_ROI = [0, 0, IMAGE_WIDTH, IMAGE_HEIGHT]
HOUGH_THRESHOLD = 1500
HOUGH_X_STRIDE = 4
HOUGH_Y_STRIDE = 4
HOUGH_X_MARGIN = 10
HOUGH_Y_MARGIN = 10
HOUGH_R_MARGIN = 5
HOUGH_R_MIN = 25
HOUGH_R_MAX = 115
HOUGH_R_STEP = 3
DETECT_EVERY_N_FRAMES = 3
MAX_HOUGH_CANDIDATES = 80
CENTER_CLUSTER_DISTANCE_PX = 8
MIN_CONCENTRIC_CIRCLES = 2
SMOOTH_ALPHA = 0.18
MAX_NORMAL_CENTER_STEP = 12
MAX_NORMAL_RADIUS_STEP = 10
RELOCK_CONFIRMATIONS = 2
MAX_MISSED_DETECTIONS = 4

UART_RX_PIN = "A17"
UART_TX_PIN = "A16"
UART_RX_FUNCTION = "UART0_RX"
UART_TX_FUNCTION = "UART0_TX"
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200


current_mode = MODE_COLOR
latest_color_detections = []
latest_circle = None
frame_id = 0
rx_buffer = bytearray()
responding = False

filtered_center = None
filtered_radius = None
pending_detection = None
pending_count = 0
consecutive_misses = 0


def clamp_u16(value):
    return max(0, min(65535, int(round(value))))


def clamp_u8(value):
    return max(0, min(255, int(round(value))))


def make_circle_response(circle):
    """Keep the exact 16-byte ColorRecognize(2).py response format."""
    if circle is None:
        flags = 0
        response_frame_id = frame_id
        cx = cy = radius = confidence = 0
    else:
        flags = 1
        response_frame_id = circle["frame_id"]
        cx = clamp_u16(circle["cx"])
        cy = clamp_u16(circle["cy"])
        radius = clamp_u16(circle["radius"])
        confidence = clamp_u8(circle["confidence"])
    body = struct.pack(
        "<BBIHHHB", CMD_REQUEST_CIRCLE, flags, response_frame_id,
        cx, cy, radius, confidence,
    )
    checksum = 0
    for value in body:
        checksum ^= value
    return bytes([0xAA]) + body + bytes([checksum, 0xBB])


def write_color_response(serial, color_code=0, cx=0, cy=0):
    """Keep the original main.py 7-byte colour response format."""
    serial.write(struct.pack("<BBHHB", 0xAA, color_code, cx, cy, 0xBB))


def reset_circle_tracker():
    global latest_circle, filtered_center, filtered_radius
    global pending_detection, pending_count, consecutive_misses
    latest_circle = None
    filtered_center = None
    filtered_radius = None
    pending_detection = None
    pending_count = 0
    consecutive_misses = 0


def handle_command(serial, command):
    global current_mode
    if command == CMD_SET_CIRCLE_MODE:
        current_mode = MODE_CIRCLE
        reset_circle_tracker()
        print("Mode: outer dashed circle")
        return
    if command == CMD_SET_COLOR_MODE:
        current_mode = MODE_COLOR
        print("Mode: colour blocks")
        return

    if command == CMD_REQUEST_CIRCLE:
        serial.write(make_circle_response(
            latest_circle if current_mode == MODE_CIRCLE else None
        ))
        return

    if command in (1, 2, 3, 4, 5, 6):
        if current_mode != MODE_COLOR:
            write_color_response(serial)
            return
        for detection in reversed(latest_color_detections):
            if detection["color"] == command:
                write_color_response(serial, command, detection["cx"], detection["cy"])
                return
        write_color_response(serial)


def on_received(serial, data):
    """Process split or concatenated UART command frames."""
    global responding
    if not data:
        return
    rx_buffer.extend(data)
    while True:
        header_index = rx_buffer.find(b"\xAA")
        if header_index < 0:
            rx_buffer.clear()
            return
        if header_index:
            del rx_buffer[:header_index]
        if len(rx_buffer) < 4:
            return
        if rx_buffer[3] != 0xBB:
            del rx_buffer[0]
            continue

        command = rx_buffer[2]
        del rx_buffer[:4]
        if responding:
            continue
        responding = True
        try:
            handle_command(serial, command)
        except Exception as error:
            print("UART command error:", error)
        finally:
            responding = False


def median(values):
    values = sorted(values)
    middle = len(values) // 2
    if len(values) & 1:
        return values[middle]
    return (values[middle - 1] + values[middle]) / 2


def find_concentric_center(gray):
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
        return None

    candidates = []
    for circle in circles:
        try:
            candidates.append({
                "x": int(circle.x()), "y": int(circle.y()),
                "r": int(circle.r()),
                "magnitude": max(1, int(circle.magnitude())),
            })
        except Exception:
            pass
    if len(candidates) < MIN_CONCENTRIC_CIRCLES:
        return None

    candidates.sort(key=lambda item: item["magnitude"], reverse=True)
    candidates = candidates[:MAX_HOUGH_CANDIDATES]
    distance_squared = CENTER_CLUSTER_DISTANCE_PX ** 2
    best_group = []
    best_score = -1
    for seed in candidates:
        group = []
        for candidate in candidates:
            dx = candidate["x"] - seed["x"]
            dy = candidate["y"] - seed["y"]
            if dx * dx + dy * dy <= distance_squared:
                group.append(candidate)
        score = len(group) * 1000000 + sum(item["magnitude"] for item in group)
        if score > best_score:
            best_group = group
            best_score = score
    if len(best_group) < MIN_CONCENTRIC_CIRCLES:
        return None

    outer_circle = max(best_group, key=lambda item: (item["r"], item["magnitude"]))
    average_magnitude = sum(item["magnitude"] for item in best_group) / len(best_group)
    confidence = min(
        100,
        min(60, len(best_group) * 15)
        + min(40, int(40 * average_magnitude / max(1, HOUGH_THRESHOLD))),
    )
    return {
        "cx": median([item["x"] for item in best_group]),
        "cy": median([item["y"] for item in best_group]),
        "radius": outer_circle["r"],
        "confidence": confidence,
        "circle_count": len(best_group),
    }


def update_circle_tracker(detection):
    global consecutive_misses, filtered_center, filtered_radius
    global pending_detection, pending_count
    if detection is None:
        consecutive_misses += 1
        pending_detection = None
        pending_count = 0
        if consecutive_misses >= MAX_MISSED_DETECTIONS:
            filtered_center = None
            filtered_radius = None
        return None

    consecutive_misses = 0
    if filtered_center is None:
        filtered_center = [detection["cx"], detection["cy"]]
        filtered_radius = detection["radius"]
        pending_detection = None
        pending_count = 0
    else:
        dx = detection["cx"] - filtered_center[0]
        dy = detection["cy"] - filtered_center[1]
        dr = detection["radius"] - filtered_radius
        if (abs(dx) > MAX_NORMAL_CENTER_STEP or
                abs(dy) > MAX_NORMAL_CENTER_STEP or
                abs(dr) > MAX_NORMAL_RADIUS_STEP):
            if pending_detection is not None and (
                    abs(detection["cx"] - pending_detection["cx"]) <= CENTER_CLUSTER_DISTANCE_PX and
                    abs(detection["cy"] - pending_detection["cy"]) <= CENTER_CLUSTER_DISTANCE_PX and
                    abs(detection["radius"] - pending_detection["radius"]) <= MAX_NORMAL_RADIUS_STEP):
                pending_count += 1
            else:
                pending_detection = detection
                pending_count = 1
            if pending_count < RELOCK_CONFIRMATIONS:
                return None
            filtered_center = [detection["cx"], detection["cy"]]
            filtered_radius = detection["radius"]
            pending_detection = None
            pending_count = 0
        else:
            pending_detection = None
            pending_count = 0
            filtered_center[0] += SMOOTH_ALPHA * dx
            filtered_center[1] += SMOOTH_ALPHA * dy
            filtered_radius += SMOOTH_ALPHA * dr

    result = dict(detection)
    result["cx"] = filtered_center[0]
    result["cy"] = filtered_center[1]
    result["radius"] = filtered_radius
    return result


def update_colour_mode(img):
    global latest_color_detections
    detections = []
    for threshold, color_code, color_name, draw_color in COLOR_THRESHOLDS:
        blobs = img.find_blobs([threshold], area_threshold=1000, pixels_threshold=1000)
        for blob in blobs:
            cx = int(blob.x() + blob.w() // 2)
            cy = int(blob.y() + blob.h() // 2)
            detections.append({"color": color_code, "cx": cx, "cy": cy})
            radius = (blob.w() + blob.h()) // 4
            img.draw_circle(cx, cy, radius, draw_color, 2)
            img.draw_line(cx - 10, cy, cx + 10, cy, image.COLOR_WHITE, 2)
            img.draw_line(cx, cy - 10, cx, cy + 10, image.COLOR_WHITE, 2)
            img.draw_string(cx - 20, cy - 20, color_name, scale=1.0, color=draw_color)
    latest_color_detections = detections


def update_circle_mode(img):
    global latest_circle
    if frame_id % DETECT_EVERY_N_FRAMES == 0:
        gray = img.to_format(image.Format.FMT_GRAYSCALE)
        detection = update_circle_tracker(find_concentric_center(gray))
        if detection is not None:
            detection["frame_id"] = frame_id
            latest_circle = detection
        elif consecutive_misses >= MAX_MISSED_DETECTIONS:
            latest_circle = None

    if latest_circle is not None:
        cx = int(round(latest_circle["cx"]))
        cy = int(round(latest_circle["cy"]))
        radius = int(round(latest_circle["radius"]))
        img.draw_circle(cx, cy, radius, image.COLOR_GREEN, 2)
        img.draw_cross(cx, cy, image.COLOR_RED, size=10, thickness=2)


try:
    print("Initializing combined MaixCAM detector...")
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
    disp = display.Display()
    pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNCTION)
    pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION)
    stm32_uart = uart.UART(UART_DEVICE, UART_BAUD)
    stm32_uart.set_received_callback(on_received)
    pinmap.set_pin_function("B3", "GPIOB3")
    led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
    led.value(1)
except Exception as error:
    print("Initialization failed:", error)
    raise


try:
    print("Starting in original colour-block mode...")
    while not app.need_exit():
        img = cam.read()
        frame_id = (frame_id + 1) & 0xFFFFFFFF
        if frame_id == 0:
            frame_id = 1
        if current_mode == MODE_COLOR:
            update_colour_mode(img)
        else:
            update_circle_mode(img)
        disp.show(img)
        time.sleep_ms(1)
except Exception as error:
    print("Main loop error:", error)
    raise
finally:
    print("Exiting combined detector.")
