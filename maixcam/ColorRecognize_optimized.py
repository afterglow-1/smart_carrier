"""Low-latency concentric dashed-circle detector for MaixCam Pro.

The UART protocol is intentionally identical to ColorRecognize(2).py:
request AA EE 10 BB, response is a 16-byte frame beginning AA 10.
"""

from maix import app, camera, display, gpio, image, pinmap, time, uart
import struct


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# Hough runs in native code, but executing it on every display frame is still
# expensive.  These settings reduce the search volume while retaining enough
# dashed-ring arcs for stable centre estimation.
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

# Spatial and temporal rejection settings.  Change these only after testing
# with the mounted camera: they are expressed in 320x240 image pixels.
CENTER_CLUSTER_DISTANCE_PX = 8
MIN_CONCENTRIC_CIRCLES = 2
SMOOTH_ALPHA = 0.18
MAX_NORMAL_CENTER_STEP = 12
MAX_NORMAL_RADIUS_STEP = 10
RELOCK_CONFIRMATIONS = 2
MAX_MISSED_DETECTIONS = 4
DRAW_DEBUG_TEXT = False


REQUEST_FRAME = bytes([0xAA, 0xEE, 0x10, 0xBB])
RESPONSE_TYPE = 0x10
RESPONSE_DETECTED_FLAG = 0x01

UART_RX_PIN = "A17"
UART_TX_PIN = "A16"
UART_RX_FUNCTION = "UART0_RX"
UART_TX_FUNCTION = "UART0_TX"
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200


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


def make_response(circle):
    if circle is None:
        flags = 0
        response_frame_id = frame_id
        cx = cy = radius = confidence = 0
    else:
        flags = RESPONSE_DETECTED_FLAG
        response_frame_id = circle["frame_id"]
        cx = clamp_u16(circle["cx"])
        cy = clamp_u16(circle["cy"])
        radius = clamp_u16(circle["radius"])
        confidence = clamp_u8(circle["confidence"])

    body = struct.pack(
        "<BBIHHHB", RESPONSE_TYPE, flags, response_frame_id,
        cx, cy, radius, confidence,
    )
    checksum = 0
    for value in body:
        checksum ^= value
    return bytes([0xAA]) + body + bytes([checksum, 0xBB])


def on_received(serial, data):
    """Accept split or concatenated STM32 request frames."""
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
        if bytes(rx_buffer[:4]) != REQUEST_FRAME:
            del rx_buffer[0]
            continue

        del rx_buffer[:4]
        if responding:
            continue
        responding = True
        try:
            # The main loop replaces latest_circle atomically with a new dict.
            serial.write(make_response(latest_circle))
        except Exception as error:
            print("UART response error:", error)
        finally:
            responding = False


def median(values):
    ordered = sorted(values)
    length = len(ordered)
    middle = length // 2
    if length & 1:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2


def candidate_score(group):
    # A real target gives several rings at nearly one centre.  Count is more
    # robust than one unusually strong edge from an occluder.
    return len(group) * 1000000 + sum(item["magnitude"] for item in group)


def find_concentric_center(gray):
    """Find the strongest concentric group and return its robust centre.

    Candidate count is capped before the O(n^2) cluster step.  The coordinate
    median prevents one high-magnitude partial edge from moving the centre.
    """
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
                "x": int(circle.x()),
                "y": int(circle.y()),
                "r": int(circle.r()),
                "magnitude": max(1, int(circle.magnitude())),
            })
        except Exception:
            pass

    if len(candidates) < MIN_CONCENTRIC_CIRCLES:
        return None

    # Sorting once also makes the cap deterministic across frames.
    candidates.sort(key=lambda item: item["magnitude"], reverse=True)
    candidates = candidates[:MAX_HOUGH_CANDIDATES]

    maximum_distance_squared = CENTER_CLUSTER_DISTANCE_PX ** 2
    best_group = []
    best_score = -1
    for seed in candidates:
        group = []
        for candidate in candidates:
            dx = candidate["x"] - seed["x"]
            dy = candidate["y"] - seed["y"]
            if dx * dx + dy * dy <= maximum_distance_squared:
                group.append(candidate)
        score = candidate_score(group)
        if score > best_score:
            best_group = group
            best_score = score

    if len(best_group) < MIN_CONCENTRIC_CIRCLES:
        return None

    center_x = median([item["x"] for item in best_group])
    center_y = median([item["y"] for item in best_group])
    # The required target is the outermost dashed circle, not the median ring.
    outer_circle = max(best_group, key=lambda item: (item["r"], item["magnitude"]))
    average_magnitude = sum(item["magnitude"] for item in best_group) / len(best_group)
    confidence = min(
        100,
        min(60, len(best_group) * 15)
        + min(40, int(40 * average_magnitude / max(1, HOUGH_THRESHOLD))),
    )
    return {
        "cx": center_x,
        "cy": center_y,
        "radius": outer_circle["r"],
        "confidence": confidence,
        "circle_count": len(best_group),
    }


def update_tracker(detection):
    """Filter jitter and require two observations before accepting a large jump."""
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
            if pending_detection is not None:
                pdx = detection["cx"] - pending_detection["cx"]
                pdy = detection["cy"] - pending_detection["cy"]
                pdr = detection["radius"] - pending_detection["radius"]
                if (abs(pdx) <= CENTER_CLUSTER_DISTANCE_PX and
                        abs(pdy) <= CENTER_CLUSTER_DISTANCE_PX and
                        abs(pdr) <= MAX_NORMAL_RADIUS_STEP):
                    pending_count += 1
                else:
                    pending_detection = detection
                    pending_count = 1
            else:
                pending_detection = detection
                pending_count = 1

            if pending_count < RELOCK_CONFIRMATIONS:
                return None
            # A consistently displaced target is genuine movement, not noise.
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


try:
    print("Initializing optimized MaixCAM circle detector...")
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
    while not app.need_exit():
        img = cam.read()
        frame_id = (frame_id + 1) & 0xFFFFFFFF
        if frame_id == 0:
            frame_id = 1

        if frame_id % DETECT_EVERY_N_FRAMES == 0:
            gray = img.to_format(image.Format.FMT_GRAYSCALE)
            detection = update_tracker(find_concentric_center(gray))
            if detection is not None:
                detection["frame_id"] = frame_id
                latest_circle = detection
            elif consecutive_misses >= MAX_MISSED_DETECTIONS:
                latest_circle = None

        if latest_circle is None:
            led.value(1)
            if DRAW_DEBUG_TEXT:
                img.draw_string(5, 5, "CIRCLE: NONE", scale=1.0, color=image.COLOR_RED)
        else:
            led.value(0)
            cx = int(round(latest_circle["cx"]))
            cy = int(round(latest_circle["cy"]))
            radius = int(round(latest_circle["radius"]))
            img.draw_circle(cx, cy, radius, image.COLOR_GREEN, 2)
            img.draw_cross(cx, cy, image.COLOR_RED, size=10, thickness=2)
            if DRAW_DEBUG_TEXT:
                img.draw_string(
                    5, 5, "CIRCLE ({},{}) r={} n={} q={}".format(
                        cx, cy, radius, latest_circle["circle_count"],
                        latest_circle["confidence"],
                    ), scale=1.0, color=image.COLOR_GREEN,
                )

        disp.show(img)
        time.sleep_ms(1)
except Exception as error:
    print("Main loop error:", error)
    raise
finally:
    print("Exiting optimized circle detector.")
