from maix import app, camera, display, gpio, image, pinmap, time, uart
import struct


# ---------------------------------------------------------------------------
# 图像与霍夫圆参数
# ---------------------------------------------------------------------------
# 霍夫圆的计算量与像素数和半径搜索层数都有关。相机直接输出160x120，
# 识别后再按2倍换算为原来的320x240协议坐标，STM32端无需修改标定参数。
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120
CAMERA_FPS = 30
PROTOCOL_COORDINATE_SCALE = 2.0

# 霍夫识别和屏幕显示分别限频。主循环仍持续读取相机，避免预览画面停顿。
DETECTION_INTERVAL_MS = 70
DISPLAY_INTERVAL_MS = 40

# 目标圆必须完整落入ROI。现场调试时优先调整半径和threshold，不要改串口协议。
CIRCLE_ROI = [0, 0, IMAGE_WIDTH, IMAGE_HEIGHT]
HOUGH_THRESHOLD = 700
HOUGH_X_STRIDE = 2
HOUGH_Y_STRIDE = 2
HOUGH_X_MARGIN = 5
HOUGH_Y_MARGIN = 5
HOUGH_R_MARGIN = 2
HOUGH_R_MIN = 12
HOUGH_R_MAX = 58
HOUGH_R_STEP = 2

# 同心圆图案会产生多个半径不同、圆心相近的霍夫候选。
# 将圆心距离不超过该值的候选归为同一组，再用幅值加权求中心。
CENTER_CLUSTER_DISTANCE_PX = 5
MIN_CONCENTRIC_CIRCLES = 2
CENTER_FILTER_ALPHA = 0.35
CLEAR_FILTER_AFTER_MISSES = 3


# ---------------------------------------------------------------------------
# MaixCAM <-> STM32 UART协议
# ---------------------------------------------------------------------------
# STM32请求：AA EE 10 BB
REQUEST_FRAME = bytes([0xAA, 0xEE, 0x10, 0xBB])

# MaixCAM响应，共16字节，小端序：
# AA | 10 | flags | frame_id(4B) | cx(2B) | cy(2B) | radius(2B)
#    | confidence(1B) | XOR(1B) | BB
# flags bit0：1=检测到圆；0=本帧未检测到圆。
RESPONSE_TYPE = 0x10
RESPONSE_DETECTED_FLAG = 0x01

# 保留原ColorRecognize.py的UART0接线，避免改变现有硬件：
# MaixCAM A16(TX) -> STM32 PE7(RX)
# MaixCAM A17(RX) <- STM32 PE8(TX)
# 若UART0受系统日志影响，可改为A18/UART1_RX、A19/UART1_TX和/dev/ttyS1。
UART_RX_PIN = "A17"
UART_TX_PIN = "A16"
UART_RX_FUNCTION = "UART0_RX"
UART_TX_FUNCTION = "UART0_TX"
UART_DEVICE = "/dev/ttyS0"
UART_BAUD = 115200


latest_circle = None
display_circle = None
frame_id = 0
rx_buffer = bytearray()
responding = False
filtered_center = None
consecutive_misses = 0
last_detection_ms = -DETECTION_INTERVAL_MS
last_display_ms = -DISPLAY_INTERVAL_MS


def clamp_u16(value):
    return max(0, min(65535, int(round(value))))


def clamp_u8(value):
    return max(0, min(255, int(round(value))))


def make_response(circle):
    if circle is None:
        flags = 0
        response_frame_id = frame_id
        cx = 0
        cy = 0
        radius = 0
        confidence = 0
    else:
        flags = RESPONSE_DETECTED_FLAG
        response_frame_id = circle["frame_id"]
        # 协议继续使用等效320x240坐标，避免STM32端的目标中心和增益跟随改变。
        cx = clamp_u16(circle["cx"] * PROTOCOL_COORDINATE_SCALE)
        cy = clamp_u16(circle["cy"] * PROTOCOL_COORDINATE_SCALE)
        radius = clamp_u16(circle["radius"] * PROTOCOL_COORDINATE_SCALE)
        confidence = clamp_u8(circle["confidence"])

    body = struct.pack(
        "<BBIHHHB",
        RESPONSE_TYPE,
        flags,
        response_frame_id,
        cx,
        cy,
        radius,
        confidence,
    )
    checksum = 0
    for value in body:
        checksum ^= value
    return bytes([0xAA]) + body + bytes([checksum, 0xBB])


def on_received(serial, data):
    """处理可能被拆包或粘包的STM32请求，并返回最新一帧识别结果。"""
    global responding

    if not data:
        return
    rx_buffer.extend(data)

    while True:
        header_index = rx_buffer.find(b"\xAA")
        if header_index < 0:
            rx_buffer.clear()
            return
        if header_index > 0:
            del rx_buffer[:header_index]
        if len(rx_buffer) < len(REQUEST_FRAME):
            return

        if bytes(rx_buffer[:4]) == REQUEST_FRAME:
            del rx_buffer[:4]
            if responding:
                continue
            responding = True
            try:
                # latest_circle采用整体对象替换，回调读取时不会看到半更新的数据。
                serial.write(make_response(latest_circle))
            except Exception as error:
                print("UART response error:", error)
            finally:
                responding = False
        else:
            # 当前0xAA不是有效请求帧头，丢弃后继续寻找下一帧。
            del rx_buffer[0]


def cluster_score(group):
    magnitude_sum = sum(item["magnitude"] for item in group)
    # 首先偏好同心候选数量多的组，其次比较霍夫幅值。
    return len(group) * 1000000 + magnitude_sum


def find_concentric_center(gray):
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

    candidates = []
    for circle in circles:
        candidates.append(
            {
                "x": int(circle.x()),
                "y": int(circle.y()),
                "r": int(circle.r()),
                "magnitude": max(1, int(circle.magnitude())),
            }
        )

    if not candidates:
        return None

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
        score = cluster_score(group)
        if score > best_score:
            best_score = score
            best_group = group

    if len(best_group) < MIN_CONCENTRIC_CIRCLES:
        return None

    total_weight = sum(item["magnitude"] for item in best_group)
    center_x = (
        sum(item["x"] * item["magnitude"] for item in best_group) / total_weight
    )
    center_y = (
        sum(item["y"] * item["magnitude"] for item in best_group) / total_weight
    )

    radii = sorted(item["r"] for item in best_group)
    representative_radius = radii[len(radii) // 2]
    average_magnitude = total_weight / len(best_group)

    # 置信度同时考虑同心圆数量和霍夫幅值，范围限制为0~100。
    count_score = min(60, len(best_group) * 12)
    magnitude_score = min(
        40, int(40 * average_magnitude / max(1, HOUGH_THRESHOLD))
    )
    confidence = min(100, count_score + magnitude_score)

    return {
        "cx": center_x,
        "cy": center_y,
        "radius": representative_radius,
        "confidence": confidence,
        "circle_count": len(best_group),
    }


def filter_detection(detection):
    global filtered_center, consecutive_misses

    if detection is None:
        consecutive_misses += 1
        if consecutive_misses >= CLEAR_FILTER_AFTER_MISSES:
            filtered_center = None
        return None

    consecutive_misses = 0
    if filtered_center is None:
        filtered_center = [detection["cx"], detection["cy"]]
    else:
        filtered_center[0] += CENTER_FILTER_ALPHA * (
            detection["cx"] - filtered_center[0]
        )
        filtered_center[1] += CENTER_FILTER_ALPHA * (
            detection["cy"] - filtered_center[1]
        )

    result = dict(detection)
    result["cx"] = filtered_center[0]
    result["cy"] = filtered_center[1]
    return result


try:
    print("Initializing MaixCAM Hough-circle detector...")
    cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT, fps=CAMERA_FPS)
    disp = display.Display()

    pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNCTION)
    pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION)
    stm32_uart = uart.UART(UART_DEVICE, UART_BAUD)
    stm32_uart.set_received_callback(on_received)

    pinmap.set_pin_function("B3", "GPIOB3")
    led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
    led.value(1)
    print("Camera, display and STM32 UART initialized.")
except Exception as error:
    print("Initialization failed:", error)
    raise


try:
    while not app.need_exit():
        img = cam.read()
        now_ms = time.ticks_ms()

        # 霍夫搜索只在固定周期执行；没有执行搜索的循环继续刷新相机帧。
        if now_ms - last_detection_ms >= DETECTION_INTERVAL_MS:
            last_detection_ms = now_ms
            gray = img.to_format(image.Format.FMT_GRAYSCALE)
            detection = filter_detection(find_concentric_center(gray))

            frame_id = (frame_id + 1) & 0xFFFFFFFF
            if frame_id == 0:
                frame_id = 1

            if detection is None:
                latest_circle = None
                display_circle = None
                led.value(1)
            else:
                detection["frame_id"] = frame_id
                latest_circle = detection
                display_circle = detection
                led.value(0)

                if frame_id % 20 == 0:
                    print(
                        "circle center=({}, {}) radius={} members={} confidence={}".format(
                            int(round(detection["cx"])),
                            int(round(detection["cy"])),
                            int(round(detection["radius"])),
                            detection["circle_count"],
                            detection["confidence"],
                        )
                    )

        # 绘制也限频，减少文字和图形叠加对识别周期的影响。
        if now_ms - last_display_ms >= DISPLAY_INTERVAL_MS:
            last_display_ms = now_ms
            if display_circle is None:
                img.draw_string(
                    3, 3, "CIRCLE: NONE", scale=1.0, color=image.COLOR_RED
                )
            else:
                cx = int(round(display_circle["cx"]))
                cy = int(round(display_circle["cy"]))
                radius = int(round(display_circle["radius"]))
                img.draw_circle(cx, cy, radius, image.COLOR_GREEN, 1)
                img.draw_cross(
                    cx, cy, image.COLOR_RED, size=6, thickness=1
                )
                img.draw_string(
                    3,
                    3,
                    "C ({},{}) r={} n={} q={}".format(
                        cx,
                        cy,
                        radius,
                        display_circle["circle_count"],
                        display_circle["confidence"],
                    ),
                    scale=1.0,
                    color=image.COLOR_GREEN,
                )
            disp.show(img)

        time.sleep_ms(1)

except Exception as error:
    print("Main loop error:", error)
    raise
finally:
    print("Exiting Hough-circle detector.")
