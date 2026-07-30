"""H-topic K230 steel-ball position sender with RTSP AP preview.

Copy this file and ``steel_ball_yolov8n_320.kmodel`` to
``/sdcard/steel_ball/``. Run it as ``/sdcard/steel_ball/main.py`` when the
camera is mounted above the 25 cm PPR rod.

UART line format:
  BALL,found,pos_cm_x10,target_cm_x10,err_cm_x10,score,status

Example:
  BALL,1,48,50,2,82,BALL
"""

import gc
import math
import os
import sys

try:
    from libs.WBCRtsp import WBCRtsp
    WBCRTSP_IMPORT_ERROR = None
except BaseException as import_error:
    WBCRtsp = None
    WBCRTSP_IMPORT_ERROR = import_error


MODEL_PATH = "/sdcard/steel_ball/steel_ball_yolov8n_320_round3.kmodel"
MODEL_PATH_FALLBACKS = (
    MODEL_PATH,
    "/sdcard/steel_ball/best.kmodel",
    "/sdcard/steel_ball_yolov8n_320.kmodel",
    "/sdcard/best.kmodel",
)
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]

SENSOR_ID = 2
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None
RGB888P_SIZE = [640, 360]
H_MIRROR = False
V_FLIP = False

CONFIDENCE_THRESHOLD = 0.40
NMS_THRESHOLD = 0.45
MAX_BOXES = 50

# Calibrate these three points from the on-screen image after final mounting.
# Coordinates use the same pixel space as the LCD overlay after mirror/flip.
ZERO_PX = (404, 242)
MINUS_5CM_PX = (258,236)
PLUS_5CM_PX = (554,238)
HALF_RANGE_CM = 5

TARGET_CM = 0.0
TARGET_HOLD_MS = 500
PRINT_EVERY_N_FRAMES = 5
GC_EVERY_N_FRAMES = 20

UART_ENABLED = True
UART_TX_GPIO = 11
UART_RX_GPIO = 12
UART_BAUDRATE = 115200

AP_SSID = "K230_STEEL_BALL"
AP_KEY = "12345678"
AP_START_TIMEOUT_MS = 5000
CLIENT_STATUS_EVERY_MS = 2000

RTSP_PORT = 8554
RTSP_SESSION = "test"
WBC_WIDTH = 640
WBC_HEIGHT = 360
VLC_NETWORK_CACHING_MS = 500
BOOT_LOG_PATH = "/sdcard/steel_ball/main_h_position_rtsp_ap_boot.log"
SCRIPT_VERSION = "main_h_position_rtsp_ap_v1"

# Display-space filter. None means full-width, center-third-height ROI.
# Set to (x, y, w, h) to override after final mounting.
PIPE_ROI = None


def log(*parts):
    msg = " ".join([str(part) for part in parts])
    print(msg)
    try:
        with open(BOOT_LOG_PATH, "a") as log_file:
            log_file.write(msg + "\n")
    except BaseException:
        pass


def start_ap():
    import network
    import time

    log("[h_position_rtsp] ap init:", AP_SSID)
    try:
        sta = network.WLAN(network.STA_IF)
        sta.active(False)
    except BaseException:
        pass

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(ssid=AP_SSID, key=AP_KEY)

    start = time.ticks_ms()
    ip = ap.ifconfig()[0]
    while (ip is None or ip == "0.0.0.0") and time.ticks_diff(time.ticks_ms(), start) < AP_START_TIMEOUT_MS:
        time.sleep_ms(100)
        ip = ap.ifconfig()[0]

    log("[h_position_rtsp] ap active:", ap.active())
    log("[h_position_rtsp] ap config:", ap.ifconfig())
    if ip is None or ip == "0.0.0.0":
        return ap, None
    return ap, ip


def get_client_count(ap):
    try:
        return len(ap.status("stations"))
    except BaseException:
        return -1


def start_rtsp(ip):
    if WBCRtsp is None:
        raise RuntimeError("libs.WBCRtsp import failed: %s" % WBCRTSP_IMPORT_ERROR)

    rtsp_url = "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION)
    log("[h_position_rtsp] ap ssid:", AP_SSID)
    log("[h_position_rtsp] ap key:", AP_KEY)
    log("[h_position_rtsp] rtsp url:", rtsp_url)
    log(
        "[h_position_rtsp] vlc tcp: vlc --rtsp-tcp --network-caching=%d %s"
        % (VLC_NETWORK_CACHING_MS, rtsp_url)
    )
    log("[h_position_rtsp] wbc: %dx%d" % (WBC_WIDTH, WBC_HEIGHT))
    WBCRtsp.configure(wbc_width=WBC_WIDTH, wbc_height=WBC_HEIGHT)
    WBCRtsp.start()
    log("[h_position_rtsp] rtsp started")
    return rtsp_url


def make_active_roi(display_size):
    if PIPE_ROI is not None:
        return PIPE_ROI

    width = int(display_size[0])
    height = int(display_size[1])
    roi_h = height // 3
    roi_y = (height - roi_h) // 2
    return (0, roi_y, width, roi_h)


def make_calibration(zero_px, minus_px, plus_px, half_range_cm):
    dx = plus_px[0] - minus_px[0]
    dy = plus_px[1] - minus_px[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length <= 0:
        raise ValueError("bad calibration marks")
    # ponytail: fixed-camera two-mark scale; use homography only if the mount moves.
    return (
        float(zero_px[0]),
        float(zero_px[1]),
        dx / length,
        dy / length,
        (2.0 * float(half_range_cm)) / length,
    )


def project_position_cm(center, calibration):
    zero_x, zero_y, axis_x, axis_y, cm_per_px = calibration
    px = float(center[0]) - zero_x
    py = float(center[1]) - zero_y
    return (px * axis_x + py * axis_y) * cm_per_px


def pixel_from_cm(pos_cm, calibration):
    zero_x, zero_y, axis_x, axis_y, cm_per_px = calibration
    offset_px = float(pos_cm) / cm_per_px
    return (
        int(round(zero_x + axis_x * offset_px)),
        int(round(zero_y + axis_y * offset_px)),
    )


def cm_to_x10(value):
    return int(round(float(value) * 10.0))


def select_model_path():
    for path in MODEL_PATH_FALLBACKS:
        try:
            os.stat(path)
            return path
        except OSError:
            pass
    raise OSError(
        "Kmodel file not exist. Copy steel_ball_yolov8n_320.kmodel to "
        "/sdcard/steel_ball/ or rename your model to /sdcard/steel_ball/best.kmodel"
    )


def get_best_detection(result, roi):
    count = 0
    best_score = -1.0
    best_center = None

    if result and len(result[0]) > 0:
        count = len(result[0])
        for index in range(count):
            x, y, width, height = result[0][index]
            center_x = int(round(x + width / 2))
            center_y = int(round(y + height / 2))
            if not point_in_roi(center_x, center_y, roi):
                continue
            score = float(result[2][index])
            if score > best_score:
                best_score = score
                best_center = (center_x, center_y)

    if best_center is None:
        count = 0
    return count, best_center, best_score


def point_in_roi(x, y, roi):
    if roi is None:
        return True
    roi_x, roi_y, roi_w, roi_h = roi
    return roi_x <= x < roi_x + roi_w and roi_y <= y < roi_y + roi_h


def init_uart():
    if not UART_ENABLED:
        return None

    from machine import FPIOA
    from machine import UART

    fpioa = FPIOA()
    fpioa.set_function(UART_TX_GPIO, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_GPIO, FPIOA.UART2_RXD)
    return UART(UART.UART2, UART_BAUDRATE)


def send_position(uart, found, pos_cm, target_cm, score, status):
    if found:
        pos_x10 = cm_to_x10(pos_cm)
        target_x10 = cm_to_x10(target_cm)
        err_x10 = cm_to_x10(target_cm - pos_cm)
        score_i = int(round(score * 100.0))
    else:
        pos_x10 = 0
        target_x10 = cm_to_x10(target_cm)
        err_x10 = 0
        score_i = 0

    # ponytail: ASCII UART for bring-up; switch to binary after MCU parsing works.
    line = "BALL,%d,%d,%d,%d,%d,%s\n" % (
        1 if found else 0,
        pos_x10,
        target_x10,
        err_x10,
        score_i,
        status,
    )
    if uart is not None:
        uart.write(line)
    return line


def draw_overlay(osd_image, center, pos_cm, target_cm, score, status, calibration, roi):
    minus_px = pixel_from_cm(-HALF_RANGE_CM, calibration)
    zero_px = pixel_from_cm(0.0, calibration)
    plus_px = pixel_from_cm(HALF_RANGE_CM, calibration)
    target_px = pixel_from_cm(target_cm, calibration)

    if roi is not None:
        roi_x, roi_y, roi_w, roi_h = roi
        osd_image.draw_rectangle(roi_x, roi_y, roi_w, roi_h, color=(0, 160, 255), thickness=2)

    osd_image.draw_line(
        minus_px[0], minus_px[1], plus_px[0], plus_px[1], color=(0, 220, 255), thickness=2
    )
    osd_image.draw_cross(zero_px[0], zero_px[1], color=(255, 255, 255), size=14, thickness=2)
    osd_image.draw_cross(target_px[0], target_px[1], color=(255, 80, 80), size=14, thickness=2)
    osd_image.draw_string_advanced(5, 5, 22, "H ball pos", color=(0, 255, 0))

    if center is None:
        osd_image.draw_string_advanced(5, 34, 20, "status: LOST", color=(255, 80, 80))
        return

    err_cm = target_cm - pos_cm
    osd_image.draw_cross(center[0], center[1], color=(255, 255, 0), size=12, thickness=3)
    osd_image.draw_string_advanced(
        5, 34, 20, "pos:%+.1fcm target:%+.1fcm" % (pos_cm, target_cm), color=(255, 255, 0)
    )
    osd_image.draw_string_advanced(
        5, 58, 20, "err:%+.1fcm score:%d %s" % (err_cm, int(score * 100.0), status), color=(255, 255, 0)
    )
    osd_image.draw_string_advanced(
        5, 82, 20, "px:%d,%d" % (center[0], center[1]), color=(180, 255, 180)
    )


def run_self_test():
    calibration = make_calibration((320, 180), (80, 180), (560, 180), 12.0)
    assert make_active_roi((640, 360)) == (0, 120, 640, 120)
    assert cm_to_x10(project_position_cm((320, 180), calibration)) == 0
    assert cm_to_x10(project_position_cm((420, 180), calibration)) == 50
    assert cm_to_x10(project_position_cm((220, 180), calibration)) == -50
    assert pixel_from_cm(5.0, calibration) == (420, 180)
    line = send_position(None, True, 4.8, 5.0, 0.82, "BALL")
    assert line == "BALL,1,48,50,2,82,BALL\n"
    print("self-test ok: main_h_topic_position_uart_rtsp_ap.py")


def main():
    from libs.PipeLine import PipeLine
    from libs.YOLO import YOLOv8
    from libs.Utils import ScopedTiming
    import time

    pipeline = None
    detector = None
    uart = None
    ap = None
    rtsp_started = False
    rtsp_url = None
    net_status = "AP: waiting"
    last_client_status = 0
    calibration = make_calibration(ZERO_PX, MINUS_5CM_PX, PLUS_5CM_PX, HALF_RANGE_CM)

    try:
        log("[h_position_rtsp] boot:", SCRIPT_VERSION)
        uart = init_uart()
        model_path = select_model_path()
        log("[h_position_rtsp] model:", model_path)
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_mode=DISPLAY_MODE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create(
            sensor_id=SENSOR_ID,
            hmirror=H_MIRROR,
            vflip=V_FLIP,
            to_ide=False,
        )
        display_size = pipeline.get_display_size()
        active_roi = make_active_roi(display_size)
        log("[h_position_rtsp] roi: x=%d y=%d w=%d h=%d" % active_roi)

        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=model_path,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES,
            debug_mode=0,
        )
        detector.config_preprocess()

        try:
            ap, ip = start_ap()
            if ip is None:
                net_status = "AP: no ip"
            else:
                rtsp_url = start_rtsp(ip)
                rtsp_started = True
                net_status = "AP: %s" % ip
        except BaseException as net_error:
            net_status = "AP/RTSP failed"
            log("[h_position_rtsp] ap rtsp error:", net_error)
            sys.print_exception(net_error)

        frame_index = 0
        last_center = None
        last_score = 0.0
        last_seen_ms = time.ticks_ms()

        while True:
            with ScopedTiming("total", 1):
                now_ms = time.ticks_ms()
                if ap is not None and time.ticks_diff(now_ms, last_client_status) >= CLIENT_STATUS_EVERY_MS:
                    last_client_status = now_ms
                    clients = get_client_count(ap)
                    if rtsp_url is None:
                        net_status = "AP clients:%d" % clients
                    else:
                        net_status = "AP clients:%d %s" % (clients, rtsp_url)

                frame = pipeline.get_frame()
                result = detector.run(frame)
                count, center, score = get_best_detection(result, active_roi)

                status = "LOST"
                if center is not None:
                    last_center = center
                    last_score = score
                    last_seen_ms = now_ms
                    status = "BALL"
                elif last_center is not None and time.ticks_diff(now_ms, last_seen_ms) <= TARGET_HOLD_MS:
                    center = last_center
                    score = last_score
                    status = "HOLD"

                found = center is not None
                pos_cm = project_position_cm(center, calibration) if found else 0.0

                detector.draw_result(result, pipeline.osd_img)
                draw_overlay(pipeline.osd_img, center, pos_cm, TARGET_CM, score, status, calibration, active_roi)
                pipeline.osd_img.draw_string_advanced(5, 106, 16, net_status, color=(255, 255, 255))
                pipeline.show_image()

                line = send_position(uart, found, pos_cm, TARGET_CM, score, status)
                if frame_index % PRINT_EVERY_N_FRAMES == 0:
                    print(line.strip())

                frame_index += 1
                if frame_index % GC_EVERY_N_FRAMES == 0:
                    gc.collect()
    except KeyboardInterrupt:
        log("[h_position_rtsp] stopped by user")
    except BaseException as error:
        log("[h_position_rtsp] error:", error)
        sys.print_exception(error)
        raise
    finally:
        send_position(uart, False, 0.0, TARGET_CM, 0.0, "LOST")
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as cleanup_error:
                print("[h_position_rtsp] detector cleanup error:", cleanup_error)
        if rtsp_started:
            try:
                WBCRtsp.stop()
            except BaseException as cleanup_error:
                print("[h_position_rtsp] rtsp cleanup error:", cleanup_error)
        if pipeline is not None and getattr(pipeline, "sensor", None) is not None:
            try:
                pipeline.destroy()
            except BaseException as cleanup_error:
                print("[h_position_rtsp] pipeline cleanup error:", cleanup_error)
        if uart is not None and hasattr(uart, "deinit"):
            uart.deinit()
        ap = None
        gc.collect()


if __name__ == "__main__":
    argv = getattr(sys, "argv", [])
    if len(argv) > 1 and argv[1] == "--self-test":
        run_self_test()
    else:
        main()
