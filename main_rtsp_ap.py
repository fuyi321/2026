"""K230 creates a Wi-Fi AP and streams the processed YOLO view by RTSP."""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming
import gc
import network
import sys
import time

try:
    from libs.WBCRtsp import WBCRtsp
    WBCRTSP_IMPORT_ERROR = None
except BaseException as import_error:
    WBCRtsp = None
    WBCRTSP_IMPORT_ERROR = import_error


MODEL_PATH = "/sdcard/steel_ball/steel_ball_yolov8n_320_round3.kmodel"
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
MAX_BOXES = 10
PRINT_EVERY_N_FRAMES = 30
GC_EVERY_N_FRAMES = 20
ROI_DIVISOR = 3
ROI_COLOR = (0, 160, 255)

AP_SSID = "K230_STEEL_BALL"
AP_KEY = "12345678"
AP_START_TIMEOUT_MS = 5000
CLIENT_STATUS_EVERY_MS = 2000

RTSP_PORT = 8554
RTSP_SESSION = "test"
WBC_WIDTH = 640
WBC_HEIGHT = 360
VLC_NETWORK_CACHING_MS = 500
BOOT_LOG_PATH = "/sdcard/steel_ball/main_rtsp_ap_boot.log"
SCRIPT_VERSION = "main_rtsp_ap_offline_first"


def log(*parts):
    msg = " ".join([str(part) for part in parts])
    print(msg)
    try:
        with open(BOOT_LOG_PATH, "a") as log_file:
            log_file.write(msg + "\n")
    except:
        pass


def start_ap():
    log("[steel_ball] ap init:", AP_SSID)
    try:
        sta = network.WLAN(network.STA_IF)
        sta.active(False)
    except:
        pass

    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(ssid=AP_SSID, key=AP_KEY)

    start = time.ticks_ms()
    ip = ap.ifconfig()[0]
    while (ip is None or ip == "0.0.0.0") and time.ticks_diff(time.ticks_ms(), start) < AP_START_TIMEOUT_MS:
        time.sleep_ms(100)
        ip = ap.ifconfig()[0]

    log("[steel_ball] ap active:", ap.active())
    log("[steel_ball] ap config:", ap.ifconfig())
    if ip is None or ip == "0.0.0.0":
        return ap, None
    return ap, ip


def get_client_count(ap):
    try:
        return len(ap.status("stations"))
    except:
        return -1


def start_rtsp(ip):
    if WBCRtsp is None:
        raise RuntimeError("libs.WBCRtsp import failed: %s" % WBCRTSP_IMPORT_ERROR)

    rtsp_url = "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION)
    log("[steel_ball] ap ssid:", AP_SSID)
    log("[steel_ball] ap key:", AP_KEY)
    log("[steel_ball] rtsp url:", rtsp_url)
    log(
        "[steel_ball] vlc tcp: vlc --rtsp-tcp --network-caching=%d %s"
        % (VLC_NETWORK_CACHING_MS, rtsp_url)
    )
    log("[steel_ball] wbc: %dx%d" % (WBC_WIDTH, WBC_HEIGHT))
    WBCRtsp.configure(wbc_width=WBC_WIDTH, wbc_height=WBC_HEIGHT)
    WBCRtsp.start()
    log("[steel_ball] rtsp started")
    return rtsp_url


def add_status_overlay(result, osd_image, frame_index):
    count = 0
    best_score = -1.0
    best_center = None

    if result and len(result[0]) > 0:
        count = len(result[0])
        for index in range(count):
            x, y, width, height = result[0][index]
            score = float(result[2][index])
            if score > best_score:
                best_score = score
                best_center = (
                    int(round(x + width / 2)),
                    int(round(y + height / 2)),
                )

    osd_image.draw_string_advanced(
        5, 5, 24, "steel_ball: %d" % count, color=(0, 255, 0)
    )

    if best_center is not None:
        center_x, center_y = best_center
        osd_image.draw_cross(
            center_x,
            center_y,
            color=(255, 255, 0),
            size=12,
            thickness=3,
        )
        osd_image.draw_string_advanced(
            5,
            34,
            20,
            "best center: %d,%d" % (center_x, center_y),
            color=(255, 255, 0),
        )

    if frame_index % PRINT_EVERY_N_FRAMES == 0:
        if best_center is None:
            print("[steel_ball] count=0")
        else:
            print(
                "[steel_ball] count=%d best_center=(%d,%d) score=%.3f"
                % (count, best_center[0], best_center[1], best_score)
            )


def get_center_roi(display_size):
    width, height = display_size
    roi_h = height // ROI_DIVISOR
    return 0, (height - roi_h) // 2, width, roi_h


def center_in_roi(box, roi):
    x, y, width, height = box
    roi_x, roi_y, roi_w, roi_h = roi
    center_x = int(round(x + width / 2))
    center_y = int(round(y + height / 2))
    return (
        roi_x <= center_x < roi_x + roi_w
        and roi_y <= center_y < roi_y + roi_h
    )


def filter_result_by_roi(result, roi):
    if not result or len(result[0]) == 0:
        return result

    filtered = [[] for _ in range(len(result))]
    for index in range(len(result[0])):
        if center_in_roi(result[0][index], roi):
            for part_index in range(len(result)):
                filtered[part_index].append(result[part_index][index])
    return filtered


def draw_roi(osd_image, roi):
    x, y, width, height = roi
    osd_image.draw_rectangle(x, y, width, height, color=ROI_COLOR, thickness=2)


def main():
    ap = None
    pipeline = None
    detector = None
    rtsp_started = False
    rtsp_url = None
    net_status = "AP: waiting"
    last_client_status = 0

    try:
        log("[steel_ball] boot:", SCRIPT_VERSION)
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
        roi = get_center_roi(display_size)
        log("[steel_ball] roi: x=%d y=%d w=%d h=%d" % roi)
        detector = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=MODEL_PATH,
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
            log("[steel_ball] ap rtsp error:", net_error)
            sys.print_exception(net_error)

        frame_index = 0
        while True:
            with ScopedTiming("total", 0):
                now = time.ticks_ms()
                if ap is not None and time.ticks_diff(now, last_client_status) >= CLIENT_STATUS_EVERY_MS:
                    last_client_status = now
                    clients = get_client_count(ap)
                    if rtsp_url is None:
                        net_status = "AP clients:%d" % clients
                    else:
                        net_status = "AP clients:%d %s" % (clients, rtsp_url)

                frame = pipeline.get_frame()
                result = detector.run(frame)
                result = filter_result_by_roi(result, roi)
                detector.draw_result(result, pipeline.osd_img)
                draw_roi(pipeline.osd_img, roi)
                add_status_overlay(result, pipeline.osd_img, frame_index)
                pipeline.osd_img.draw_string_advanced(
                    5, 58, 16, net_status, color=(255, 255, 255)
                )
                pipeline.show_image()
                frame_index += 1
                if frame_index % GC_EVERY_N_FRAMES == 0:
                    gc.collect()
    except KeyboardInterrupt:
        log("[steel_ball] stopped by user")
    except BaseException as error:
        log("[steel_ball] error:", error)
        sys.print_exception(error)
        raise
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as cleanup_error:
                print("[steel_ball] detector cleanup error:", cleanup_error)
        if rtsp_started:
            try:
                WBCRtsp.stop()
            except BaseException as cleanup_error:
                print("[steel_ball] rtsp cleanup error:", cleanup_error)
        if pipeline is not None and getattr(pipeline, "sensor", None) is not None:
            try:
                pipeline.destroy()
            except BaseException as cleanup_error:
                print("[steel_ball] pipeline cleanup error:", cleanup_error)
        ap = None
        gc.collect()


main()
