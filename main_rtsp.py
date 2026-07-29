"""K230 CanMV steel-ball detection with processed-frame RTSP streaming.

Copy this file and the kmodel to /sdcard/steel_ball/. Run it from CanMV IDE,
or copy it to /sdcard/main.py for auto-start after board-side validation.
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from libs.Utils import ScopedTiming
from libs.WBCRtsp import WBCRtsp
import gc
import network
import sys
import time


MODEL_PATH = "/sdcard/steel_ball/steel_ball_yolov8n_320_round3.kmodel"
#MODEL_PATH = "/sdcard/steel_ball/steel_ball_yolov8n_320_r2.kmodel"
#MODEL_PATH = "/sdcard/steel_ball/steel_ball_yolov8n_320.kmodel"
LABELS = ["steel_ball"]
MODEL_INPUT_SIZE = [320, 320]

# Verified on LushanPi Lite / K230D with CanMV v1.8.
SENSOR_ID = 2
DISPLAY_MODE = "lcd"
DISPLAY_SIZE = None
RGB888P_SIZE = [640, 360]
H_MIRROR = True
V_FLIP = True

CONFIDENCE_THRESHOLD = 0.40
NMS_THRESHOLD = 0.45
MAX_BOXES = 50
PRINT_EVERY_N_FRAMES = 10

USE_WLAN = True
WIFI_SSID = "K230TEST"
WIFI_PASSWORD = "CHANGE_ME"
WIFI_TIMEOUT_S = 20
LAN_STATIC_CONFIG = None

RTSP_PORT = 8554
RTSP_SESSION = "test"
WBC_WIDTH = 800
WBC_HEIGHT = 480


def ifconfig_ip(netif):
    cfg = netif.ifconfig()
    if cfg:
        return cfg[0]
    return None


def connect_network():
    start_time = time.time()
    print("[steel_ball] network init")

    if USE_WLAN:
        netif = network.WLAN(network.STA_IF)
        netif.active(True)
        if not netif.isconnected():
            print("connecting WiFi:", WIFI_SSID)
            netif.connect(WIFI_SSID, WIFI_PASSWORD)
            while not netif.isconnected():
                if time.time() - start_time > WIFI_TIMEOUT_S:
                    raise RuntimeError("WLAN connect timeout")
                time.sleep_ms(100)
        ip = netif.ifconfig()[0]
    else:
        netif = network.LAN()
        if LAN_STATIC_CONFIG:
            netif.ifconfig(LAN_STATIC_CONFIG)
        netif.active(True)
        print("activating LAN...")
        ip = ifconfig_ip(netif)
        while ip is None or ip == "0.0.0.0":
            if time.time() - start_time > WIFI_TIMEOUT_S:
                raise RuntimeError("LAN activation timeout")
            time.sleep_ms(100)
            ip = ifconfig_ip(netif)

    print("[steel_ball] network:", netif.ifconfig())
    print("[steel_ball] K230 IP:", ip)
    return netif, ip


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


def main():
    netif = None
    pipeline = None
    detector = None
    rtsp_started = False
    try:
        print("[steel_ball] boot")
        netif, ip = connect_network()
        print("[steel_ball] rtsp url: rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SESSION))

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

#        WBCRtsp.configure(wbc_width=WBC_WIDTH, wbc_height=WBC_HEIGHT)
        WBCRtsp.configure(wbc_width=480, wbc_height=480)
        WBCRtsp.start()
        rtsp_started = True
        print("[steel_ball] rtsp started")

        display_size = pipeline.get_display_size()
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

        frame_index = 0
        while True:
            with ScopedTiming("total", 0):
                frame = pipeline.get_frame()
                result = detector.run(frame)
                detector.draw_result(result, pipeline.osd_img)
                add_status_overlay(result, pipeline.osd_img, frame_index)
                pipeline.show_image()
                frame_index += 1
                gc.collect()
    except KeyboardInterrupt:
        print("[steel_ball] stopped by user")
    except BaseException as error:
        print("[steel_ball] error:", error)
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
        netif = None
        gc.collect()


main()
