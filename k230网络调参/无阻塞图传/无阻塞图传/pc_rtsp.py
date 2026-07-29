"""
pc_rtsp.py — PC RTSP 播放器 + UDP 发现
"""
import os

os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|stimeout;3000000",
)

import cv2, socket, time

UDP_PORT = 19124


def discover():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', UDP_PORT))
    s.settimeout(0)
    print("[发现] 等待 K230...")
    for _ in range(60):
        try:
            data, addr = s.recvfrom(256)
            port = 8554
            for p in data.decode().split(':'):
                if p.isdigit(): port = int(p)
            print("[发现] {}:{}".format(addr[0], port))
            s.close()
            return addr[0], port
        except: time.sleep(0.5)
    s.close()
    return None, None


def main():
    while True:
        ip, port = discover()
        if ip: break
        time.sleep(2)

    url = "rtsp://{}:{}/k230".format(ip, port)
    cv2.namedWindow("K230", cv2.WINDOW_NORMAL)

    while True:
        print("[RTSP]", url)
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            time.sleep(3)
            continue
        while True:
            ok, frame = cap.read()
            if not ok: break
            cv2.imshow("K230", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                cap.release(); cv2.destroyAllWindows(); return
        cap.release()
        time.sleep(1)


if __name__ == "__main__":
    main()
