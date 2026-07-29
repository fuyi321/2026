"""
wt5_mjpeg.py - K230 MJPEG browser stream
"""
import gc
import network
import os
import socket
import sys
import time

from media.mjpeg import MJPEGEncoder
from media.sensor import Sensor


WIFI_SSID = 'Ciallo～(∠・ω< )⌒☆'
WIFI_PASS = '0d000721'

SERVER_PORT = 8080
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_ALIGNMENT = 12
JPEG_QUALITY = 45
STREAM_FPS = 12
REQUEST_TIMEOUT_MS = 2000
SEND_STALL_TIMEOUT_MS = 5000
SEND_CHUNK_BYTES = 8192

INDEX_HTML = b"""<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K230 MJPEG</title>
<style>
html,body{margin:0;width:100%;height:100%;background:#111}
body{display:grid;place-items:center}
img{max-width:100vw;max-height:100vh;object-fit:contain}
</style>
</head>
<body><img src="/stream"></body>
</html>
"""


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if wlan.isconnected():
        wlan.disconnect()
        time.sleep_ms(300)
    print("Connecting WiFi:", WIFI_SSID)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    start = time.time()
    while not wlan.isconnected():
        if time.time() - start > 20:
            raise RuntimeError("WiFi connect timeout")
        os.exitpoint()
        time.sleep_ms(100)
    print("Open http://%s:%d/" % (wlan.ifconfig()[0], SERVER_PORT))
    return wlan


def send_all(client, data):
    view = memoryview(data)
    offset = 0
    deadline = time.ticks_add(time.ticks_ms(), SEND_STALL_TIMEOUT_MS)
    while offset < len(view):
        try:
            end = min(offset + SEND_CHUNK_BYTES, len(view))
            sent = client.send(view[offset:end])
            if sent:
                offset += sent
                deadline = time.ticks_add(time.ticks_ms(), SEND_STALL_TIMEOUT_MS)
                continue
        except OSError as err:
            if err.errno not in (11, 110):
                raise
        if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            raise OSError(110)
        os.exitpoint()
        time.sleep_ms(1)


def read_path(client):
    data = bytearray()
    deadline = time.ticks_add(time.ticks_ms(), REQUEST_TIMEOUT_MS)
    client.setblocking(False)
    while len(data) < 1024:
        try:
            chunk = client.recv(128)
        except OSError as err:
            if err.errno not in (11, 110):
                raise
            chunk = None
        if chunk:
            data.extend(chunk)
            if data.find(b"\r\n\r\n") >= 0:
                break
        elif time.ticks_diff(deadline, time.ticks_ms()) <= 0:
            break
        else:
            os.exitpoint()
            time.sleep_ms(10)
    parts = bytes(data).split(b"\r\n", 1)[0].split()
    if len(parts) < 2:
        return "/"
    return parts[1].decode().split("?", 1)[0]


def capture_jpeg(sensor, encoder):
    frame = sensor.snapshot(dump_frame=True)
    jpeg = encoder.encode(frame, timeout_ms=1000)
    del frame
    return jpeg


def stream_mjpeg(client, sensor, encoder):
    send_all(client, b"HTTP/1.1 200 OK\r\n")
    send_all(client, b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n")
    send_all(client, b"Cache-Control: no-store\r\n\r\n")
    interval = 1000 // STREAM_FPS
    count = 0
    while True:
        start = time.ticks_ms()
        jpeg = capture_jpeg(sensor, encoder)
        header = (
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n\r\n"
        ) % len(jpeg)
        send_all(client, header.encode())
        send_all(client, jpeg)
        send_all(client, b"\r\n")
        del jpeg
        count += 1
        if count % 20 == 0:
            gc.collect()
        delay = interval - time.ticks_diff(time.ticks_ms(), start)
        if delay > 0:
            time.sleep_ms(delay)
        os.exitpoint()


def serve_client(client, sensor, encoder):
    path = read_path(client)
    if path == "/stream":
        stream_mjpeg(client, sensor, encoder)
    elif path == "/snapshot.jpg":
        jpeg = capture_jpeg(sensor, encoder)
        send_all(client, ("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg)).encode())
        send_all(client, jpeg)
        del jpeg
    else:
        send_all(client, ("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\n\r\n" % len(INDEX_HTML)).encode())
        send_all(client, INDEX_HTML)


def main():
    wlan = None
    sensor = None
    encoder = None
    server = None
    client = None
    try:
        wlan = connect_wifi()
        sensor = Sensor(id=2, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=STREAM_FPS)
        sensor.reset()
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT, alignment=FRAME_ALIGNMENT)
        sensor.set_pixformat(Sensor.YUV420SP)
        sensor.run()
        encoder = MJPEGEncoder(quality=JPEG_QUALITY)
        for _ in range(5):
            sensor.snapshot()
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(socket.getaddrinfo("0.0.0.0", SERVER_PORT)[0][-1])
        server.listen(2)
        server.setblocking(False)
        while True:
            try:
                client, addr = server.accept()
            except OSError as err:
                if err.errno != 11:
                    raise
                os.exitpoint()
                time.sleep_ms(20)
                continue
            print("Client:", addr)
            try:
                serve_client(client, sensor, encoder)
            except OSError:
                pass
            finally:
                client.close()
                client = None
                gc.collect()
    except BaseException as err:
        sys.print_exception(err)
    finally:
        if client:
            client.close()
        if server:
            server.close()
        if encoder:
            encoder.close()
        if sensor:
            sensor.stop()
        wlan = None


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()
