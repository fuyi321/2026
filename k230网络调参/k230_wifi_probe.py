import network
import time

SSID = "K230TEST"
PASSWORD = "12345678"
TIMEOUT_S = 30

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

print("scan start")
try:
    for ap in wlan.scan():
        print(ap)
except Exception as e:
    print("scan failed:", e)

print("connect:", SSID)
wlan.connect(SSID, PASSWORD)
start = time.time()
while not wlan.isconnected():
    if time.time() - start > TIMEOUT_S:
        print("connect timeout")
        try:
            print("ifconfig:", wlan.ifconfig())
        except Exception as e:
            print("ifconfig failed:", e)
        raise RuntimeError("wifi connect failed")
    time.sleep_ms(200)

print("connected:", wlan.ifconfig())
