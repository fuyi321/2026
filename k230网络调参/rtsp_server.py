# Description: This example demonstrates how to stream video and audio to the network using the RTSP server.
#
# Note: You will need an SD card to run this example.
#
# You can run the rtsp server to stream video and audio to the network

from media.vencoder import *
from media.sensor import *
from media.media import *
import time, os
import _thread
import network
import multimedia as mm

USE_WLAN = True
#WIFI_SSID = "K230TEST"
#WIFI_PASSWORD = "59@2Vy28"
WIFI_SSID = "K230TEST"
WIFI_PASSWORD = "12345678"
WIFI_TIMEOUT_S = 20
LAN_STATIC_CONFIG = None # 例: ("192.168.137.2", "255.255.255.0", "192.168.137.1", "8.8.8.8")
STREAM_WIDTH = 640
STREAM_HEIGHT = 360
STREAM_FPS = 30
USE_H265 = False

def ifconfig_ip(netif):
    cfg = netif.ifconfig()
    if cfg:
        return cfg[0]
    return None

def init_network():
    start_time = time.time()

    if USE_WLAN:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            print("connecting WiFi:", WIFI_SSID)
            while not wlan.isconnected():
                if time.time() - start_time > WIFI_TIMEOUT_S:
                    raise RuntimeError("WLAN connect timeout; check 2.4G SSID/password, or set USE_WLAN=False for LAN")
                time.sleep_ms(100)
        ip = wlan.ifconfig()[0]
    else:
        lan = network.LAN()
        if LAN_STATIC_CONFIG:
            lan.ifconfig(LAN_STATIC_CONFIG)
        lan.active(True)
        print("activating LAN...")
        ip = ifconfig_ip(lan)
        while ip is None or ip == "0.0.0.0":
            if time.time() - start_time > 20:
                raise RuntimeError("LAN activation timeout; set USE_WLAN=True for WiFi, or check Ethernet/DHCP")
            time.sleep_ms(100)
            ip = ifconfig_ip(lan)

    print("K230 IP:", ip)
    return ip

class RtspServer:
    def __init__(self,session_name="test",port=8554,video_type = mm.multi_media_type.media_h264,enable_audio=False):
        self.session_name = session_name # session name
        self.video_type = video_type  # 视频类型264/265
        self.enable_audio = enable_audio # 是否启用音频
        self.port = port   #rtsp 端口号
        self.rtspserver = mm.rtsp_server() # 实例化rtsp server
        self.start_stream = False #是否启动推流
        self.runthread_over = False #推流线程是否结束线程
        self.enc_chn_id = VENC_CHN_ID_0 # 编码通道号

    def start(self):
        try:
            # 清理上次异常退出后可能残留的媒体资源
            try:
                MediaManager.deinit()
                time.sleep_ms(300)
            except:
                pass

            # 初始化推流
            self._init_stream()
            MediaManager.init()
            self.rtspserver.rtspserver_init(self.port)
            # 创建session
            self.rtspserver.rtspserver_createsession(self.session_name,self.video_type,self.enable_audio)
            # 启动rtsp server
            self.rtspserver.rtspserver_start()
            self._start_stream()

            # 启动推流线程
            self.start_stream = True
            _thread.start_new_thread(self._do_rtsp_stream,())
        except BaseException:
            self.stop()
            raise


    def stop(self):
        if (self.start_stream == True):
            # 等待推流线程退出
            self.start_stream = False
            while not self.runthread_over:
                time.sleep_ms(100)
            self.runthread_over = False

        # 停止推流
        try:
            self._stop_stream()
        except:
            pass
        try:
            self.rtspserver.rtspserver_stop()
        except:
            pass
        #self.rtspserver.rtspserver_destroysession(self.session_name)
        try:
            self.rtspserver.rtspserver_deinit()
        except:
            pass
        try:
            MediaManager.deinit()
        except:
            pass

    def get_rtsp_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)

    def _init_stream(self):
        width = STREAM_WIDTH
        height = STREAM_HEIGHT
        width = ALIGN_UP(width, 16)
        # 初始化sensor
        self.sensor = Sensor(id=2, width=width, height=height, fps=STREAM_FPS)
        self.sensor.reset()
        self.sensor.set_framesize(width = width, height = height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)
        # 实例化video encoder
        self.encoder = Encoder()
        self.encoder.SetOutBufs(chn=self.enc_chn_id, buf_num=8, width=width, height=height)
        # 创建编码器
        if USE_H265:
            self.video_type = mm.multi_media_type.media_h265
            self.chnAttr = ChnAttrStr(self.encoder.PAYLOAD_TYPE_H265, self.encoder.H265_PROFILE_MAIN, width, height)
        else:
            self.video_type = mm.multi_media_type.media_h264
            self.chnAttr = ChnAttrStr(self.encoder.PAYLOAD_TYPE_H264, self.encoder.H264_PROFILE_MAIN, width, height)
        # 绑定camera和venc
        self.link = MediaManager.link(self.sensor.bind_info()['src'], (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.enc_chn_id))
        print("stream config: %dx%d@%dfps %s" % (width, height, STREAM_FPS, "H265" if USE_H265 else "H264"))

    def _start_stream(self):
        # 创建编码器
        self.encoder.Create(self.enc_chn_id, self.chnAttr)
        # 开始编码
        self.encoder.Start(self.enc_chn_id)
        # 启动camera
        self.sensor.run()

    def _stop_stream(self):
        # 停止camera
        self.sensor.stop()
        # 接绑定camera和venc
        del self.link
        # 停止编码
        self.encoder.Stop(self.enc_chn_id)
        self.encoder.Destroy(self.enc_chn_id)

    def _do_rtsp_stream(self):
        streamData = StreamData()
        while self.start_stream:
            os.exitpoint()
            got_stream = False
            try:
                # 获取一帧码流
                self.encoder.GetStream(self.enc_chn_id, streamData)
                got_stream = True
                # 推流
                for pack_idx in range(0, streamData.pack_cnt):
                    stream_data = bytes(uctypes.bytearray_at(streamData.data[pack_idx], streamData.data_size[pack_idx]))
                    self.rtspserver.rtspserver_sendvideodata(self.session_name,stream_data, streamData.data_size[pack_idx],1000)
                    #print("stream size: ", streamData.data_size[pack_idx], "stream type: ", streamData.stream_type[pack_idx])
                # 释放一帧码流
                self.encoder.ReleaseStream(self.enc_chn_id, streamData)
            except Exception as e:
                import sys
                sys.print_exception(e)
                if got_stream:
                    try:
                        self.encoder.ReleaseStream(self.enc_chn_id, streamData)
                    except:
                        pass
                time.sleep_ms(100)

        self.runthread_over = True

if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    # 创建rtsp server对象
    rtspserver = RtspServer()
    try:
        ip = init_network()
        # 启动rtsp server
        rtspserver.start()
        # 打印rtsp url
        url = rtspserver.get_rtsp_url()
        if url.startswith("rtsp://0.0.0.0"):
            url = "rtsp://%s:%d/%s" % (ip, rtspserver.port, rtspserver.session_name)
        print("rtsp server start:",url)
        while True:
            time.sleep(1)
    finally:
        # 停止rtsp server
        rtspserver.stop()
        print("done")
