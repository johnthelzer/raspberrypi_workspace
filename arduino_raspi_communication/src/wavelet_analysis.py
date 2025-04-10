#!/usr/bin/env python3
import rospy
import cv2
import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import pywt
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from mavros_msgs.msg import RCIn
from cv_bridge import CvBridge, CvBridgeError
port_name = '/dev/ttyACM0'
baudrate = 115200
char_sent = True
initialized = False
accel_z_array = []
stop_time = 1

class wavelet_analyzer:
    def __init__(self):
        rospy.init_node('wavelet_analyzer', anonymous=True)
        self.rate = rospy.Rate(20)
        self.wavelet_pub = rospy.Publisher('/wavelet_graph', Image, queue_size = 1)
        self.rc_sub = rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_callback)
        self.bridge = CvBridge()
        #self.cam = cv2.VideoCapture(0)
        rospy.spin()

    def publish_image(self):
        global char_sent
        global initialized
        global stop_time
        global accel_z_array
        if not char_sent:
            #open port
            port = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.25)
            rospy.sleep(3)
            print("port opened")
            #send start byte to port
            port.write(b'w')
            char_sent = True
            #start reading accelerations
            start_time_sec = rospy.get_time()
            while (rospy.get_time() - start_time_sec <= stop_time):
                if(port.inWaiting()>0):
                    incoming_data = port.readline(port.inWaiting()).decode('utf-8')
                    #parse data- remove newline symbol (could be at beginning or end of input
                    for i in range(len(incoming_data)):
                        if incoming_data[i] == '\n':
                            incoming_data = incoming_data.replace('\n', '')
                    #check for truncation of string (ending at decimal point or just including decimal point, nothing else, or ending at decimal point)
                    truncated = False
                    if '.' not in incoming_data:
                        truncated = True
                    elif incoming_data[0] == '.':
                        truncated = True
                    elif incoming_data[len(incoming_data)-1] == '.':
                        truncated = True
                    #if len(incoming_data) <= 2:
                        #truncated = True
                    #only record data if not truncated
                    if truncated == False:
                        incoming_data = float(incoming_data)
                        accel_z_array = np.append(accel_z_array, incoming_data)
                    #print(incoming_data)
            port.close()
            #form wavelet
            wavelet = "cmorl1.5-1.0"
            widths = np.geomspace(1, 1024, num=100)
            sampling_period = 1/500
            cwtmatr, freqs = pywt.cwt(accel_z_array, widths, wavelet, sampling_period=sampling_period)
            cwtmatr = np.abs(cwtmatr[:-1, :-1])
            fig, axs = plt.subplots(1,1)
            t = np.arange(len(accel_z_array))/len(accel_z_array)
            pcm = axs.pcolormesh(t, freqs, cwtmatr)
            axs.set_yscale("log")
            axs.set_xlabel("Time (s)")
            axs.set_ylabel("Frequency (Hz)")
            axs.set_title("Continuous Wavelet Transform")
            fig.colorbar(pcm, ax=axs)
            plt.tight_layout()
            plt.plot()
            #convert to cv2 image
            fig.canvas.draw()
            plt.close()
            img_plot = np.array(fig.canvas.renderer.buffer_rgba())
            cv2_image = cv2.cvtColor(img_plot, cv2.COLOR_RGBA2BGR)
                #compress image
                #encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                #result, cv2_compressed_image_encoded = cv2.imencode('.jpg', cv2_image, encode_param)
                #cv2_compressed_image = cv2.imdecode(cv2_compressed_image_encoded, cv2.IMREAD_COLOR)
                #print(cv2_compressed_image.shape)
                #cv2.imshow('Image', cv2.cvtColor(img_plot, cv2.COLOR_RGBA2BGR))
                #cv2.waitKey(0)
                #ret, cv2_image = self.cam.read()
                #cv2.imshow("Image window", cv2_compressed_image)
                #cv2.waitKey(3)
            try:
                #img_msg = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
                self.wavelet_pub.publish(self.bridge.cv2_to_imgmsg(cv2_image, "bgr8"))
                #self.wavelet_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv2_compressed_image, "bgr8"))
                print('image published')
                #print(cv2_compressed_image.shape)
            except CvBridgeError as error:
                print(error)

    def rc_callback(self, data):
        global char_sent
        global initialized
        global stop_time
        global accel_z_array
        rc_value = data.channels[7] #channel 8
        print(rc_value)
        if ((rc_value < 1700) and initialized == False): #switch "high" for the first time
            char_sent = False 
            initialized = True
            self.publish_image() #publish (only actually publishes if char_sent is false)
        elif (rc_value > 1700): #switch is "low"
            char_sent = True #prevent publishing
            initialized = False
        #else (switch high but already initialized), do nothing
        
        
if __name__ == '__main__':
    wa_ = wavelet_analyzer()
