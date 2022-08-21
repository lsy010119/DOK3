import socket
import cv2
import time
import numpy as np
import sys
import base64
import threading
from queue import Queue

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import signal



class ClientSocket:
   
    def __init__(self, ip, port):

        self.TCP_SERVER_IP = ip
        self.TCP_SERVER_PORT = port
        self.connectCount = 0

        # Set queues
        self.detected_frame = Queue()

        # Count image
        self.img_num = 0

        self.x = None

        # Connect to server (Main thread)
        self.connectServer()

        # Image publisher thread (Sub thread 1)
        threading.Thread(target = self.img_pub).start()

        # Initial time
        self.t1 = time.time()

        # ros thread (Sub thread 2)
        rospy.init_node('imgSender')
        rospy.Subscriber('/bottom_cam/bottom_image_raw/compressed',CompressedImage,self.img_callback,queue_size=1)
        rospy.spin()



    def connectServer(self):

        try:
            self.sock = socket.socket()
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            print(u'Client socket is connected with Server socket [ TCP_SERVER_IP: ' + self.TCP_SERVER_IP + ', TCP_SERVER_PORT: ' + str(self.TCP_SERVER_PORT) + ' ]')
            self.connectCount = 0

        except Exception as e:
            print(e)
            self.connectCount += 1
            if self.connectCount == 10:
                print(u'Connect fail %d times. exit program'%(self.connectCount))
                time.time(2)
                sys.exit()
            print(u'%d times try to connect with server'%(self.connectCount))
            self.connectServer()


    # Callback function for ROS
    def img_callback(self,msg):

        # buffer to nd.array
        frame = np.frombuffer(msg.data,np.uint8)
        # decode to cv2 image
        frame = cv2.imdecode(frame,cv2.COLOR_RGB2GRAY)
        # resize
        frame = cv2.resize(frame,(640,360))
        # detect marker
        self.detect_marker(frame)


    def detect_marker(self,frame):
        
            t1 = time.time()
            #thresh = cv2.inRange(frame,0,1)

            #contour 시각화 시작
            edges = cv2.Canny(frame, 180,200)

            #morphologyEX화, kernel은 클러스터링 느낌의 크기
            kernel = np.ones((1,1),np.uint16)
            edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN,kernel)

            # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
            contours, hierarchy = cv2.findContours(edges.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
            

            n = 0
            detected = []
            detected_pts = []
            detected_hier = []

            for pts in contours:
                if cv2.contourArea(pts) < 1000:  #노이즈 제거, 넓이가 너무 작으면 무시
                    n+=1
                    continue

                #근사화 -> cv2.approxPolyDP(사진, 근사화할 비율, True: 폐곡선을 의미)
                approx = cv2.approxPolyDP(pts, cv2.arcLength(pts,True)*0.02,True)
                #근사화 결과 점 개수
                vtc = len(approx)
                # print(f'꼭짓점 수: {vtc}')

                #12각형 -> 십자 마크
                if vtc == 12:
                      detected.append(n)
                      detected_pts.append(pts)
                      detected_hier.append(hierarchy[0,n])
                      print(n, hierarchy[0,n])
                      self.setLabel2(frame,pts)

                n+=1

            if len(detected) == 2:
                if detected[0] + 1 == detected[1]:
                    print('marker detected')
                    self.setLabel(frame,detected_pts[0])

            elif len(detected) == 1:
                if detected_hier[0][2] == -1 and detected[0]-1 == detected_hier[0][3] :
                    print('detected one but it is marker')
                    self.setLabel(frame,detected_pts[0])

            elif len(detected) > 2:
                for i in range(len(detected)-1) :
                    if detected[i] + 1 == detected[i+1]:
                        print('detected more then two but among them are marker')
                        self.setLabel(frame,detected_pts[i])
                        break

                    elif detected_hier[i][2] == -1 and detected[i]-1 == detected_hier[i][3]:
                        print('detected more then two but among them are marker(one)')
                        self.setLabel(frame,detected_pts[i])
                        break

                    elif i == len(detected)-1 :
                        print('detected more then two but among them are marker(one)')
                        if detected_hier[i+1][2] == -1 and detected[i+1]-1 == detected_hier[i+1][3]:
                            self.setLabel(frame,detected_pts[i+1])

            print('=====================')

            if not self.detected_frame.empty():
                self.detected_frame.get()
            self.detected_frame.put(frame)


    # 십자가로 인식된 마커 --> 붉은색 박스
    def setLabel(self,img, pts, label):
        # 사각형 좌표 받아오기
        (x, y, w, h) = cv2.boundingRect(pts)

        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        cv2.rectangle(img, (x,y),(x+w,y+h), (0 , 0, 255), 5)


    # 걸러지는 이미지 --> 파란색 박스
    def setLabel2(self,img, pts):
        # 사각형 좌표 받아오기
        (x, y, w, h) = cv2.boundingRect(pts)

        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        cv2.rectangle(img, (x,y),(x+w,y+h), (255 , 0, 0), 5)


    def img_pub(self):
        while True:

            frame = self.detected_frame.get()
            encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),20]
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            img = np.array(imgencode)

            stringData = base64.b64encode(img)
            length = str(len(stringData))

            self.sock.sendall(length.encode('utf-8').ljust(64))
            self.sock.send(stringData)

            self.img_num +=1
            print(self.img_num)


def main():
    TCP_IP = '165.246.139.32'
    TCP_PORT = 9502
    client = ClientSocket(TCP_IP, TCP_PORT)


def handler(signum, frame):
    print('**Server Killed**')
    sys.exit()



if __name__ == "__main__":
    main()
    signal.signal(signal.SIGTSTP, handler)