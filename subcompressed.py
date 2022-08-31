#!/usr/bin/env python3
import sys,time
import numpy as np
import cv2
import rospy
import roslib

from scipy.ndimage   import filters
from sensor_msgs.msg import CompressedImage
from std_msgs.msg    import Float32MultiArray

class subcomim:
    def __init__(self):
        #subscriber
        rospy.init_node('subscribecompressedimage', anonymous= False)
        self.subscriber = rospy.Subscriber('/bottom_cam/bottom_image_raw/compressed', CompressedImage, self.callback, queue_size=1)
        
        #publisher
        self.publisher = rospy.Publisher('marker_detected', Float32MultiArray, queue_size = 1)


    def callback(self, ros_data):
        #binarary image -> decode
        self.np_arr = np.frombuffer(ros_data.data, np.uint8)
        self.image_np = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
        img_gray = cv2.cvtColor(self.image_np, cv2.COLOR_RGB2GRAY)
        ################# thresh -> binary image (사진 이진화)
        ret, thresh = cv2.threshold(img_gray,130,255, cv2.THRESH_BINARY)
        # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
        #contour 시각화 시작
        # edges = cv2.Canny(img_gray, 10, 100)
        ################ binary image
        edges = cv2.Canny(thresh, 10, 100)
        #morphologyEX화, kernel은 클러스터링 느낌의 크기
        kernel = np.ones((1,1),np.uint16)
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN,kernel)
        contours, hierarchy = cv2.findContours(edges.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        #approxpolydp 사용 -> 근사화=
        for pts in contours:
            if cv2.contourArea(pts) < 1000:  #노이즈 제거, 넓이가 너무 작으면 무시
                continue
            #근사화 -> cv2.approxPolyDP(사진, 근사화할 비율, True: 폐곡선을 의미)
            approx = cv2.approxPolyDP(pts, cv2.arcLength(pts,True)*0.02,True)
            #근사화 결과 점 개수
            vtc = len(approx)
            # print(f'꼭짓점 수: {vtc}')
            #12각형 -> 십자 마크
            if vtc == 12:
                self.setLabel(self.image_np,pts,'+MARKER')
                # thresh
                # setLabel(thresh,pts,'+MARK0ER')
                print('+marker detected')
        # cv2.imshow('cv_img', self.image_np)
        # cv2.waitKey(1)
        # if KeyboardInterrupt:
        #     cv2.destroyAllWindows()


    def setLabel(self,img, pts, label):
        # 사각형 좌표 받아오기
        (x, y, w, h) = cv2.boundingRect(pts)
        #cv2.aruco.estimatePoseSingleMarkers 함수의 인자때문
        x = float(x)
        y = float(y)
        w = float(w)
        h = float(h)
        pt1 = np.array([x, y])
        pt2 = np.array([x + w, y + h])
        pt3 = np.array([x + w, y])
        pt4 = np.array([x,y+w])
        #marker corner
        markerCorner = np.array([[
            pt4,
            pt2,
            pt3,
            pt1
        ]])
        # 카메라 매트릭스 및 dist coefficient
        cam_mtx = np.array([[503.55091216,   0,         305.63078085],
                            [  0,         503.09411772, 243.44984751],
                            [  0,           0,           1          ]])
        dist_coeff = np.array([[ 1.23526628e-01, -5.69162763e-01,  1.44318047e-04, -1.04153911e-03, 9.04960749e-01]])
        # marker corner, real size (cm), cam_mtx, dist coeff
        rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner,0.531, cam_mtx, dist_coeff)
        # rcev : rotation vector
        # tvec : translation vector
        #cv2.rectangle 함수의 인자때문
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        pt1 = np.array([x, y])
        pt2 = np.array([x + w, y + h])
        pt3 = np.array([x + w, y])
        pt4 = np.array([x,y+w])

        cX = int((pt1[0]+pt2[0])/2.0)
        cY = int((pt1[1]+pt2[1])/2.0)



        #publish center_marker
        center_marker = Float32MultiArray()
        center_marker.data = [cX,cY]
        self.publisher.publish(center_marker)

        #for center of marker
        return [cX,cY]

        

        
        #rectangle bounding box
        # cv2.rectangle(img, pt1, pt2, (0, 0, 255), 5)
        # cv2.circle(img, (cX, cY), 10, (0, 0,255), -1)
        # print(rvec)
        # cv2.putText(img,'x:%.4f, y:%.4f, z:%.4f'%(tvec[0,0,0],tvec[0,0,1],tvec[0,0,2]),pt1, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))


if __name__ == '__main__':
    image = subcomim()
    time.sleep(1.0)
    while True:
        img_gray = cv2.cvtColor(image.image_np, cv2.COLOR_RGB2GRAY)
        ################# thresh -> binary image (사진 이진화)
        ret, thresh = cv2.threshold(img_gray,20,255, cv2.THRESH_BINARY)
        # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
        #contour 시각화 시작
        # edges = cv2.Canny(img_gray, 10, 100)
        ################ binary image
        edges = cv2.Canny(thresh, 10, 100)
        #morphologyEX화, kernel은 클러스터링 느낌의 크기
        kernel = np.ones((1,1),np.uint16)
        edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN,kernel)
        contours, hierarchy = cv2.findContours(edges.copy(), mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        #approxpolydp 사용 -> 근사화=
        for pts in contours:
            if cv2.contourArea(pts) < 1000:  #노이즈 제거, 넓이가 너무 작으면 무시
                continue
            #근사화 -> cv2.approxPolyDP(사진, 근사화할 비율, True: 폐곡선을 의미)
            approx = cv2.approxPolyDP(pts, cv2.arcLength(pts,True)*0.02,True)
            #근사화 결과 점 개수
            vtc = len(approx)
            # print(f'꼭짓점 수: {vtc}')
            #12각형 -> 십자 마크
            if vtc == 12:
                [cX,cY] = image.setLabel(image.image_np,pts,'+MARKER')
                # thresh
                # setLabel(thresh,pts,'+MARK0ER')
                print(f'+marker detected {cX},{cY}')
                

        cv2.imshow('cv_img', image.image_np)
        cv2.waitKey(1)