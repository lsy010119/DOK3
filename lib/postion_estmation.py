 #python3 aruco_detector.py --image images/example_01.png --type DICT_5X5_100

# import the necessary packages
import imutils
import cv2
import numpy as np


class ArUcoPosEstimator:
    def __init__(self):
 
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }


    def run(self, image, cam_mtx, dist_coeff, type, resize_width):

        ids = {}
        centers = {}
        topR = {}
        topL = {}
        bottomR = {}
        bottomL = {}
        tvec = {}
        x = {}
        y = {}
        z = {}
        image = imutils.resize(image, width=resize_width)
    

        arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[type])

        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
            parameters=arucoParams)

        if len(corners) > 0:

            ids = ids.flatten()
            
            for (markerCorner, ID) in zip(corners, ids):

                centers[ID]= [-1, -1]

                corner = markerCorner.reshape((4, 2))

                (topLeft, topRight, bottomRight, bottomLeft) = corner

                rvec , tvec[ID], _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner,13, cam_mtx, dist_coeff)
                
                x[ID] = tvec[ID][0][0][0]
                
                y[ID] = tvec[ID][0][0][1]
                
                z[ID] = tvec[ID][0][0][2]

                topR[ID] = (int(topRight[0]), int(topRight[1]))

                bottomR[ID] = (int(bottomRight[0]), int(bottomRight[1]))

                bottomL[ID] = (int(bottomLeft[0]), int(bottomLeft[1]))

                topL[ID] = (int(topLeft[0]), int(topLeft[1]))

                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)


                centers[ID] = (cX, cY)
                centers[ID]= [-1, -1]
                corner = markerCorner.reshape((4, 2))

                (topLeft, topRight, bottomRight, bottomLeft) = corner

        # if len(ids) > 0:
        return ids,x,y,z



# ids,centers,topR,bottomL,x,y,z= self.detecter.run(self.marker_id,\
#                                                   img,self.cam_mtx,\
#                                                   self.dist_coeff,\
#                                                   "DICT_5X5_1000",\
#                                                   resize_width)