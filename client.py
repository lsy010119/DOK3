import threading
import numpy as np
import sys
import socket
import time
from queue import Queue
import datetime
import rospy

from std_msgs.msg import String

class Client:
    
    def __init__(self,ip,port,datahub):
        self.TCP_SERVER_IP = ip
        self.TCP_SERVER_PORT = port
        self.datahub = datahub

        self.target_wp_pub = rospy.Publisher("/target_wp",String,queue_size=1)

    def start(self):
        self.connectServer()

    def connectServer(self):
        try:
            self.sock = socket.socket()
            self.sock.connect((self.TCP_SERVER_IP, self.TCP_SERVER_PORT))
            print(u'connected with server')
            self.connectCount = 0
            self.update_datas()

        except Exception as e:
            self.connectCount += 1

            if self.connectCount == 10:
                print(u'Connect fail %d times. Exit program'%(self.connectCount))
                sys.exit()

            print(u'Connection %d times fail. Wait 1 sec '%(self.connectCount))
            self.connectServer()
            time.time(1)


    def update_datas(self):


        while True:

            mode = str(self.datahub.flight_mode)
            isAuto = self.parsingisAuto(mode)
            wayPoint = 'w,'+str(self.datahub.waypoints)
            state = 't'+str(self.datahub.state)
            speed = 's'+str(round(np.linalg.norm(self.datahub.posvel_ned[3:]),2))
            imu = 'i,'+str(round(self.datahub.attitude_eular[0],2))+','+str(round(self.datahub.attitude_eular[1],2))+','+str(round(self.datahub.attitude_eular[2],2))
            gps = self.parsingGPS()
            heading = 'h'+str(self.datahub.heading_wp)
            jps     = 'j'+str(self.datahub.jps_map[:-2].tolist())
            batt    = 'b'+str(self.datahub.battery)    

            try:
                traj    = 'o'+str( ( ( ((self.datahub.traj[:2,:] - np.reshape(self.datahub.traj[:2,0],(2,1)))//0.0666666) + \
                            (300*np.ones((2,1))) ).flatten() ).tolist() )
                self.send(traj)
            except:
                traj    = 'o'+str( np.array([]) )
                self.send(traj)

            # Upload to Server
            self.send(isAuto)
            self.send(wayPoint)
            self.send(state)
            self.send(speed)
            self.send(imu)
            self.send(gps)
            self.send(heading)
            self.send(jps)
            self.send(batt)

            # ROStopic publishing
            target_wp = str(self.datahub.heading_wp)
            self.target_wp_pub.publish(target_wp)

            time.sleep(0.1)

    def send(self,stringData):
        length = str(len(stringData))
        self.sock.sendall(length.encode('utf-8').ljust(64))
        self.sock.send(stringData.encode('utf-8'))

    def parsingisAuto(self,mode):
        if mode == 'OFFBOARD' or mode == 'LAND':
            return 'a1'
        else:
            return 'a0'

    def parsingGPS(self):
        t = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-5]+'0'
        latitude = str(round(self.datahub.pos_global[0],6))
        longitude = str(round(self.datahub.pos_global[1],6))
        altitude = str(round(self.datahub.pos_global[3],2))
        return 'g,'+t+','+latitude+','+longitude+','+altitude