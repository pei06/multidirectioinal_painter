#!/usr/bin/env python
import cv_bridge
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image

import numpy as np
import cv2
import math
from matplotlib import pyplot as plt
from cv_bridge import CvBridge
from std_msgs.msg import String
import glob
# from scipy.spatial.transform import Rotation as R

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
    
def grow(img,k,j,i,stack):
    (m,n) = np.shape(img)

    #step 1
    img[k,j] = i
    stack.append((k,j))
    stack.append((0,0))

    #step 2
    while True:
        if (j<n) and (img[k,j+1]) == 255:
            img[k,j+1] = i
            stack.append((k,j+1))

        #step 3
        if (k>1) and img[k-1,j] ==255:
            img[k-1,j] = i
            stack.append((k-1,j))

        #step 4
        if (j>1) and img[k,j-1] == 255:
            img[k,j-1] = i
            stack.append((k,j-1))

        #step 5
        if k<m and img[k+1,j] == 255:
            img[k+1,j] = i
            stack.append((k+1,j))


        (k,j) = stack.pop()
        if (k,j) == (0,0):
            (k,j) = stack.pop()
            return img,stack


def my_clustering(img):
    (m,n) = np.shape(img)
    #print(n)
    #print(m)
    for k in range(m):
        for j in range(n):
            if img[k,j] == 1:
                img[k,j] = 255
    i = 0
    stack=[]
    for k in range(m):
        for j in range(n):
            if img[k,j] == 255:
                i = i+1
                img, stack = grow(img,k,j,i,stack)

    stacked_img = np.zeros((m,n,i))
    for l in range(i):
        for k in range(m):
            for j in range(n):
                if img[k,j] == l+1:
                    stacked_img[k,j,l] = 1
        #cv2.imshow('test'+str(l),stacked_img[:,:,l])
        #cv2.waitKey(0)
    return stacked_img,i

def extend(xc,yc,phi):
    x1 = xc + np.cos(phi)*(-1000)
    y1 = yc + np.sin(phi)*(-1000)
    x2 = xc + np.cos(phi)*1000
    y2 = yc + np.sin(phi)*1000
    return x1,y1,x2,y2

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 
        'techman_image', self.image_callback, 10)
        self.subscription
        self.publisher_ = self.create_publisher(String, 'coordinate', 10)
    def camera_calibration(self, imgs):
        pixel = 25
        mtx = np.array([[  1.34309615e+03,   0.00000000e+00,   6.55785690e+02],
       [  0.00000000e+00,   1.33923997e+03,   5.15956218e+02],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
        dist = np.array([[ -1.76284945e-01,   2.97229985e+00,   3.59648414e-04,
          5.21315621e-03,  -1.29162192e+01]])
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:6,0:9].T.reshape(-1,2)*pixel
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)*pixel

        
        img = imgs
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (6,9),None)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            # project 3D points to image plane
            rot, __ = cv2.Rodrigues(rvecs)
            rot = np.array(rot)
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            img = draw(img,corners2,imgpts)
            cv2.imwrite('img.png',img)
            # cv2.imshow('img',img)
                # k = cv2.waitKey(0) & 0xFF
                # if k == ord('s'):
                #     cv2.imwrite(fname[:6]+'.png', img)
        # cv2.destroyAllWindows()
        # r = R.from_matrix(rot)
        # eu = r.as_euler('xyz', degrees=True)
        return rot, tvecs

    def image_callback(self, data):
        self.get_logger().info('Received image')
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        
        rot, trans=self.camera_calibration(img)

        msg = String()
        mat = ''
        for i in rot:
            for j in i:
                mat+=str(j)+','
        for i in trans:
            mat+=str(i[0])+','
        # msg.data = str(rot)
        msg.data = mat
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        cv2.imwrite('square.png',img)
        '''
        self.get_logger().info('Received image')
        self.get_logger().info('hiho')
        bridge = CvBridge()
        fn = bridge.imgmsg_to_cv2(data)
        #img=cv2.imread(fn)
        img=fn
        bmg=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        bmg=cv2.GaussianBlur(bmg, (9, 9), 0, 0)
        ret,bmg=cv2.threshold(bmg, 140, 255, cv2.THRESH_BINARY)
        bmg=cv2.erode(bmg,(3,3))

        smg,n = my_clustering(bmg)
        # cv2.imwrite('square.png',img)
        # self.get_logger().info(img.shape)
        # cv2.imshow('clustered',bmg)
        # cv2.waitKey(0)
        aa = 80
        bb = 550
        msg = String()
        for i in range(n):
            bmg=smg[:,:,i]
            m=cv2.moments(bmg)
            xc=m['m10']/m['m00']
            yc=m['m01']/m['m00']
            cv2.circle(img,(int(xc),int(yc)),radius=5,color=(0,0,255), thickness=1)
            phi=1/2*math.atan2(2*m['mu11'],m['mu20']-m['mu02'])
            x1,y1,x2,y2 = extend(xc,yc,phi)
            cv2.line(img,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
            phi = int(180*phi/math.pi)
            # plt.text(aa,bb,"Centroid ("+str(int(xc))+", "+str(int(yc))+")   Principle Angle = "+str(phi)+" degree")
            bb = bb+25
            self.get_logger().info(str(xc)+', '+str(yc)+', '+str(phi))

            msg.data = str(xc)+','+str(yc)+','+str(phi)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        # plt.savefig('~/workspace2/team8_ws/try.png')
        # plt.savefig('try.png')
        cv2.imwrite('clustered.png',img)
        # cv2.imshow('hi',img)
        # cv2.waitKey(0)
    
        # TODO (write your code here)
        '''

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
