#!/usr/bin/env python

from numpy.core.fromnumeric import shape
import rclpy

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
from std_msgs.msg import String
from rclpy.node import Node

import cv2
import math
import numpy as np
hight = 110

# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

# gripper client
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

def drawing(x, y, z, a=-180, b=0, c=135 ):
    targetP1 = str(x)+", "+str(y) + ", " + str(z) + ", "+str(a) +", "+str(b) +", "+str(c)
    script = "PTP(\"CPP\","+targetP1+",400,200,0,false)"
    send_script(script)

def Eu2mtx(ini, E_x, E_y, E_z):
    R_x =  np.array([[1, 0, 0], [0, np.cos(E_x), -np.sin(E_x)], [0, np.sin(E_x), np.cos(E_x)]])
    R_y =  np.array([[np.cos(E_y), 0, np.sin(E_y)], [0, 1, 0], [-np.sin(E_y), 0, np.cos(E_y)]])
    R_z =  np.array([[np.cos(E_z), -np.sin(E_z), 0], [np.sin(E_z), np.cos(E_z), 0], [0, 0, 1]])
    R = R_x @ R_y @ R_z
    R = np.concatenate( (np.concatenate((R, ini), axis=1), np.array([0, 0, 0, 1]).reshape((1,4))), axis=0)
    return R

def Mtx2eu(rot):
    tx = np.arctan2(rot[2][1],rot[2][2])*180/np.pi
    ty = np.arctan2(rot[2][0],np.sqrt( np.square(rot[2][1]) + np.square(rot[2][2]) ))*180/np.pi
    tz = np.arctan2(rot[1][0],rot[0][0])*180/np.pi
    return tx, ty, tz

def xyrot(rot):
    q2 = 1/np.sqrt(2)
    x1 = (rot[0]*q2-rot[1]*q2)
    y1 = -(rot[0]*q2+rot[1]*q2)
    return x1, y1 

def draw_pixel(x, y, height, rotm, rot, tx,ty,tz, ax, ay):
        pen_offset=np.matmul(rotm, np.array([x+ax,y+ay,height]))
        x1, y1 = xyrot(rot[9:11]+[0, -80])
        x2, y2 = xyrot(pen_offset)
        drawing(400+x1-x2, 300+y1-y2,700-rot[11]+pen_offset[2],-180-tx,-ty    ,135-tz)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'coordinate',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        # rot = np.array(msg.data)
        rot = np.array(msg.data.split(","))
        rot = rot[:-1].astype(np.float)
        rotm = rot[:9].reshape((3,3))
        tx, ty, tz = Mtx2eu(rotm)
        tvex = rot[9:].reshape((3,1))
        
        # pen_offset=np.matmul(rotm, np.array([0,0,100]))
        # print(pen_offset)
        # x1, y1 = xyrot(rot[9:11])
        # x2, y2 = xyrot(pen_offset)
        # drawing(400+x1-x2, 300+y1-y2,700-rot[11]+pen_offset[2],-180-tx,-ty    ,135-tz)
        ax = -25*3
        ay = -25*4
        # ax, ay = 0, 0
        f = open("path_0105.txt")
        kk=0
        a = 0
        flag = True
        height=135 #short tool: 125
        for x in f:
            if len(x)>2:
                if kk%5 == 0:
                    x = x.replace('\r\n','')
                    a = np.array(x.split(',')).astype('float')
                    a = a/10-40
                    if flag:
                        draw_pixel(a[0], a[1], height+10, rotm, rot, tx, ty, tz, ax, ay)
                        flag = False
                    draw_pixel(a[0], a[1], height, rotm, rot, tx, ty, tz, ax, ay)
                kk=kk+1
            else:
                draw_pixel(a[0], a[1], height+10, rotm, rot, tx, ty, tz, ax, ay)
                flag = True
                 

        # draw_pixel(20,0, 170, rotm, rot, tx, ty, tz, ax, ay)        
        # draw_pixel(20,20, 400, rotm, rot, tx, ty, tz, ax, ay)        
        # draw_pixel(0,20, 400, rotm, rot, tx, ty, tz, ax, ay)        
        # draw_pixel(0,0, 400, rotm, rot, tx, ty, tz, ax, ay)        
        # pen_offset=np.matmul(rotm, np.array([20,0,100]))
        # x1, y1 = xyrot(rot[9:11])
        # x2, y2 = xyrot(pen_offset)
        # drawing(400+x1-x2, 300+y1-y2,700-rot[11]+pen_offset[2],-180-tx,-ty    ,135-tz)

        # pen_offset=np.matmul(rotm, np.array([20,20,100]))
        # x1, y1 = xyrot(rot[9:11])
        # x2, y2 = xyrot(pen_offset)
        # drawing(400+x1-x2, 300+y1-y2,700-rot[11]+pen_offset[2],-180-tx,-ty    ,135-tz)
        
        
def main(args=None):
    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    rclpy.init(args=args)
    #--- move command by joint angle ---#

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#

    drawing(400, 300, 700)

    
    

# What does Vision_DoJob do? Try to use it...
# -------------------------------------------------
    send_script("Vision_DoJob(job1)")
    # cv2.waitKey(1)
    # send_script("Vision_DoJob(job1)")
    # cv2.waitKey(1)
#--------------------------------------------------

    set_io(0.0) # 1.0: close gripper, 0.0: open gripper
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


    

if __name__ == '__main__':
    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    main()
    
