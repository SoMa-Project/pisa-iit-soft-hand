#!/usr/bin/env python
import rospy
from qb_interface.msg import *
from soft_hand_ros_control.srv import *
from threading import Lock

class PisaHandController:

    def __init__(self):
        rospy.init_node('pisa_hand_controller', anonymous=False)
        self.closure = None
        self.current = None
        self.steady_state_current = None
        self.hand_control_enabled = False
        self.hand_cmd = handRef()
        self.closure_lock = Lock()
        self.current_lock = Lock()
        self.hand_control_lock = Lock()
        self.k = None
        self.xr = None
        rospy.Service('hand_cmd', hand_cmd, self.hand_cmd_callback)
        rospy.Service('enable_control', enable_control, self.enable_control_callback)
        self.closure_sub = rospy.Subscriber("/qb_class/hand_measurement", handPos, self.closure_callback)
        self.current_sub = rospy.Subscriber("/qb_class/hand_current", handCurrent, self.current_callback)
        self.current_pub = rospy.Publisher("/qb_class/hand_ref", handRef, queue_size=1)

    def closure_callback(self, data):
    	with self.closure_lock:
    	    self.closure = data.closure[0]
    	if not self.hand_control_enabled or (self.xr is None or self.k is None) or (self.xr < 50 and self.closure < 50) or self.closure > 18900:
    	    self.hand_cmd.closure = [0,]
            self.current_pub.publish(self.hand_cmd)
    	else:
            self.hand_cmd.closure = [min(max(self.k * (self.xr - self.closure), -500), 1500),]
            if abs(self.current - self.hand_cmd.closure[0]) < 20 and self.steady_state_current is None:
                self.steady_state_current = self.hand_cmd.closure[0]
            elif abs(self.current - self.hand_cmd.closure[0]) < 20 and self.steady_state_current is not None:
                self.hand_cmd.closure = [self.steady_state_current,]
            else:
                self.steady_state_current = None
        # print "Commanding current: " + str(self.hand_cmd.closure[0]) 
        # print "Measured current is: " + str(self.current)
        self.current_pub.publish(self.hand_cmd)
        
          

    def current_callback(self, data):
        with self.current_lock:
            self.current = data.current[0]    	
		
    def hand_cmd_callback(self, req):
        with self.hand_control_lock:
            self.k = req.k
            self.xr = req.xr
        return hand_cmdResponse()

    def enable_control_callback(self, req):
        with self.hand_control_lock:
            self.hand_control_enabled = req.enable
        return enable_controlResponse()


if __name__ == '__main__':
	pisa_hand_controller = PisaHandController()
	rospy.spin()
    
