#!/usr/bin/python
import rospy
from loco_pilot.msg import AutopilotModes
from rcvm_autopilot_client import RCVMPilotClient

if __name__ == '__main__':
	rospy.init_node('APC_test')
	params = {}
	params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
	ap = RCVMPilotClient(params)

	rospy.loginfo("Attempting straight line motion for 5 seconds, low vx.")
	ap.do_straight_line(5, ap.get_rpy_of_imu_in_global(), ap.current_depth, vx=0.2)
	rospy.loginfo("Attempting straight line motion for 3 seconds, high vx.")
	ap.do_straight_line(3, ap.get_rpy_of_imu_in_global(), ap.current_depth, vx=0.6)

	rospy.loginfo("Attempting straight line motion [reversed] for 5 seconds, low vx.")
	ap.do_straight_line(5, ap.get_rpy_of_imu_in_global(), ap.current_depth, vx=-0.2)
	rospy.loginfo("Attempting straight line motion [reversed] for 3 seconds, high vx.")
	ap.d_straight_line(3, ap.get_rpy_of_imu_in_global(), ap.current_depth, vx=-0.6)


#    rospy.loginfo("Attemping roll of 20")
#    ap.do_relative_angle_change((20,0,0), ap.current_depth, 0.1, threshold=2, timeout=10)
#    rospy.loginfo("Going back ")
#    ap.do_relative_angle_change((-20,0,0), ap.current_depth, 0.1,threshold=2, timeout=10)

#    rospy.loginfo("Attemping pitch of 20")
#    ap.do_relative_angle_change((0,-20,0), ap.current_depth, 0.1,threshold=2, timeout=10)
#    rospy.loginfo("Going back ")
#    ap.do_relative_angle_change((0,20,0), ap.current_depth, 0.1, threshold=2, timeout=10)

#    rospy.loginfo("Attemping yaw of 45")
#    ap.do_relative_angle_change((0,0,45), ap.current_depth, 0.1, threshold=2, timeout=10)
#    rospy.loginfo("Going back")
#    ap.do_relative_angle_change((0,0,-45), ap.current_depth, 0.1,threshold=2, timeout=10) 

#    rospy.loginfo("Same, but no vx")
#    rospy.loginfo("Attemping roll of 20")
#    ap.do_relative_angle_change((20,0,0), ap.current_depth, 0.1, threshold=2, timeout=10)
#    rospy.loginfo("Going back ")
#    ap.do_relative_angle_change((-20,0,0), ap.current_depth, 0.1, threshold=2, timeout=10)

#    rospy.loginfo("Attemping pitch of 20")
#    ap.do_relative_angle_change((0,-20,0), ap.current_depth, 0.1, threshold=2, timeout=10)
#    rospy.loginfo("Going back ")
#    ap.do_relative_angle_change((0,20,0), ap.current_depth, 0.1, threshold=2, timeout=10)

#    rospy.loginfo("Attemping yaw of 45")
#    ap.do_relative_angle_change((0,0,45), ap.current_depth, 0.1, threshold=2, timeout=10)
#    rospy.loginfo("Going back")
#    ap.do_relative_angle_change((0,0,-45), ap.current_depth, 0.1, threshold=2, timeout=10) 


#    rospy.loginfo("Do depth change. -.1 meters.")
#    ap.do_relative_depth_change(-0.1, 0, 0.3)

#    rospy.loginfo("Do depth change. -.5 meters.")
#    ap.do_relative_depth_change(-0.5, 0, 0.3)
