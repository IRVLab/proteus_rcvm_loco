#!/usr/bin/env python3

'''
    The RCVM_Pilot_Client is a modified version of the autopilot client from the
    McGill created package aquaautopilot. McGill Copyright and License notice follows below.
'''
################################################################################
# DO NOT MODIFY - AUTO-GENERATED
# 
# Copyright (c) 2016, McGill University / Independent Robotics Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

import rospy
import tf
import sys
from math import pi
from std_msgs.msg import Float32 
from aquacore.msg import Command
from aquacore.msg import AutopilotModes
from geometry_msgs.msg import PoseStamped
from aquacore.msg import StateMsg
from aquacore.srv import SetAutopilotMode  
import dynamic_reconfigure.client

from timeout import Timeout

class RCVMPilotClient:
    
    def __init__(self, params):
        self.mode = params['mode']
        self.listener = tf.TransformListener()
        self.target_pose_pub = rospy.Publisher('/aqua/target_pose', PoseStamped, queue_size=3)
        self.current_depth = 0

        rospy.wait_for_service('/aqua/set_3Dauto_mode')
        rospy.wait_for_message('/AP_filtered_depth', Float32)
        
        self.depth_sub = rospy.Subscriber("/AP_filtered_depth", Float32, self.depth_callback)
        self.set_3d_auto_mode = rospy.ServiceProxy('/aqua/set_3Dauto_mode', SetAutopilotMode)
        
        self.listener.waitForTransform('/latest_fix', '/aqua_base', rospy.Time(0), rospy.Duration(4))

        self.dyn_reconf = dynamic_reconfigure.client.Client("/AP_depth_filter", timeout=5)  
        self.original_params = self.dyn_reconf.get_configuration(timeout=5) 
        print('Previous value of window_size_sec  was: ' + str(self.original_params['window_size_sec']))

        rospy.loginfo('Setting window_size_sec to ' + str(0.5)) 
        self.dyn_reconf.update_configuration({'window_size_sec':float(0.5)}) 
        rospy.loginfo('Returned from update_configuration')

        try:
            self.resp1 = self.set_3d_auto_mode(mode=self.mode)
        except rospy.ServiceException as exc:
            print("Could not set 3d autopilot mode")

    def set_mode(self,params):
        self.mode = params['mode']
        try:
            self.resp1 = self.set_3d_auto_mode(mode=self.mode)
        except rospy.ServiceException as exc:
            print("Could not set 3d autopilot mode")

    def angle_diff(self, from_theta, to_theta):
        diff = to_theta - from_theta
        if (diff > pi):
            diff = diff - 2*pi
    
        if (diff < -pi):
            diff = diff + 2*pi
        
        return diff

    def clamp180(self,angle):
        while angle > 180.0:
            angle = angle - 360.0
        while angle < -180.0:
            angle = angle + 360.0
            
        return angle

    def DEG2RAD(self, degs):
        return degs * (pi/180)
        
    def RAD2DEG(self, rads):
        return rads * (180/pi)

   
    def get_rpy_of_imu_in_global(self):
        self.listener.waitForTransform('/latest_fix', '/aqua_base', rospy.Time(0), rospy.Duration(4))
        (position_of_imu_in_global, rotation_from_imu_to_global) = \
            self.listener.lookupTransform('/latest_fix', '/aqua_base', rospy.Time(0))
        rpy_from_imu_to_global = tf.transformations.euler_from_quaternion(rotation_from_imu_to_global)
        return rpy_from_imu_to_global


    def depth_callback(self, msg):
        self.current_depth = msg.data

    def do_straight_line(self, dt_in_sec, target_angles_in_deg, target_depth, vx, vz):
    
        if type(dt_in_sec) == int:
            dt_in_sec = rospy.Duration(dt_in_sec)
        rate = rospy.Rate(10)
        started_at = rospy.Time.now()

        #while (not rospy.is_shutdown()) and ((rospy.Time.now() - started_at).to_sec() < dt_in_sec):
        while (not rospy.is_shutdown()) and (rospy.Time.now() < (started_at + dt_in_sec)):
            rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                      target_angles_in_deg[1]*pi/180, 
                                                                                      target_angles_in_deg[2]*pi/180)
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx # forward speed
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel)
            rate.sleep()

    def goto_target_orientation(self, target_angles_in_deg, target_depth, vx, vz, threshold=7, check_roll=True, check_pitch=True, check_yaw=True, timeout=None):
        
        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_angles_in_deg[0]*pi/180, 
                                                                                  target_angles_in_deg[1]*pi/180, 
                                                                                  target_angles_in_deg[2]*pi/180)

        target_angles = tf.transformations.euler_from_quaternion(rotation_from_target_to_global)

        rate = rospy.Rate(10.0)
        if timeout != None:
            finish = rospy.Time.now() + rospy.Duration.from_sec(timeout)

        while (not rospy.is_shutdown()) and \
          ( ((not check_roll) or (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > threshold)) or \
            ((not check_pitch) or (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > threshold)) or \
            ((not check_yaw) or (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > threshold))) \
            and ((not timeout) or (rospy.Time.now() < finish )):
                
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'
            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()

            print('angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                                abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                                abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi))
            
    def do_relative_angle_change(self, delta_angles_deg, target_depth, vx, vz, time_sec=None, threshold=7, check_roll=True, check_pitch=True, check_yaw=True, timeout=None):

        print('da: ', delta_angles_deg)
        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        
        print('rpy: ', (rpy_from_imu_to_global[0]*180/pi, rpy_from_imu_to_global[1]*180/pi, rpy_from_imu_to_global[2]*180/pi))
        target_roll  = rpy_from_imu_to_global[0] + delta_angles_deg[0]*pi/180
        target_pitch = rpy_from_imu_to_global[1] + delta_angles_deg[1]*pi/180
        target_yaw   = rpy_from_imu_to_global[2] + delta_angles_deg[2]*pi/180
        print('Ta: ', (target_roll*180/pi, target_pitch*180/pi, target_yaw*180/pi))
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(target_roll, 
                                                                                  target_pitch, 
                                                                                  target_yaw)

        target_angles = tf.transformations.euler_from_quaternion(rotation_from_target_to_global)
        print('Trimmed ta: ', (target_angles[0]*180/pi, target_angles[1]*180/pi, target_angles[2]*180/pi))
        
        print('angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                               abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                               abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi))
        
        rate = rospy.Rate(10.0)
        if timeout != None:
            finish = rospy.Time.now() + rospy.Duration.from_sec(timeout)
        
        while (not rospy.is_shutdown()) and \
          ( ((not check_roll) or (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > threshold)) or \
            ((not check_pitch) or (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > threshold)) or \
            ((not check_yaw) or (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > threshold))) \
            and ((not timeout) or (rospy.Time.now() < finish )):
            
          pose_and_vel = PoseStamped()
          pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
          pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
          pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
          pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
          pose_and_vel.pose.position.x = vx
          pose_and_vel.pose.position.y = vz # heave
          pose_and_vel.pose.position.z = target_depth 
          pose_and_vel.header.stamp = rospy.Time.now()
          pose_and_vel.header.frame_id = '/latest_fix'

          self.target_pose_pub.publish(pose_and_vel) 
          rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
          rate.sleep()
                
          print('angle diff: ', (abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi, 
                                abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi, 
                                abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi))
            
    def do_relative_depth_change(self, dz, vx, vz):

        rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
        rotation_from_target_to_global = tf.transformations.quaternion_from_euler(rpy_from_imu_to_global[0], 
                                                                                  rpy_from_imu_to_global[1], 
                                                                                  rpy_from_imu_to_global[2])
        target_angles = rpy_from_imu_to_global
        target_depth = self.current_depth + dz
        rate = rospy.Rate(10.0)
        
        while (not rospy.is_shutdown()) and \
                ((abs(self.angle_diff(rpy_from_imu_to_global[0], target_angles[0]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[1], target_angles[1]))*180/pi > 5) or \
                 (abs(self.angle_diff(rpy_from_imu_to_global[2], target_angles[2]))*180/pi > 5) or \
                 (abs(target_depth - self.current_depth) > 0.2)):
            
            pose_and_vel = PoseStamped()
            pose_and_vel.pose.orientation.x = rotation_from_target_to_global[0]
            pose_and_vel.pose.orientation.y = rotation_from_target_to_global[1]
            pose_and_vel.pose.orientation.z = rotation_from_target_to_global[2]
            pose_and_vel.pose.orientation.w = rotation_from_target_to_global[3]
            pose_and_vel.pose.position.x = vx
            pose_and_vel.pose.position.y = vz # heave
            pose_and_vel.pose.position.z = target_depth 
            pose_and_vel.header.stamp = rospy.Time.now()
            pose_and_vel.header.frame_id = '/latest_fix'

            self.target_pose_pub.publish(pose_and_vel) 
            rpy_from_imu_to_global = self.get_rpy_of_imu_in_global()
            rate.sleep()
