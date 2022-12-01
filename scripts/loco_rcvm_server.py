#! /usr/bin/python

import sys, math, threading, signal
from time import sleep
from math import pi

from loco_pilot.msg import AutopilotModes
from rcvm_autopilot_client import RCVMPilotClient

import rospy
from rosnode import get_node_names
from tf.transformations import euler_from_quaternion
import roslaunch
import rosnode 

import xml.etree.ElementTree as ET
from proteus.kineme import Kineme, KNode, KNodeAbsolute, KNodePause, KNodeDepth, KNodeDirectional, KNodeQuantity
from proteus_msgs.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity

rospy.init_node('loco_rcvm_server', argv=None, anonymous=True)

params = {}
params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
rpc = RCVMPilotClient(params)

rcvm_params = None
symbols = None
kinemes = None

# Farms out the execution of the kineme to the appropriate function
def service_cb(req, kineme):
    rospy.logdebug('Service callback for kineme %s'%(kineme.id))
    if kineme.call_type == 'trigger':
        return execute_trigger(req, kineme)
    elif kineme.call_type == 'directional':
        return execute_directional(req, kineme)
    elif kineme.call_type == 'target':
        return execute_target(req, kineme)
    elif kineme.call_type == 'quantity':
        return execute_quantity(req, kineme)
    else:
        return False

# Executes kinemes which are trigger called, meaning that there's no information in the service call.
def execute_trigger(req, kineme):
    rospy.loginfo('Executing trigger kineme %s'%(kineme.id))
    for knode in kineme.knodes:
        if type(knode) == KNodeAbsolute:
            if knode.orientation.roll == 0.0 and knode.orientation.pitch == 0.0 and knode.orientation.yaw == 0:
                rpc.do_straight_line(knode.duration.seconds, (knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw),rpc.current_depth, knode.velocity.surge, knode.velocity.heave)
            else:
                rpc.do_relative_angle_change((knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw), rpc.current_depth, knode.velocity.surge, knode.velocity.heave, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'], timeout=knode.duration.seconds * 1.5)
        elif type(knode) == KNodeDepth:
            if knode.depth.mode == 'relative':
                rpc.do_relative_depth_change(knode.depth.amount, knode.velocity.surge, knode.velocity.heave)
        elif type(knode) == KNodePause:
            sleep(knode.duration.seconds)
        else:
            return False
    return True
    
# Executes kinemes which are directional called, meaning that there's directional information in the service call.
def execute_directional(req, kineme):
    rospy.loginfo('Executing directional kineme %s'%(kineme.id))
    transform = req.transform
    pos = transform.translation
    orr = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])

    for knode in kineme.knodes:
        if type(knode) == KNodeAbsolute:
            if knode.orientation.roll == 0.0 and knode.orientation.pitch == 0.0 and knode.orientation.yaw == 0:
                rpc.do_straight_line(knode.duration.seconds, (knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw),rpc.current_depth, knode.velocity.surge, knode.velocity.heave)
            else:
                rpc.do_relative_angle_change((knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw), rpc.current_depth, knode.velocity.surge, knode.velocity.heave, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'], timeout=knode.duration.seconds * 1.5)
        elif type(knode) == KNodeDirectional:
            rpc.do_relative_angle_change((rpc.RAD2DEG(orr[0]), rpc.RAD2DEG(orr[1]), rpc.RAD2DEG(orr[2])), rpc.current_depth, 0, 0, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'], timeout=knode.duration.seconds * 1.5)
        elif type(knode) == KNodeDepth:
            if knode.depth.mode == 'relative':
                rpc.do_relative_depth_change(knode.depth.amount, knode.velocity.surge, knode.velocity.heave)
        elif type(knode) == KNodePause:
            sleep(knode.duration.seconds)
            
    return True

# Executes kinemes which are target called, meaning that there is a need to connect the kineme to a target.
def execute_target(req, kineme):
    return False

def execute_quantity(req, kineme):
    for knode in kineme.knodes:
        if type(knode) == KNodeAbsolute:
            if knode.orientation.roll == 0.0 and knode.orientation.pitch == 0.0 and knode.orientation.yaw == 0:
                rpc.do_straight_line(knode.duration.seconds, (knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw),rpc.current_depth, knode.velocity.surge, knode.velocity.heave)
            else:
                rpc.do_relative_angle_change((knode.orientation.roll, knode.orientation.pitch, knode.orientation.yaw), rpc.current_depth, knode.velocity.surge, knode.velocity.heave, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'], timeout=knode.duration.seconds * 1.5)
        elif type(knode) == KNodeDepth:
            if knode.depth.mode == 'relative':
                rpc.do_relative_depth_change(knode.depth.amount, knode.velocity.surge, knode.velocity.heave)
        elif type(knode) == KNodePause:
            sleep(knode.duration.seconds)
        elif type(knode) == KNodeQuantity:
            if knode.quantity.display_on == 'heave':
                depth_change = knode.quantity.amount * req.quantity
                rpc.do_relative_depth_change(depth_change, 0, 0.3)
            elif knode.quantity.display_on == 'pitch':
                pitch_angle = knode.quantity.amount * req.quantity
                rpc.do_relative_angle_change((0, pitch_angle, 0), rpc.current_depth, 0, 0, knode.duration.seconds, threshold=rcvm_params['angle_diff_threshold'], timeout=knode.duration.seconds * 1.5)
        else:
            return False
    return True

if __name__ == '__main__':
    rospy.loginfo('Initializing the LoCO RCVM server')

    rcvm_params = {"angle_diff_threshold": 5.0}

    #Check if PROTEUS language server is up
    rospy.loginfo('Checking PROTEUS language server...')
    lang_server_active = False
    nodes = get_node_names()
    rospy.logdebug(nodes)
    for n in nodes:
        if n.split('/')[-1] == 'proteus_language_server':
            lang_server_active = True
            break
    if not lang_server_active:
        rospy.logerr("This RCVM implementation requires the PROTEUS language server to be active.")
        sys.exit(1)
    else:
        rospy.loginfo('PROTEUS language server OK!')

    # Find kineme language definition file
    rospy.loginfo("Loading RCVM vector information...")
    rcvm_info = rospy.get_param('vectors/out/RCVM')
    rcvm_def_file = rcvm_info['definition_file']

    # Find symbol definitions
    rospy.loginfo("Loading symbol information...")
    symbols = rospy.get_param('symbols/out')
    
    # Process kineme definition file into kineme objects
    rospy.loginfo("Loading Kineme definitions from kineme definition file.")
    kinemes = dict()

    #Load XML file
    tree = ET.parse(rcvm_def_file)
    root = tree.getroot()
    for kdef in root:
        k = Kineme()
        k.parse_from_xml(kdef)
        kinemes[k.id] = k

    # Check for symbol matchup.
    for s in symbols:
        for key in kinemes:
            k = kinemes[key]
            if s == k.id:
                rospy.loginfo("Found match beteween symbol %s and kineme %s, associating data."%(s, k.id))
                rospy.logdebug("Call type: %s"%(symbols.get(s).get('call_type')))
                k.set_call_type(symbols.get(s).get('call_type'))
                break

    for key,kineme in kinemes.items():
        print(key, kineme)
    
    # Setup service calls
    for key, kineme in kinemes.items():
        service_class = None
        if kineme.call_type == 'trigger':
            service_class = SymbolTrigger
        elif kineme.call_type == 'directional':
            service_class = SymbolDirectional
        elif kineme.call_type == 'target':
            service_class = SymbolTarget
        elif kineme.call_type == 'quantity':
            service_class = SymbolQuantity
        else:
            rospy.logwarn("Unexpected kineme call type %s"%(kineme.call_type))

        service_name = 'rcvm/'+ kineme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for kineme %s at service endpoint: %s'%(kineme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, kineme=kineme: service_cb(req, kineme))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass