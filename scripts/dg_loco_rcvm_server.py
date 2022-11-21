#! /usr/bin/python3

import sys, math, threading, signal
from time import sleep
from math import pi

import rospy
from rosnode import get_node_names
from loco_pilot.msg import Command
from std_msgs.msg import Header

import xml.etree.ElementTree as ET
from proteus.kineme import Kineme, KNodeDeadGuess, KNodePause, KNodeQuantity
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity

rospy.init_node('dg_loco_rcvm_server', argv=None, anonymous=True)

cmd_queue = []

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
    global cmd_queue
    rospy.loginfo('Executing trigger kineme %s'%(kineme.id))
    for knode in kineme.knodes:
        #HACK
        number_of_messages = int(knode.duration.seconds * 10)
        if type(knode) == KNodeDeadGuess:
            c = Command()
            c.header = Header()
            c.throttle = knode.command.throttle
            c.pitch = knode.command.pitch
            c.yaw = knode.command.yaw
            cmd_queue.extend([c]*number_of_messages)
        elif type(knode) == KNodePause:
            cmd_queue.extend([None]*number_of_messages)
        else:
            return False
    return True
    
# Executes kinemes which are directional called, meaning that there's directional information in the service call.
def execute_directional(req, kineme):
    return False

# Executes kinemes which are target called, meaning that there is a need to connect the kineme to a target.
def execute_target(req, kineme):
    return False

def execute_quantity(req, kineme):
    for knode in kineme.knodes:
        number_of_messages = int(knode.duration.seconds * 10)
        if type(knode) == KNodeDeadGuess:
            c = Command()
            c.header = Header()
            c.throttle = knode.command.throttle
            c.pitch = knode.command.pitch
            c.yaw = knode.command.yaw
            cmd_queue.extend([c]*number_of_messages)
        elif type(knode) == KNodePause:
            cmd_queue.extend([None]*number_of_messages)
        elif type(knode) == KNodeQuantity:
            pitch_value = req.quantity * knode.quantity.max_amount

            c = Command()
            c.throttle = 0
            c.pitch = pitch_value
            c.yaw = 0
            cmd_queue.extend([c]*number_of_messages)
        else:
            return False
    return True

if __name__ == '__main__':
    rospy.loginfo('Initializing the LoCO RCVM server')

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

        cmd_pub = rospy.Publisher('/loco/command', Command, queue_size=5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if len(cmd_queue) > 0:
            top = cmd_queue.pop(0)
            if type(top) == Command:
                cmd_pub.publish(top)
            elif type(top) == None:
                rospy.sleep(0.1)

        rate.sleep()

else:
    pass