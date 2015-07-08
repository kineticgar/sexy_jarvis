#!/usr/bin/env python
################################################################################
#
#    Copyright (c) 2015 Garrett Brown
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in
#    all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
################################################################################

from wakeonlan import wol

import rospy
import subprocess

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sexy_jarvis.srv import StartCamera
from sexy_jarvis.srv import StartCameraResponse
from sexy_jarvis.srv import WakeOnLan
from sexy_jarvis.srv import WakeOnLanResponse

################################################################################
#
# Globals
#
################################################################################

NODE_NAME = 'power_manager'

################################################################################
#
# Computer properties
#
################################################################################

class Computer(object):
    host_name  = None
    ip_address = None

    def __init__(self, host_name, ip_address):
        self.host_name  = host_name
        self.ip_address = ip_address

    @staticmethod
    def FromParams(host_name, params):
        computer = None
        if 'ip_address' not in params:
            rospy.logdebug('Machine %s has no parameter "ip_address"', host_name)
        else:
            computer = Computer(host_name, params['ip_address'])
        return computer

class NetworkComputer(Computer):
    mac_address = None

    def __init__(self, host_name, ip_address, mac_address):
        super(NetworkComputer, self).__init__(host_name, ip_address)
        self.mac_address = mac_address

    @staticmethod
    def FromParams(host_name, params):
        computer = None
        base = Computer.FromParams(host_name, params)
        if base:
            if 'mac_address' not in params:
                rospy.logdebug('Machine %s has no parameter "mac_address"', base.host_name)
            else:
                computer = NetworkComputer(base.host_name,
                                           base.ip_address,
                                           params['mac_address'])
        return computer

class CameraComputer(Computer):
    camera     = None
    camera_fps = None

    def __init__(self, host_name, ip_address, camera, camera_fps):
        super(CameraComputer, self).__init__(host_name, ip_address)
        self.camera     = camera
        self.camera_fps = camera_fps

    @staticmethod
    def FromParams(host_name, params):
        computer = None
        base = Computer.FromParams(host_name, params)
        if base:
            if 'camera' not in params:
                rospy.logdebug('Machine %s has no parameter "camera"', base.host_name)
            elif 'camera_fps' not in params:
                rospy.logdebug('Machine %s has no parameter "camera_fps"', base.host_name)
            else:
                computer = CameraComputer(base.host_name,
                                          base.ip_address,
                                          params['camera'],
                                          params['camera_fps'])
        return computer

################################################################################
#
# service handlers
#
################################################################################

def handle_wake_on_lan(req):
    machines = rospy.get_param('machines')
    if req.machine_name not in machines:
        rospy.logerror('No parameters found for machine "%s"', req.machine_name)
    else:
        rospy.logdebug('Waking up %s', req.machine_name)
        computer = NetworkComputer.FromParams(req.machine_name, machines[req.machine_name])
        if computer:
            wol.send_magic_packet(computer.mac_address)
    return WakeOnLanResponse()

def handle_start_camera(req):
    machines = rospy.get_param('machines')
    if req.machine_name not in machines:
        rospy.logerror('No parameters found for machine "%s"', req.machine_name)
    else:
        rospy.logdebug('Starting image pipeline for %s', req.machine_name)
        computer = CameraComputer.FromParams(req.machine_name, machines[req.machine_name])
        if computer:
            subprocess.Popen(['roslaunch',
                              'sexy_jarvis',
                              'camera.launch',
                              'NAMESPACE:=%s' % rospy.get_namespace(),
                              'MACHINE:=%s' % computer.host_name,
                              'MACHINE_IP:=%s' % computer.ip_address,
                              'DEVICE:=%s' % computer.camera,
                              'FPS:=%s' % computer.camera_fps])
    return StartCameraResponse()

def handle_power_on(req):
    rospy.logdebug('Turning power on')
    wake_on_lan = rospy.ServiceProxy('wake_on_lan', WakeOnLan)
    start_camera = rospy.ServiceProxy('start_camera', StartCamera)
    machines = rospy.get_param('machines')
    for machine_name in machines:
        computer = Computer.FromParams(machine_name, machines[machine_name])
        if computer:
            try:
                wake_on_lan(computer.host_name)
                start_camera(computer.host_name)
            except rospy.ServiceException as e:
                rospy.logerr('Service call failed: %s', e)
    return EmptyResponse()

def handle_power_off(req):
    rospy.logdebug('TODO: Turning power off')
    return EmptyResponse()

################################################################################
#
# power_manager() - node entry point
#
################################################################################

def power_manager():
    rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
    wake_on_lan_service  = rospy.Service('wake_on_lan',  WakeOnLan,   handle_wake_on_lan)
    start_camera_service = rospy.Service('start_camera', StartCamera, handle_start_camera)
    power_on_service     = rospy.Service('power_on',     Empty,       handle_power_on)
    power_off_service    = rospy.Service('power_off',    Empty,       handle_power_off)
    rospy.logdebug('Power manager started')
    rospy.spin()

if __name__ == "__main__":
    power_manager()

