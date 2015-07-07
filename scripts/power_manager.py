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

import rospy

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sexy_jarvis.srv import WakeOnLan
from sexy_jarvis.srv import WakeOnLanResponse

def handle_wake_on_lan(req):
    rospy.logdebug('TODO: Waking up %s', req.mac_address)
    return WakeOnLanResponse()

def handle_power_on(req):
    rospy.logdebug('TODO: Turning power on')
    return EmptyResponse()

def handle_power_off(req):
    rospy.logdebug('TODO: Turning power off')
    return EmptyResponse()

def power_manager():
    rospy.init_node('power_manager', log_level=rospy.DEBUG)
    wake_on_lan_service = rospy.Service('wake_on_lan', WakeOnLan, handle_wake_on_lan)
    power_on_service    = rospy.Service('power_on',    Empty,     handle_power_on)
    power_off_service   = rospy.Service('power_off',   Empty,     handle_power_off)
    rospy.logdebug('Power manager started')
    rospy.spin()

if __name__ == "__main__":
    power_manager()

