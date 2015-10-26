# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.


# Copyright (c) 2015  Niklas Rosenstein
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


from __future__ import print_function

import myo as libmyo; libmyo.init()
import time
import sys
import math

from threading import Thread
import logging
sys.path.append("lib")
import cflib  # noqa
from cflib.crazyflie import Crazyflie  # noqa

logging.basicConfig(level=logging.ERROR)

channel = 75
connected = False

class Listener(libmyo.DeviceListener):

    interval = 0.05  # Output only 0.05 seconds

    def __init__(self):
        super(Listener, self).__init__()
        self.orientation = None
        self.pose = libmyo.Pose.rest
        self.emg_enabled = False
        self.locked = False
        self.rssi = None
        self.emg = None
        self.last_time = 0

        link_uri = "radio://0/80/2M"
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        
        self.roll_h = 0
        self.pitch_h = 0
        self.yaw_h = 0
        self.holder = 0
        self.lock = 1

        print("Connecting to %s" % link_uri)

    #MyoBand
    def on_connect(self, myo, timestamp, firmware_version):
        myo.vibrate('short')
        myo.vibrate('short')
        myo.request_rssi()
        myo.request_battery_level()

    def on_rssi(self, myo, timestamp, rssi):
        self.rssi = rssi

    def on_pose(self, myo, timestamp, pose):
        if pose == libmyo.Pose.double_tap:
            self.holder = 0
            print("tap")
        elif pose == libmyo.Pose.fingers_spread:
            self._cf.commander.send_setpoint(0, 0, 0, 0)	#Unlocks thrust
            self.lock = 0
            print("finger")
        elif pose == libmyo.Pose.fist:
            self._cf.commander.send_setpoint(0, 0, 0, 0)	#Clear packets
            time.sleep(0.1)
            self.lock = 1
            self._cf.close_link()
            print("fist")
        self.pose = pose

    def on_orientation_data(self, myo, timestamp, quat):
        pitch = float(math.atan2(2.0 * (quat.w * quat.x + quat.y * quat.z),1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)))
        thrust = float(math.asin(max(-1.0, min(1.0, 2.0 * (quat.w * quat.y - quat.z * quat.x)))))
        roll = float(math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)))
        if self.holder == 0:
            self.pitch_h = int((pitch + math.pi)/(math.pi*2.0) * 100)
            self.thrust_h = int((thrust + math.pi/2.0)/math.pi * 70000)
            self.roll_h = int((roll + math.pi)/(math.pi*2.0) * 100)
            self.holder = 1
        pitch_w = int((pitch + math.pi)/(math.pi*2.0) * 100) - self.pitch_h
        thrust_w = abs(int((thrust + math.pi/2.0)/math.pi * 70000) - self.thrust_h)
        roll_w = int((roll + math.pi)/(math.pi*2.0) * 100) - self.roll_h

        print (pitch_w,"          ",roll_w,"          ",thrust_w)

        yaw = 0
        self._cf.commander.send_setpoint(roll_w, pitch_w, yaw, thrust_w)

        self.orientation = quat

    def on_accelerometor_data(self, myo, timestamp, acceleration):
        pass

    def on_gyroscope_data(self, myo, timestamp, gyroscope):
        pass

    def on_emg_data(self, myo, timestamp, emg):
        self.emg = emg

    def on_unlock(self, myo, timestamp):
        self.locked = False

    def on_lock(self, myo, timestamp):
        self.locked = True

    def on_event(self, kind, event):
        pass

    def on_event_finished(self, kind, event):
        pass

    def on_pair(self, myo, timestamp, firmware_version):
        pass

    def on_unpair(self, myo, timestamp):
        pass

    def on_disconnect(self, myo, timestamp):
        pass

    def on_arm_sync(self, myo, timestamp, arm, x_direction, rotation,
                    warmup_state):
        pass

    def on_arm_unsync(self, myo, timestamp):
        pass

    def on_battery_level_received(self, myo, timestamp, level):
        pass

    def on_warmup_completed(self, myo, timestamp, warmup_result):
        pass
        
    #CrazyFlie
    def _connected(self, link_uri):
        global connected
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        connected = True

    def _connection_failed(self, link_uri,msg):
        global connected
        print("Connection to %s failed: %s" % (link_uri, msg))
        connected = False

    def _connection_lost(self,link_uri,msg):
        global connected
        print("Connection to %s lost: %s" % (link_uri, msg))
        connected = False

    def _disconnected(self, link_uri):
        global connected
        print("Disconnected from %s" % link_uri)
        connected = False

def main():

    print("Connecting to Myo ... Use CTRL^C to exit.")
    try:
        hub = libmyo.Hub()
    except MemoryError:
        print("Myo Hub could not be created. Make sure Myo Connect is running.")
        return

    hub.set_locking_policy(libmyo.LockingPolicy.none)

    cflib.crtp.init_drivers(enable_debug_driver=False)
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    print("Crazyflies found:")
    for i in available:
        print(i[0])
    if len(available) > 0:
        le = hub.run(1000, Listener())
    else:
        print("No Crazyflies found, cannot run example")

    try:
        while hub.running:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nQuitting ...")
    finally:
        print("Shutting down hub...")
        if connected:
            le._cf.close_link()
        hub.shutdown()

if __name__ == '__main__':
    main()