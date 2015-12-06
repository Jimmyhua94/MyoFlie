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
from cfclient.utils.logconfigreader import LogConfig  # noqa
from cfclient.ui.widgets.ai import AttitudeIndicator
from cflib.crazyflie import Crazyflie  # noqa

logging.basicConfig(level=logging.ERROR)

channel = "70"
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
        
        global channel

        link_uri = "radio://0/" + channel + "/2M"
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        
        self.thrust_h = 0
        self.holder = 0
        self.cal = 0
        
        self.quat = libmyo.Quaternion(0,0,0,1)

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
        elif pose == libmyo.Pose.fist:
            self.cal = 0
            print("finger")
        elif pose == libmyo.Pose.wave_out:
            self._cf.commander.send_setpoint(0, 0, 0, 0)	#Clear packets
            time.sleep(0.1)
            self._cf.close_link()
            print("fist")
        self.pose = pose

    def on_orientation_data(self, myo, timestamp, quat):
        thrust = float(math.asin(max(-1.0, min(1.0, 2.0 * (quat.w * quat.y - quat.z * quat.x)))))
        if self.cal == 0:
            self.quat = libmyo.Quaternion(quat.x,quat.y,quat.z,quat.w).conjugate()
            self.cal = 1
        if self.holder == 0:
            self.thrust_h = int((thrust + math.pi/2.0)/math.pi * 120000)
            self.holder = 1
        tempquat = quat * self.quat
        roll_w = int(math.atan2(2.0 * (quat.w * quat.x + quat.y * quat.z),1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)) * 100 / math.pi)
        pitch_w = int(math.atan2(2.0 * (tempquat.w * tempquat.z + tempquat.x * tempquat.y),1.0 - 2.0 * (tempquat.y * tempquat.y + tempquat.z * tempquat.z)) * 100 / math.pi)
        thrust_t = abs(int((thrust + math.pi/2.0)/math.pi * 120000) - self.thrust_h)
        if thrust_t < 60000:
            thrust_w = thrust_t
        else:
            thrust_w = 60000

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
        
        self._lg_acc = LogConfig(name="Acceleration", period_in_ms=10)
        self._lg_acc.add_variable("acc.zw","float")
        self._lg_acc.add_variable("altHold.target","float")
        self._lg_acc.add_variable("stabilizer.thrust","float")
        
        try:
            self._cf.log.add_config(self._lg_acc)
            # This callback will receive the data
            self._lg_acc.data_received_cb.add_callback(self._acc_log_data)
            # This callback will be called on errors
            self._lg_acc.error_cb.add_callback(self._acc_log_error)
            # Start the logging
            self._lg_acc.start()
        except KeyError as e:
            print("Could not start log configuration,"
                  "{} not found in TOC".format(str(e)))
        except AttributeError:
            print("Could not add Acceleration log config, bad configuration.")

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

    def _acc_log_error(self, logconf, msg):
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _acc_log_data(self, timestamp, data, logconf):
        """if(data["acc.zw"] < -0.98):
            self._cf.commander.send_setpoint(0,0,0,45000)
            print("[%d][%s]: %s" % (timestamp, logconf.name, data))
        elif(data["acc.zw"] > 0.5):
            self._cf.commander.send_setpoint(0,0,0,0)
            print("[%d][%s]: %s" % (timestamp, logconf.name, data))"""

def main():
    global channel

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
        if i[0] == "radio://0/"+ channel +"/2M":
            le = hub.run(1000, Listener())
            break
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