#!/usr/bin/env python
from __future__ import print_function
import sys
import numpy as np
import rospy
from std_msgs.msg import String, Bool
import time
from geometry_msgs.msg import WrenchStamped
import pickle


class optoforce_to_file:

    def __init__(self):

        self.opto_sub = rospy.Subscriber("/optoforce_node/OptoForceWrench", WrenchStamped, self.callbackOpto, queue_size = 1)
        self.opto_sub_raw = rospy.Subscriber("/optoforce_node/OptoForceWrench_raw", WrenchStamped, self.callbackOpto_raw, queue_size = 1)

        self.start_recording = rospy.Subscriber("/record_start", Bool, self.startCallback)
        self.stop_recording = rospy.Subscriber("/record_stop", Bool, self.stopCallback)

        self.stop_recording = rospy.Subscriber("/calc_mean", Bool, self.calcMeanCallback)

        self.touchId = rospy.Subscriber("/touch_id", String, self.touchIdCallback)

        self.start_flag = False
        self.stop_flag = True

        self.calc_mean_flag = True


        self.new_recording = False
        self.wrench = [0, 0, 0, 0, 0, 0]

        self.new_recording_raw = False
        self.wrench_raw = [0, 0, 0, 0, 0, 0]

        self.dc_offset = 0.0

        self.dc_array = []
        self.meas_array = []

    def callbackOpto(self, data):
        self.wrench[0] = data.wrench.force.x
        self.wrench[1] = data.wrench.force.y
        self.wrench[2] = data.wrench.force.z
        self.wrench[3] = data.wrench.torque.x
        self.wrench[4] = data.wrench.torque.y
        self.wrench[5] = data.wrench.torque.z
        self.new_recording = True

    def callbackOpto_raw(self, data):
        self.wrench_raw[0] = data.wrench.force.x
        self.wrench_raw[1] = data.wrench.force.y
        self.wrench_raw[2] = data.wrench.force.z
        self.wrench_raw[3] = data.wrench.torque.x
        self.wrench_raw[4] = data.wrench.torque.y
        self.wrench_raw[5] = data.wrench.torque.z
        self.new_recording_raw = True

    def touchIdCallback(self, data):
        self.touchId = data.data

    def startCallback(self, data):
        self.start_flag = True
        self.stop_flag = False

    def stopCallback(self, data):
        self.stop_flag = True

    def calcMeanCallback(self, data):
        self.calc_mean_flag = data.data
        if self.calc_mean_flag == True:
            self.dc_array = []
        else:
            self.dc_offset = np.mean(self.dc_array)

def main(args):
    opto2file = optoforce_to_file()
    rospy.init_node('save_optoforce', anonymous=True)

    wrench_array = []
    wrench_array_raw = []
    z_array = []

    try:
        while True:
            time.sleep(0.003)

            if opto2file.start_flag:
                if opto2file.stop_flag:
                    outFolder = "/home/franka/datasets/force_all/1004/optoforce_1004/"
                    pathOut_raw = outFolder + "z_force_" + opto2file.touchId + ".txt"
                    z_touch = np.mean(z_array) - opto2file.dc_offset
                    print(z_touch)
                    with open(pathOut_raw, 'wb') as fp:
                        pickle.dump([z_touch], fp)
                    opto2file.start_flag = False
                    wrench_array = []
                    wrench_array_raw = []
                    z_array = []
                else:
                    if opto2file.new_recording:
                        a = [i for i in opto2file.wrench]
                        z = opto2file.wrench[2]
                        z_array.append(z)
                        wrench_array.append(a)
                        opto2file.new_recording = False
                    if opto2file.new_recording_raw:
                        a = [i for i in opto2file.wrench_raw]
                        wrench_array_raw.append(a)
                        opto2file.new_recording_raw = False

            if opto2file.calc_mean_flag:
                if opto2file.new_recording:
                    opto2file.dc_array.append(opto2file.wrench[2])
                    opto2file.new_recording = False

    except KeyboardInterrupt:
        print("Shutting down")
        rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
