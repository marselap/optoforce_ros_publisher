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

        self.touchId = rospy.Subscriber("/touch_id", String, self.touchIdCallback)

        self.start_flag = False
        self.stop_flag = True

        self.new_recording = False
        self.wrench = [0, 0, 0, 0, 0, 0]

        self.new_recording_raw = False
        self.wrench_raw = [0, 0, 0, 0, 0, 0]

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


def main(args):
    opto2file = optoforce_to_file()
    rospy.init_node('save_optoforce', anonymous=True)

    wrench_array = []
    wrench_array_raw = []

    try:
        while True:
            time.sleep(0.003)
            if opto2file.start_flag:
                if opto2file.stop_flag:
                    outFolder = "/home/franka/datasets/force_all/1004/optoforce_1004/"
                    pathOut = outFolder + "optoforce_" + opto2file.touchId + ".txt"
                    with open(pathOut, 'wb') as fp:
                        pickle.dump(wrench_array, fp)
                    pathOut_raw = outFolder + "optoforce_raw_" + opto2file.touchId + ".txt"
                    with open(pathOut_raw, 'wb') as fp:
                        pickle.dump(wrench_array_raw, fp)
                    opto2file.start_flag = False
                    wrench_array = []
                    wrench_array_raw = []
                else:
                    if opto2file.new_recording:
                        a = [i for i in opto2file.wrench]
                        wrench_array.append(a)
                        opto2file.new_recording = False
                    if opto2file.new_recording_raw:
                        a = [i for i in opto2file.wrench_raw]
                        wrench_array_raw.append(a)
                        opto2file.new_recording_raw = False

    except KeyboardInterrupt:
        print("Shutting down")
        rospy.signal_shutdown("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
