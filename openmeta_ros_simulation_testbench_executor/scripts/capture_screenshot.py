#!/usr/bin/env python
"""
# Description:
#
# References:
#
"""

import rospy
import datetime
import subprocess


if __name__ == '__main__':
    rospy.init_node('capture_screenshot')
    rate = rospy.Rate(10)

    # Get ROS Params
    prefix = rospy.get_param('~prefix', 'openmeta_tb_sim_executor')

    if rospy.has_param('~capture_times') and rospy.get_param('~capture_times') != "":
        times = [float(x) for x in rospy.get_param('~capture_times').split(' ')]
        times.sort()  # e.g. [1.4, 5, 45]
        print("Capturing screenshot at: {} s".format(' '.join(str(t) for t in times)))

        for capture_time in times:
            while not rospy.is_shutdown():
                if rospy.get_time() < capture_time:
                    rate.sleep()
                else:
                    date = str(datetime.datetime.now()).replace(' ', 'T')
                    ss_filename = prefix+'_'+date+'.png'
                    cmd = "--file="+ss_filename
                    proc = subprocess.call(["gnome-screenshot", cmd])
                    print("Captured screenshot {}.".format(ss_filename))
                    break
