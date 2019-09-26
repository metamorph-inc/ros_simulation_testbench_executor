#!/usr/bin/env python
"""
# Description:
#   Include in ROS *.launch file with required="true" and appropriate ROS Param values.
#
# References:
#   The UUV Simulator Authors (https://github.com/uuvsimulator/uuv_simulator)
"""

import time
from threading import Thread

import rospy


# Thread function
def get_sim_time(ros_rate, sim_time):
    while not rospy.is_shutdown():
        sim_time[0] = rospy.get_time()
        ros_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('set_simulation_timer')

    if rospy.is_shutdown():
        rospy.ROSException('ROS master is not running!')

    # ROS Params
    timeout = 0.0
    if rospy.has_param('~timeout'):
        timeout = float(rospy.get_param('~timeout'))
        if timeout <= 0:
            raise rospy.ROSException('Termination time must be a positive floating point value')
    sim_start_delay = rospy.get_param('~sim_start_delay', 60.0)
    sim_max_pause_duration = rospy.get_param('~sim_max_pause_duration', 60.0)

    print("Simulation start delay              = {:.2f} s".format(sim_start_delay))
    print("Simulation maximum pause duration   = {:.2f} s".format(sim_max_pause_duration))
    print("Starting simulation timer - Timeout = {:.2f} s".format(timeout))
    print("----------------------------------------------")

    # Logic
    init_time = time.time()
    sim_start_time = init_time + sim_start_delay  # TODO: Add 5% tolerance

    sim_time = [0.0]  # or None

    # Spin up thread to update sim_time
    t = Thread(target=get_sim_time, args=(rospy.Rate(100), sim_time))
    t.daemon = True  # We don't care about this thread finishing
    t.start()

    # Wait until expected simulation start time
    print("Waiting for {:.2f} s until expected simulation start time...".format(sim_start_delay))
    cur_time = time.time()
    while not rospy.is_shutdown():
        cur_time = time.time()
        if cur_time > sim_start_time:
            break
        time.sleep(0.1)
    print("Done waiting.")

    last_sim_time = sim_time[0]
    last_sim_update_time = time.time()
    while not rospy.is_shutdown():
        if last_sim_time != sim_time[0]:
            last_sim_update_time = time.time()
            last_sim_time = sim_time[0]
        if time.time() - last_sim_update_time > sim_max_pause_duration:
            print("Simulation pause > {:.2f} s detected... Killing simulation...".format(sim_max_pause_duration))
            break
        elif sim_time[0] > timeout:
            print("Simulation timeout... Killing simulation...")
            break
        else:
            time.sleep(0.02)
