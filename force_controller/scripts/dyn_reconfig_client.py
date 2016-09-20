#!/usr/bin/env python

PACKAGE = 'force_controller'
import rospy
import pdb
import dynamic_reconfigure.client


class DynaClient():
    def __init__(self):   
        rospy.init_node("force_error_constants_dynamic_client") # Name this client
        rospy.loginfo("Connecting to force_controller_constants dyn_rec_client.")
        #pdb.set_trace() # debug

        # Client runs once at 10Hz and calls upate_configuration
        client = dynamic_reconfigure.client.Client("force_error_constants", timeout=30, config_callback=self.callback)

        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            #client.update_configuration({"kf0":kf0, "kf1":kf1, "kf2":kf2, "km0":km0, "km1":km1, "km2":km2})
            client.update_configuration()
        r.sleep()

    # This callback prints the configuration returned by the server. Optional callback. 
    def callback(config):
        rospy.loginfo("Your force error parameters kf[0-2] and km[0-2] are now set to: " + str(config['kf0'])  + str(config['kf1'])  + str(config['kf2']) + str(config['km0']) + str(config['km1'])  + str(config['km2']) )

if __name__ == '__main__':
    DynaClient()
