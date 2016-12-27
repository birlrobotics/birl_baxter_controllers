#!/usr/bin/env python

PACKAGE = 'force_controller'
import rospy
import ipdb
import dynamic_reconfigure.client


class DynaClient():
    def __init__(self):   
        rospy.init_node("force_controller_reconfigure") # Name this client
        rospy.loginfo("Connecting to force_controller_constants dyn_rec_client.")
        #pdb.set_trace() # debug

        # Client runs once at 10Hz and calls upate_configuration
        client = dynamic_reconfigure.client.Client("force_error_constants", timeout=5, config_callback=self.callback)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #client.update_configuration({"k_fp0":k_fp0, "k_fp1":k_fp1, "k_fp2":k_fp2, "k_mp0":k_mp0, "k_mp1":k_mp1, "k_mp2":k_mp2})
            client.update_configuration()
        r.sleep()

    # This callback prints the configuration returned by the server. Optional callback. 
    def callback(config):
        rospy.loginfo("Your force error parameters k_fp[0-2] and k_mp[0-2] are now set to: " 
                      + str(config['k_fp0'])  + str(config['k_fp1'])  + str(config['k_fp2']) 
                      + str(config['k_mp0']) + str(config['k_mp1'])  + str(config['k_mp2']) )

if __name__ == '__main__':
    DynaClient()
