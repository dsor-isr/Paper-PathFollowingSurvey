#!/usr/bin/env python3
import rospy
import time

# Import the path following package services
import path_following.srv as medusa_pf

# Setup the controllers list and corresponding services to invoke them
controllers_medusa_dic = {'aguiar': '/PFSetAguiar', 
                          'lapierre': '/PFSetLapierre', 
                          'samson': '/PFSetSamson',
                          'fossen': '/PFSetFossen',
                          'pramod': '/PFSetPramod',
                          'romulo': '/PFSetRomulo',
                          'brevik': '/PFSetBrevik',
                          'relative_heading': 'PFSetRelativeHeading'}

# Method to set the desired controller for a medusa
def spawn_medusa_controller(control_algorithm: str, vehicle_name: str):
    
    try:

        # Call the service to switch to the desired controller
        control = rospy.ServiceProxy(vehicle_name + controllers_medusa_dic[control_algorithm], medusa_pf.SetPF)
        resp = control()
        time.sleep(3)

        # Call the service to start a path following mission
        start = rospy.ServiceProxy(vehicle_name + '/PFStart', medusa_pf.StartPF)
        resp2 = start()
        return resp2.success

    # Check if there was any exception when invoking the services
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e) 