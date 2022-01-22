#!/usr/bin/env python3
import time
import rospy
from controllers import *
from paths import *

# Services used to start/stop and switch between different path following algorithms
# and services to create paths for the path following algorithms to follow
services_required = [
        '/PFStart', 
        '/PFStop', 
        '/PFUpdateGains', 
        '/ResetPath', 
        '/SetConstVdRabbit', 
        '/SetConstVdVehicle', 
        '/SetMode', 
        '/SpawnArc2DPath', 
        '/SpawnBernoulliPath', 
        '/SpawnCircle2DPath', 
        '/SpawnLinePath']

# The entry point of the program
if __name__ == "__main__":
    
    # Get the name of the vehicle being used
    vehicle_name = rospy.get_param('vehicle_name', 'myellow')
    
    # Wait for the required services to start
    for service in services_required:
        rospy.wait_for_service(vehicle_name + service)

    # Reset the current path in memory
    reset_path(vehicle_name)

    # Spawn a lawnmower path at 30 m of altitude
    spawn_lawn_mower(-30, vehicle_name)
    #spawn_bernoulli(30, 4290841.0, 491926.0,-30, vehicle_name)

    # Ask for the desired speed for the vehicle
    spawn_const_speed(0.5, vehicle_name)

    # Wait for the required controller services to start
    for controller in controllers_medusa_required:
        rospy.wait_for_service(vehicle_name + '/' + controller)

    # Ask for the desired controller for this experiment
    spawn_medusa_controller('Marcelo', vehicle_name)  