#!/usr/bin/env python3
import rospy
from dsor_paths.srv import *

# Services used to start/stop and switch between different path following algorithms
# and services to create paths for the path following algorithms to follow
path_services_required = [
        # Service names used to start and stop a path following mission
        '/PFStart', 
        '/PFStop', 

        # Service names to clear the path
        '/ResetPath', 

        # Service names to set the desired speed profile for the vehicle and/or virtual target
        '/SetConstVdRabbit', 
        '/SetConstVdVehicle', 
        
        # Services names to add lines, arcs, circles or bernoulli lemniscates to the current path
        '/SpawnArc2DPath', 
        '/SpawnBernoulliPath', 
        '/SpawnCircle2DPath', 
        '/SpawnLinePath']

# Function to reset the previous path in memory
def reset_path(vehicle_name: str):

    try:
        # Call the service to clear the path
        reset_srv = rospy.ServiceProxy(vehicle_name + '/ResetPath', ResetPath)
        res = reset_srv(True)
        return res.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to set the desired speed for each vehicle
def spawn_const_speed(speed_value, vehicle_name: str):
    
    try:
        # Call the service to set a constant speed for the vehicle
        speed = rospy.ServiceProxy(vehicle_name + '/SetConstVdVehicle', SetConstSpeed)  
        resp = speed(speed_value, speed_value)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a line for each vehicle
def spawn_line(start, end, ref, vehicle_name: str):
    
    try:

        # Call the service to create a line
        line = rospy.ServiceProxy(vehicle_name + '/SpawnLinePath', SpawnLine)  
        resp = line(start, end, ref)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a circle for each vehicle
def spawn_circle(radius, center_x, center_y, z, vehicle_name: str):
    
    try:
        # Call the service to create a circle
        circle = rospy.ServiceProxy(vehicle_name + '/SpawnCircle2DPath', SpawnCircle2D)  
        resp = circle(radius, center_x, center_y, z)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a bernoulli for each vehicle
def spawn_bernoulli(radius, center_x, center_y, z, vehicle_name: str):
    
    try:
        # Call the service to create a bernoulli lemniscate
        bernoulli = rospy.ServiceProxy(vehicle_name + '/SpawnBernoulliPath', SpawnBernoulli)  
        resp = bernoulli(radius, center_x, center_y, z)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a circle path for each vehicle
def spawn_arc(start, end, center, direction, z, vehicle_name: str):
    
    try:
        # Call the service to create an arc
        arc = rospy.ServiceProxy(vehicle_name + '/SpawnArc2DPath', SpawnArc2D)  
        resp = arc(start, end, center, direction, z)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to create a lawnmower path
def spawn_lawn_mower(z: float, vehicle_name: str):
   
    x_ref: float = 491873.487
    y_ref: float = 4290833.305

    # Spawn a line
    spawn_line([y_ref+39.1, x_ref+40.41, z], [y_ref-27.16, x_ref+39.32, z], [0.0, 0.0, 0.0], vehicle_name) 
    
    # Spawn an arc
    spawn_arc([y_ref-27.16, x_ref+39.32], [y_ref-26.72, x_ref+12.81], [y_ref-26.94, x_ref+26.06], -1, z, vehicle_name)
    
    # Spawn a line
    spawn_line([y_ref-26.72, x_ref+12.81, z], [y_ref+26.28, x_ref+13.69, z], [0.0, 0.0, 0.0], vehicle_name)

    # Spawn an arc
    spawn_arc([y_ref+26.28, x_ref+13.69], [y_ref+26.72, x_ref-12.81], [y_ref+26.5, x_ref+0.44], 1, z, vehicle_name)

    # Spawn a line
    spawn_line([y_ref+26.72, x_ref-12.81, z], [y_ref-26.28, x_ref-13.69, z], [0.0, 0.0, 0.0], vehicle_name)
    
    # Spawn an arc
    spawn_arc([y_ref-26.28, x_ref-13.69], [y_ref-25.84, x_ref-40.19], [y_ref-26.06, x_ref-26.94], -1, z, vehicle_name)
    
    # Spawn a line
    spawn_line([y_ref-25.84, x_ref-40.19, z], [y_ref+40.41, x_ref-39.10, z], [0.0, 0.0, 0.0], vehicle_name)