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
def spawn_const_speed(speed_value: float, vehicle_name: str):
    
    try:
        # Call the service to set a constant speed for the vehicle
        speed = rospy.ServiceProxy(vehicle_name + '/SetConstVdVehicle', SetConstSpeed)  
        resp = speed(speed_value, speed_value)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a line for each vehicle
def spawn_line(start, end, ref, vehicle_name: str, speed: float):
    
    try:
        # Call the service to create a line
        line = rospy.ServiceProxy(vehicle_name + '/SpawnLinePath', SpawnLine)  
        resp = line(start, end, ref)

        # Set the speed for this section
        spawn_const_speed(speed, vehicle_name)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a circle for each vehicle
def spawn_circle(radius, center_x, center_y, z, vehicle_name: str, speed: float):
    
    try:
        # Call the service to create a circle
        circle = rospy.ServiceProxy(vehicle_name + '/SpawnCircle2DPath', SpawnCircle2D)  
        resp = circle(radius, center_x, center_y, z)

        # Set the speed for this section
        spawn_const_speed(speed, vehicle_name)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a bernoulli for each vehicle
def spawn_bernoulli(radius: float, center_x: float, center_y: float, z: float, vehicle_name: str, speed: float):
    
    try:
        # Call the service to create a bernoulli lemniscate
        bernoulli = rospy.ServiceProxy(vehicle_name + '/SpawnBernoulliPath', SpawnBernoulli)  
        resp = bernoulli(radius, center_x, center_y, z)

        # Set the speed for this section
        spawn_const_speed(speed, vehicle_name)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to spawn a circle path for each vehicle
def spawn_arc(start, end, center, direction, z, vehicle_name: str, speed: float):
    
    try:
        # Call the service to create an arc
        arc = rospy.ServiceProxy(vehicle_name + '/SpawnArc2DPath', SpawnArc2D)  
        resp = arc(start, end, center, direction, z)

        # Set the speed for this section
        spawn_const_speed(speed, vehicle_name)

        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Function to create a lawnmower path
def spawn_lawn_mower(center_x: float, center_y: float, z: float, vehicle_name: str, speed: float):

    # Spawn a line
    spawn_line([center_x+39.1, center_y+40.41, z], [center_x-27.16, center_y+39.32, z], [0.0, 0.0, 0.0], vehicle_name, speed) 
    
    # Spawn an arc
    spawn_arc([center_x-27.16, center_y+39.32], [center_x-26.72, center_y+12.81], [center_x-26.94, center_y+26.06], -1, z, vehicle_name, speed)
    
    # Spawn a line
    spawn_line([center_x-26.72, center_y+12.81, z], [center_x+26.28, center_y+13.69, z], [0.0, 0.0, 0.0], vehicle_name, speed)

    # Spawn an arc
    spawn_arc([center_x+26.28, center_y+13.69], [center_x+26.72, center_y-12.81], [center_x+26.5, center_y+0.44], 1, z, vehicle_name, speed)

    # Spawn a line
    spawn_line([center_x+26.72, center_y-12.81, z], [center_x-26.28, center_y-13.69, z], [0.0, 0.0, 0.0], vehicle_name, speed)
    
    # Spawn an arc
    spawn_arc([center_x-26.28, center_y-13.69], [center_x-25.84, center_y-40.19], [center_x-26.06, center_y-26.94], -1, z, vehicle_name, speed)
    
    # Spawn a line
    spawn_line([center_x-25.84, center_y-40.19, z], [center_x+40.41, center_y-39.10, z], [0.0, 0.0, 0.0], vehicle_name, speed)