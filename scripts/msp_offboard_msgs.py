#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Request and response definitions for services

class NavigateRequest:
    """Request for navigate service"""
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0, speed=0.0, frame_id="map", auto_arm=False):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.speed = speed
        self.frame_id = frame_id
        self.auto_arm = auto_arm

class NavigateResponse:
    """Response for navigate service"""
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

class SetVelocityRequest:
    """Request for set_velocity service"""
    def __init__(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, frame_id="body"):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw_rate = yaw_rate
        self.frame_id = frame_id

class SetVelocityResponse:
    """Response for set_velocity service"""
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

class SetPositionRequest:
    """Request for set_position service"""
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0, frame_id="map"):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.frame_id = frame_id

class SetPositionResponse:
    """Response for set_position service"""
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

# Service handlers

def navigate_service_handler(navigate_method):
    """Creates a handler for navigate service"""
    def handle(req):
        # Parse parameters from request string
        params = req.request.split()
        
        # Create request object
        navigate_req = NavigateRequest()
        
        # Parse parameters
        i = 0
        while i < len(params):
            if params[i] == "x":
                navigate_req.x = float(params[i+1])
                i += 2
            elif params[i] == "y":
                navigate_req.y = float(params[i+1])
                i += 2
            elif params[i] == "z":
                navigate_req.z = float(params[i+1])
                i += 2
            elif params[i] == "yaw":
                navigate_req.yaw = float(params[i+1])
                i += 2
            elif params[i] == "speed":
                navigate_req.speed = float(params[i+1])
                i += 2
            elif params[i] == "frame_id":
                navigate_req.frame_id = params[i+1]
                i += 2
            elif params[i] == "auto_arm":
                navigate_req.auto_arm = params[i+1].lower() == "true"
                i += 2
            else:
                i += 1
        
        # Call navigate method
        response = navigate_method(navigate_req)
        
        # Create service response
        trigger_response = TriggerResponse()
        trigger_response.success = response.success
        trigger_response.message = response.message
        
        return trigger_response
    
    return handle

def set_velocity_service_handler(set_velocity_method):
    """Creates a handler for set_velocity service"""
    def handle(req):
        # Parse parameters from request string
        params = req.request.split()
        
        # Create request object
        velocity_req = SetVelocityRequest()
        
        # Parse parameters
        i = 0
        while i < len(params):
            if params[i] == "vx":
                velocity_req.vx = float(params[i+1])
                i += 2
            elif params[i] == "vy":
                velocity_req.vy = float(params[i+1])
                i += 2
            elif params[i] == "vz":
                velocity_req.vz = float(params[i+1])
                i += 2
            elif params[i] == "yaw_rate":
                velocity_req.yaw_rate = float(params[i+1])
                i += 2
            elif params[i] == "frame_id":
                velocity_req.frame_id = params[i+1]
                i += 2
            else:
                i += 1
        
        # Call set_velocity method
        response = set_velocity_method(velocity_req)
        
        # Create service response
        trigger_response = TriggerResponse()
        trigger_response.success = response.success
        trigger_response.message = response.message
        
        return trigger_response
    
    return handle

def set_position_service_handler(set_position_method):
    """Creates a handler for set_position service"""
    def handle(req):
        # Parse parameters from request string
        params = req.request.split()
        
        # Create request object
        position_req = SetPositionRequest()
        
        # Parse parameters
        i = 0
        while i < len(params):
            if params[i] == "x":
                position_req.x = float(params[i+1])
                i += 2
            elif params[i] == "y":
                position_req.y = float(params[i+1])
                i += 2
            elif params[i] == "z":
                position_req.z = float(params[i+1])
                i += 2
            elif params[i] == "yaw":
                position_req.yaw = float(params[i+1])
                i += 2
            elif params[i] == "frame_id":
                position_req.frame_id = params[i+1]
                i += 2
            else:
                i += 1
        
        # Call set_position method
        response = set_position_method(position_req)
        
        # Create service response
        trigger_response = TriggerResponse()
        trigger_response.success = response.success
        trigger_response.message = response.message
        
        return trigger_response
    
    return handle 