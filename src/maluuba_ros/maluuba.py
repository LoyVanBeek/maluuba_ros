#!/usr/bin/env python

import roslib; roslib.load_manifest('maluuba_ros')

import maluuba_ros.srv

import rospy
import std_msgs

from maluuba_napi import client
APIKEY = "n393BfPBw473QiFPzHBAfxxkKZAbc8QN"

class Maluuba(object):
    """docstring for Interpreter"""
    def __init__(self, key):
        c = client.NAPIClient(key)
        super(Maluuba, self).__init__()
        self.client = c
    
        rospy.Service('maluuba/interpret', maluuba_ros.srv.Interpret, self.interpret)


    def interpret(self, request):
        response = self.client.interpret(request.sentence)

        return maluuba_ros.srv.InterpretResponse(
            str(response.entities), 
            str(response.category), 
            str(response.action))

    def normalize(self, request):
        response = self.client.normalize(request.data)

        
        

if __name__ == "__main__":
    rospy.init_node("maluuba_node")

    m = Maluuba(APIKEY)
    
    rospy.spin()