#!/usr/bin/env python

import roslib; roslib.load_manifest('maluuba_ros')

import maluuba_ros.srv

import sys

import rospy
import std_msgs

from maluuba_napi import client

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

    apikey = sys.argv[1]

    m = Maluuba(apikey)
    
    rospy.spin()