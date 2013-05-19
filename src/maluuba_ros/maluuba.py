#!/usr/bin/env python

import roslib; roslib.load_manifest('maluuba_ros')

import maluuba_ros.srv
from maluuba_ros.msg import Entities, TimeRange

import sys

import rospy

from maluuba_napi import client

class Maluuba(object):
    """Wrapper for the Maluuba natural language understanding API.
    the nAPI supports two services, interpret and normalize. """
    def __init__(self, key):
        c = client.NAPIClient(key)
        super(Maluuba, self).__init__()
        self.client = c
    
        rospy.Service('maluuba/interpret', maluuba_ros.srv.Interpret, self.interpret)
        #rospy.Service('maluuba/normalize', maluuba_ros.srv.Normalize, self.normalize)

        self.processors = {"REMINDER_SET":self._parse_REMINDER_SET}

    def interpret(self, request):
        response = self.client.interpret(request.phrase)

        import ipdb; ipdb.set_trace()

        entities = response.entities

        if "contacts" in entities.keys():
            #translate contact-dict to list of names
            entities["contacts"] = [contact["name"] for contact in entities["contacts"]]
        if "originalDate" in entities.keys():
            pass
        if "replacementDate" in entities.keys():
            pass
        if "originalTime" in entities.keys():
            pass
        if "replacementTime" in entities.keys():
            pass
        if "replacementDate" in entities.keys():
            pass
        if "duration" in entities.keys():
            pass
        if "dateRange" in entities.keys():
            pass
        if "timeRange" in entities.keys():
            pass
        if "lengthOfTime" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass
        if "" in entities.keys():
            pass


        ents = Entities(**entities)

        return maluuba_ros.srv.InterpretResponse(
            ents, 
            str(response.category), 
            str(response.action))

    def default_parser(self, entities):
        return str(entities)

    def _parse_REMINDER_SET(self, entities):
        pass

    def normalize(self, request):
        response = self.client.normalize(request.phrase, request.type, request.timezone)

        return maluuba_ros.srv.NormalizeResponse(
            str(response.entities), 
            str(response.category), 
            str(response.action))


if __name__ == "__main__":
    rospy.init_node("maluuba_node")

    apikey = sys.argv[1]

    m = Maluuba(apikey)
    
    rospy.spin()