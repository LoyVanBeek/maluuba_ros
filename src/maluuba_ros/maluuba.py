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

        allFields = [   "album","rating","playlist","song","artist","station",
                        "genre","originalTitle","replacementTitle","originalDate",
                        "replacementDate","originalLocation","replacementLocation",
                        "originalTime","replacementTime","contactName","deleteContactName",
                        "duration","dateRange","timeRange","repeatDaysLength","lengthOfTime",
                        "REPEATDAYS","meetingTitle","location","date","contacts","message",
                        "subject","keyword","time","leaveLocation","destination","origin",
                        "transitType","route","searchTerm","numPeople","appName",
                        "contactField","contactFieldValue","mpaaRating","actor",
                        "theatre","title","numTickets","departureTime","departureDay",
                        "returnTime","returnDay","departing","carrier","sortOrder",
                        "noReturn","child","adult","senior","price","luxury"]

        intFields = ["rating", "numPeople", "numTickets", "child", "adult"]
        timeFields = ["originalDate", "replacementDate", "originalTime", "replacementTime", "date", "time", "departureTime", "returnTime"]
        floatFields = ["price"]
        durationFields = ["duration"]
        TimeRangeFields = ["dateRange", "timeRange"]
        stringArrayFields = ["REPEATDAYS"]
        specialFields = ["contacts"]

        otherFields = list( set(allFields) -
                            set(intFields) - 
                            set(timeFields) - 
                            set(floatFields) - 
                            set(durationFields) - 
                            set(TimeRangeFields) - 
                            set(stringArrayFields)- 
                            set(specialFields))

        entities = response.entities

        for field in [field for field in intFields if field in entities.keys()]:
            entities[field] = int(entities[field])
        
        for field in [field for field in floatFields if field in entities.keys()]:
            if field in entities.keys():
                entities[field] = float(entities[field])

        #{u'timeRange': [{u'start': datetime.time(8, 0), u'end': datetime.time(10, 0)}], u'title': [u'meeting']}
        for field in [field for field in TimeRangeFields if field in entities.keys()]:
            if field in entities.keys():
                _range  = entities[field][0]
                start   = str(_range["start"])
                end     = str(_range["end"])
                entities[field] = TimeRange(start, end)

        for field in [field for field in durationFields if field in entities.keys()]:
            if field in entities.keys():
                entities[field] = str(entities[field])

        #Time as returned by Maluuba cannot be put in a ROS Time message.
        for field in [field for field in timeFields if field in entities.keys()]:
            if field in entities.keys():
                entities[field] = str(entities[field])
        
        if "contacts" in entities.keys():
            #translate contact-dict to list of names
            entities["contacts"] = [contact["name"] for contact in entities["contacts"]]
        
        for field in [field for field in stringArrayFields if field in entities.keys()]:
            entities[field] = [str(item) for item in entities[field]]

        for field in [field for field in otherFields if field in entities.keys()]:
            entities[field] = [str(value) for value in entities[field]][0]

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