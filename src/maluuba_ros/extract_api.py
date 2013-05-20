#!/usr/bin/env python

from BeautifulSoup import BeautifulSoup
import requests

import roslib
roslib.load_manifest('maluuba_ros')
import rospy
import maluuba_ros.srv
from maluuba_ros.msg import TimeRange

service = rospy.ServiceProxy("/maluuba/interpret", maluuba_ros.srv.Interpret)
    
def parse_table_bs(tableSoup):
    rows = tableSoup.findChildren("tr")

    headers = [head.text for head in rows[0].findChildren("th")]

    contents = rows[1:]

    for row in contents:
        yield dict([(headers[i], col) for i, col in enumerate(row.findAll("td"))])

base_url = """http://dev.maluuba.com"""
categories_url = "/categories"

text = requests.get(base_url+categories_url).text

soup = BeautifulSoup(text)

def get_category_pages(pageSoup):
    for table in pageSoup.findAll('table'):
        table = table.findChildren('tbody')[0]
        for row in parse_table_bs(table):
            for k,v in row.iteritems():
                links = v.findChildren("a")
                if links:
                    url = links[0]['href']
                    name = links[0].text
                    yield (name, base_url+url)

categories = dict(row for row in get_category_pages(soup))
#print categories

def extract_examples(catPageSoup):
    exampleTable = catPageSoup.findAll("table")[0]
    exampleTable = exampleTable.findChildren('tbody')[0]

    for row in parse_table_bs(exampleTable):
        text = row['Example'].text
        text = text.replace(u'\u201c', '')
        text = text.replace(u'\u201d', '')
        text = text.replace(u'&quot;', '') 
        examples = text.split(",")
        for ex in examples: yield ex

fails = []

for category, cat_url in categories.iteritems():
    text = requests.get(cat_url).text
    catSoup = BeautifulSoup(text)
    for example in extract_examples(catSoup):
        try:
            response = service(example)
            print example
            print response.action, " - " ,response.category
            fields = response.entities.__slots__
            for key in fields:
                value = response.entities.__getattribute__(key)
                if value:
                    #import ipdb;ipdb.set_trace()
                    if isinstance(value, TimeRange):
                        if value.start:
                            print key,value
                    else:
                        print key, value
            print "-"*10
        except Exception, e:
            print example
            print e
            print "*"*10
            fails += [example]

print fails





