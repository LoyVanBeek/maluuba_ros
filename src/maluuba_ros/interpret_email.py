#!/usr/bin/env python
"""ROS node that listens to maluuba_ros Interpretation-messages.
The Interpretation is processed by the node when it can handle the requested action"""

import roslib
roslib.load_manifest('maluuba_ros')
import os
import rospy
import yaml
from datetime import datetime

from maluuba_ros.msg import Interpretation
from maluuba_ros.srv import Normalize, NormalizeResponse

def make_ical(startdate, summary, enddate=None, filename='reminder.ics'):
    from icalendar import Calendar, Event
    cal = Calendar()
    cal.add('prodid', '-//My calendar product//mxm.dk//')
    cal.add('version', '2.0')

    import pytz
    event = Event()
    event.add('summary', summary)
    event.add('dtstart', startdate)#datetime(2005,4,4,8,0,0,tzinfo=pytz.utc))
    if enddate:
        event.add('dtend', enddate)#datetime(2005,4,4,10,0,0,tzinfo=pytz.utc))
    event.add('dtstamp', datetime.now())#datetime(2005,4,4,0,10,0,tzinfo=pytz.utc))
    event['uid'] = '20050115T101010/27346262376@mxm.dk'
    event.add('priority', 5)

    cal.add_component(event)

    f = open(filename, 'wb')
    f.write(cal.to_ical())
    f.close()


class MailInterpreter(object):
    """Interprets Maluuba EMAIL_SEND actions"""

    def __init__(self, configfile):
        self.normalizer = rospy.ServiceProxy('maluuba/normalize', Normalize)
        self.subscriber = rospy.Subscriber("/maluuba/interpretations", Interpretation, self.process_interpretation)
        self.mailadress = None
        self.password = None
        self.address_book = {}

        self.configure(configfile)

    def configure(self, configfile):
        config = list(yaml.load_all(configfile))
        
        account = config[0]
        address_book = config[1]

        self.address_book = dict([(name,value['email']) for name, value in address_book.iteritems()])

        self.mailadress = account["sender"]
        self.password = account["password"]

    def sendmail(self, subject, message, receivers, attachments={}):
        # a list as default arg can get bad, as they are mutable
        """Based off http://stackoverflow.com/questions/6270782/sending-email-with-python"""
        # Import smtplib for the actual sending function
        import smtplib

        # Import the email modules we'll need
        from email.MIMEMultipart import MIMEMultipart
        from email.MIMEBase import MIMEBase
        from email.MIMEText import MIMEText
        from email import Encoders
        # Create a text/plain message

        msg = MIMEMultipart()
        msg.attach( MIMEText(message) )
        msg['Subject'] = subject
        msg['From'] = self.mailadress
        msg['To'] = receivers[0]

        s = smtplib.SMTP('smtp.gmail.com', 587)
        s.starttls()
        s.login(self.mailadress, self.password)

        for f, (mimeapp, mimecontent) in attachments.iteritems():
            part = MIMEBase(mimeapp, mimecontent)#MIMEBase('application', "octet-stream")
            part.set_payload( open(f,"rb").read() )
            Encoders.encode_base64(part)
            part.add_header('Content-Disposition', 'attachment; filename="%s"' % os.path.basename(f))
            msg.attach(part)

        s.sendmail(self.mailadress, receivers, msg.as_string())
        rospy.loginfo("Sending {0}, {1}, {2}".format(subject, message, receivers))
        s.quit()

    def process_interpretation(self, msg):
        if msg.action == "EMAIL_SEND":
            subject = msg.entities.subject if msg.entities.subject else ""
            message = msg.entities.message if msg.entities.message else ""
            receivers = set([self.address_book.get(contact.name.lower(), self.mailadress) for contact in msg.entities.contacts])

            self.sendmail(subject, message, list(receivers))

        elif msg.action == "REMINDER_SET":
            receivers = [self.mailadress] #set([self.address_book.get(contact.name.lower(), self.mailadress) for contact in msg.entities.contacts])

            
            message = msg.entities.message if msg.entities.message else "reminder"

            # from dateutil import parser
            # normalized_time = self.normalizer(timestr, "TimeRange", "GMT+2")
            import ipdb; ipdb.set_trace()
            from dateutil import tz
            start = datetime.fromtimestamp(msg.entities.time)
            from_zone = tz.gettz('America/Montreal')
            to_zone = tz.gettz('Europe/Amsterdam')
            start = start.replace(tzinfo=from_zone)
            start = start.astimezone(to_zone)

            make_ical(start, message, filename='reminder.ics')
            rospy.loginfo("Made iCAL with {0}, {1}".format(start, message))
            #self.sendmail("Reminder", message, list(receivers), {'reminder.ics':("text", "calendar")})

if __name__ == "__main__":
    import sys
    rospy.init_node("interpret_email_send")
    
    try:
        interpreter = MailInterpreter(open(sys.argv[1]))
    except IOError as e:
        rospy.logerr(e)

    rospy.spin()
