maluuba_ros
===========

Interface the Maluuba.com nAPI with ROS

===========
Installation:

$ git clone https://github.com/yol/maluuba_ros.git

$ rosmake maluuba_ros #You may need to first source ~/.bashrc

$ rosrun maluuba_ros maluuba.py "APIKEY"

$ #different terminal

$ rosservice call /maluuba/interpret "Remind me to get the eggs in 3 minutes"

It will return a giant ROS message with all maluuba entities.

TODO: find a better way to store entities in a ROS message.