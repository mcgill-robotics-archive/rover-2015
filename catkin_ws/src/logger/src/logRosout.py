#!/usr/bin/python

import xlsxwriter, rospy, time

from rosgraph_msgs import Log

class LogRosout():
    def __init__(self, page):
        #page should be a xlsxwriter.Worksheet() object
        self.page = page
        self.sub = None

    def log_callback(self, message):

    def log_info():

    def log_warm():

    def log_error():

    def log_debug():

    def init_ros(self):
        rospy.init_node("logger", anonymous=True)
        self.sub = rospy.Subscriber("/rosout_agg", Log, self.log_callback)


