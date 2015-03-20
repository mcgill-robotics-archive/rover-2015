#!/usr/bin/python

import xlsxwriter
import rospy
import time
import datetime

from rosgraph_msgs.msg import Log

DEBUG = 1
INFO = 2
WARNING = 4
ERROR = 8
FATAL = 16

"""
-----------------------------------------
| TimeStamp | Severity | Node | Message |
-----------------------------------------=
"""


class LogRosout():
    def __init__(self, workbook, worksheet):

        self.workbook = workbook
        self.page = worksheet
        self.extended_date = None
        self.yellow_fill = None
        self.red_fill = None
        self.black_fill = None
        self.row_index = 0

        self.format_page()

        self.init_ros()
        self.sub = None

    def format_page(self):
        self.extended_date = self.workbook.add_format({'num_format': 'dd/mm/yyyy hh:mm:ss'})

        self.yellow_fill = self.workbook.add_format()
        self.yellow_fill.set_pattern(1)
        self.yellow_fill.set_bg_color('yellow')

        self.red_fill = self.workbook.add_format()
        self.red_fill.set_pattern(1)
        self.red_fill.set_bg_color('red')

        self.black_fill = self.workbook.add_format()
        self.black_fill.set_pattern(1)
        self.black_fill.set_bg_color('black')
        self.black_fill.set_font_color('white')

        self.page.set_column(0, 0, 20)

    def log_callback(self, message):
        self.page.write_datetime(self.row_index,
                                 0,
                                 datetime.datetime.fromtimestamp(message.header.stamp.secs),
                                 self.extended_date)

        self.page.write_string(self.row_index, 2, message.name)
        self.page.write_string(self.row_index, 3, message.msg)

        if message.level is INFO:
            self.page.write_string(self.row_index, 1, "INFO")
            # Leave row white
            self.page.set_row(self.row_index, 16)
        elif message.level is WARNING:
            self.page.write_string(self.row_index, 1, "WARNING")
            # Highlight yellow
            self.page.set_row(self.row_index, 16, self.yellow_fill)
        elif message.level is ERROR:
            self.page.write_string(self.row_index, 1, "ERROR")
            # Highlight red
            self.page.set_row(self.row_index, 16, self.red_fill)
        elif message.level is FATAL:
            self.page.write_string(self.row_index, 1, "FATAL")
            # Highlight Black
            self.page.set_row(self.row_index, 16, self.black_fill)

        self.row_index += 1

        print message

    """
    def log_info(self, message):


    def log_warm(self, message):


    def log_error(self, message):


    def log_debug(self, message):

    """
    def init_ros(self):
        rospy.init_node("logger", anonymous=True)
        self.sub = rospy.Subscriber("/rosout_agg", Log, self.log_callback)


if __name__ == "__main__":
    workbook = xlsxwriter.Workbook('demo.xlsx')
    worksheet = workbook.add_worksheet()
    logger = LogRosout(workbook, worksheet)
    rospy.spin()
    workbook.close()
