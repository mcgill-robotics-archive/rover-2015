#!/usr/bin/python

import xlsxwriter
import rospy
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
        self.row_index = 2

        self.format_page()

        self.sub = None
        self.init_ros()

    def format_page(self):
        big_font = self.workbook.add_format({'font_size':'30'})
        self.page.write_string(0, 0, "Rosout Log", big_font)
        self.page.set_row(0, 35)

        self.page.write_string(1, 0, "Date")
        self.page.write_string(1, 1, "Severity")
        self.page.write_string(1, 2, "Node")
        self.page.write_string(1, 3, "Message")
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
        self.page.set_column(1, 1, 10)
        self.page.set_column(2, 2, 20)
        self.page.set_column(3, 3, 100)

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

    def init_ros(self):
        #rospy.init_node("logger", anonymous=True)
        self.sub = rospy.Subscriber("/rosout_agg", Log, self.log_callback)
        rospy.loginfo("Rosout logger initialized")

    def close(self):
        self.page.autofilter(1, 0, self.row_index, 2)

if __name__ == "__main__":
    workbook = xlsxwriter.Workbook('demo.xlsx')
    worksheet = workbook.add_worksheet()
    logger = LogRosout(workbook, worksheet)
    rospy.spin()
    logger.close()
    workbook.close()
