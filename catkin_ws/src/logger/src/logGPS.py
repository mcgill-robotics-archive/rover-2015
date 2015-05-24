#!/usr/bin/python

import xlsxwriter
import rospy
import datetime

from rover_msgs.msg import GPS

"""
------------------------------------------------------
| TimeStamp | Node | Latitude | Longitude | Altitude |
------------------------------------------------------
"""


class LogRawGPS():
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
        big_font = self.workbook.add_format({'font_size': '30'})
        self.page.write_string(0, 0, "GPS Log", big_font)
        self.page.set_row(0, 35)

        self.page.write_string(1, 0, "Timestanp")
        self.page.write_string(1, 1, "Node")
        self.page.write_string(1, 2, "Latitude")
        self.page.write_string(1, 3, "Longitude")
        self.page.write_string(1, 4, "Altitude")

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
        self.page.set_column(1, 1, 5)
        self.page.set_column(2, 4, 20)

    def log_callback(self, message):
        timestamp = datetime.datetime(message.year,
                                      message.month,
                                      message.day,
                                      message.hour,
                                      message.minute,
                                      message.sec,
                                      message.csec)
        self.page.write_datetime(self.row_index,
                                 0,
                                 timestamp,
                                 self.extended_date)

        self.page.write_string(self.row_index, 2, str(message.latitude))
        self.page.write_string(self.row_index, 3, str(message.longitude))
        self.page.write_string(self.row_index, 4, str(message.altitude))

        self.row_index += 1

        print message

    def init_ros(self):
        self.sub = rospy.Subscriber("/raw_gps", GPS, self.log_callback)
        rospy.loginfo("GPS logger initialized")

    def close(self):
        self.page.autofilter(1, 0, self.row_index, 2)

if __name__ == "__main__":
    workbook = xlsxwriter.Workbook('demo.xlsx')
    worksheet = workbook.add_worksheet()
    rospy.init_node("logger", anonymous=True)
    logger = LogRawGPS(workbook, worksheet)
    rospy.spin()
    logger.close()
    workbook.close()
