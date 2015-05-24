#!/usr/bin/python

import xlsxwriter
import rospy
import time
from std_msgs.msg import Int16
from odometry.msg import RoverSpeed
from logRosout import LogRosout
from logGPS import LogRawGPS


class GenerateReport():
    def __init__(self):
        filename = "report.xlsx"
        self.document = xlsxwriter.Workbook(filename)
        self.graphSheet = self.document.add_worksheet('graphs')
        self.dataSheets = [self.document.add_worksheet('battery'),
                           self.document.add_worksheet('speed')]
        self.speeds = RoverSpeed()
        self.rosout_sheet = self.document.add_worksheet("rosout")
        self.gps_sheet = self.document.add_worksheet("gps")

        self.logRosout = LogRosout(self.document, self.rosout_sheet)
        self.logGPS = LogRawGPS(self.document, self.gps_sheet)

        self.index = [0, 0]
        self.graphs = 0
        self.startTime = time.time()

        battery1topic = rospy.get_param("battery1topic", "integers")
        rospy.init_node("excelBuilder", anonymous=False)

        self.sub1 = rospy.Subscriber(battery1topic, Int16, self.record_battery)
        
        rospy.Subscriber('/odo_speeds', RoverSpeed, self.update_speeds, queue_size=10)

    def update_speeds(self, msg):
        self.speeds = msg

    def record_speeds(self):
        self.dataSheets[1].write(self.index[1], 1, time.time()-self.startTime)
        self.dataSheets[1].write(self.index[1], 2, self.speeds.linear)
        self.dataSheets[1].write(self.index[1], 3, self.speeds.angular)
        self.index[1] += 1

    def record_battery(self, batteryLevel):
        self.dataSheets[0].write(self.index[0], 1, batteryLevel.data)
        self.dataSheets[0].write(self.index[0], 2,
            batteryLevel.data-1)
        self.index[0] += 1

    def create_graph(self, sheet, x, y, labelx, labely, title):
        serie1 = ['speed', 0, x, self.index[sheet]-1, x]
        # serie2 = ['timedData', 0, y, self.index[sheet]-1, y]
        gr = self.document.add_chart({'type': 'line'})
        gr.add_series({
            'name':     labelx,
            'values':   serie1
            })
        # gr.add_series({
        #    'name':     labely,
        #    'values':   serie2
        #    })
        
        # Add a chart title and some axis labels.
        gr.set_title({'name': title})
        gr.set_x_axis({'name': labelx})
        gr.set_y_axis({'name': labely})

        # Insert the chart into the dataSheet (with an offset).
        self.dataSheets[sheet].insert_chart('H'+str(self.graphs), gr)
        self.graphs += 1

    def run(self):
        rospy.loginfo("Entered run state")
        # calculate required wheel angles, speeds
        r = rospy.Rate(60)
        # continue endlessly
        while not rospy.is_shutdown():
            # write the current speeds to file
            self.record_speeds()
            r.sleep()
    
        print "Closing"
        self.close()

    def close(self):
        self.logRosout.close()
        print"called close"
        # need to create graph for each relation we want to display
        self.create_graph(1, 1, 2, "Time (s)", "Linear Speed (m/s)", "Linear")
        print "done graph 1"
        self.create_graph(1, 1, 3, "Time (s)", "Angular Speed (rad/s)", "Angular")
        self.document.close()
        print "exit"

if __name__ == "__main__":
    print "Init node"
    graphGen = GenerateReport()
    print "Running node"
    graphGen.run()
    rospy.spin()
    print "Clean --------  ~     "

