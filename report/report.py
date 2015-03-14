#!/usr/bin/python

import xlsxwriter,rospy,time
from control_systems.msg import SetPoints
from std_msgs.msg import Int16
from odometry.msg import RoverSpeed

1 = 10



class GenerateReport():
    def __init__(self):
        filename = "report.xlsx"
        self.document = xlsxwriter.Workbook("report.xlsx")        
        self.graphSheet = self.document.add_worksheet('graphs')
        self.dataSheet = self.document.add_worksheet('data')
        self.dataSheetTimed = self.document.add_worksheet('timedData')
        self.speeds = RoverSpeed()
      
        self.rowIndex = 0
        self.timeIndex = 0

        battery1topic = rospy.get_param("battery1topic", "integers")
        rospy.init_node("excelBuilder", anonymous=False)

        self.sub1 = rospy.Subscriber(battery1topic, Int16, self.recordBattery)
        
        rospy.Subscriber('/odo_speeds',RoverSpeed,self.update_speeds,
            queue_size=10)

    def update_speeds(self, msg):
        self.speeds = msg

    def recordSpeeds(self):
        self.dataSheetTimed.write(self.timeIndex, 1, time.time())
        self.dataSheetTimed.write(self.timeIndex, 2, self.speeds.linear)

        self.timeIndex += 1

    def recordBattery(self, batteryLevel):
        self.dataSheet.write(self.rowIndex, 1, batteryLevel.data)
        self.dataSheet.write(self.rowIndex, 2, 
            batteryLevel.data-1)
        self.rowIndex += 1


    def createGraph(self): 
        batterySerie1 = ['data', 0, 1, self.rowIndex-1, 
            1]  
        batterySerie2 = ['data', 0, 2, self.rowIndex-1, 
            2]  
        gr1 = self.document.add_chart({'type': 'line'})
        gr1.add_series({
            'name':     'Battery 1',
            'values':   batterySerie1
            })
        gr1.add_series({
            'name':     'Battery 2',
            'values':   batterySerie2
            })
        
        # Add a chart title and some axis labels.
        gr1.set_title ({'name': 'Battery Voltage'})
        gr1.set_x_axis({'name': 'Time (s)'})
        gr1.set_y_axis({'name': 'Volts'})

        # Insert the chart into the dataSheet (with an offset).
        self.graphSheet.insert_chart('A1', gr1)

    def run(self):
        #calculate required wheel angles, speeds
        r = rospy.Rate(60)
        #continue endlessly
        while not rospy.is_shutdown():
            #update time
            self.clock = time.time()
            #write the current speeds to file
            self.recordSpeeds()
            r.sleep()
        self.close()

    def close(self):
        self.createGraph()
        self.document.close()
        print "exit"

if __name__=="__main__":
    print "Init node"
    graphGen=GenerateReport()
    print "Running node"
    graphGen.run()
    rospy.spin()
    print "Clean --------  ~     "