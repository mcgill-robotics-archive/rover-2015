#!/usr/bin/python

import xlsxwriter
import rospy
from control_systems.msg import SetPoints
from std_msgs.msg import Int16

batteryColumn = 1


class GenerateReport():
    def __init__(self):
        filename = "report.xlsx"
        self.document = xlsxwriter.Workbook("report.xlsx")        
        self.graphSheet = self.document.add_worksheet('graphs')
        self.dataSheet = self.document.add_worksheet('data')   
      
        self.rowIndex = 0

        battery1topic = rospy.get_param("battery1topic", "integers")
        rospy.init_node("excelBuilder", anonymous=False)

        self.sub1 = rospy.Subscriber(battery1topic, Int16, self.recordBattery)
        rospy.spin()


    def recordBattery(self, batteryLevel):
        self.dataSheet.write(self.rowIndex, batteryColumn, batteryLevel.data)
        self.dataSheet.write(self.rowIndex, batteryColumn+1, batteryLevel.data-1)
        self.rowIndex += 1


    def createGraph(self): 
        batterySerie1 = ['data', 0, batteryColumn, self.rowIndex-1, batteryColumn]  
        batterySerie2 = ['data', 0, batteryColumn+1, self.rowIndex-1, batteryColumn+1]  
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


    def close(self):
        self.createGraph()
        self.document.close()
        print "exit"

if __name__=="__main__":
    reportGernerate=None
    try:
        reportGernerate=GenerateReport()
    finally:
        reportGernerate.close()
        print "Clean exit"