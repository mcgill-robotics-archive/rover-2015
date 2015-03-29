#!/usr/bin/python

import rospy
from rover_camera.srv import *
import time

import subprocess


def call_launch_file(cam_name, device_file, output_topic):
    device = "device:=" + device_file
    camera = "name:=" + cam_name
    topic = "topic:=True"
    topic_name = "topic_name:=" + output_topic
    process = subprocess.Popen([
        "roslaunch",
        "rover_camera",
        "uvc.launch",
        device,
        camera,
        topic,
        topic_name])
    return process.pid


def kill_process(pid):
    pid_str = str(pid)
    subprocess.Popen(["kill", pid_str])


class FeedSwitcher():
    def __init__(self):

        self.screen_main_pid = 0
        self.screen_top_pid = 0
        self.screen_bottom_pid = 0

        self.camera_list = ["/dev/video0", "/dev/video1"]
        self.camera_name = ["haz_front", "haz_left"]

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.handle_change_feed)

        rospy.loginfo('feed_switcher ready')
        print "Loaded"
        rospy.spin()

    def handle_change_feed(self, req):
        screen = req.screenId
        camera = req.feedId
        name = self.camera_name[camera]
        dev = self.camera_list[camera]
        if screen is 1:
            kill_process(self.screen_main_pid)
            time.sleep(0.5)
            topic = "main/compressed"
            self.screen_main_pid = call_launch_file(name, dev, topic)
            rospy.logerr("AFTER THE PROCESS")
        elif screen is 2:
            kill_process(self.screen_top_pid)
            time.sleep(0.5)
            topic = "top/compressed"
            self.screen_top_pid = call_launch_file(name, dev, topic)
        elif screen is 3:
            kill_process(self.screen_bottom_pid)
            time.sleep(0.5)
            topic = "bottom/compressed"
            self.screen_bottom_pid = call_launch_file(name, dev, topic)

        call_launch_file("banane", "/dev/video1", "foo")
        return ChangeFeedResponse(123)

    def dhandle_change_feed(self, req):
        screen = req.screenId
        camera = req.feedId

        if camera > len(self.camera_list)-1 or 0 > screen > 3:
            return 400

        name = self.camera_name[camera]
        dev = self.camera_list[camera]

        if screen is 1:
            kill_process(self.screen_main_pid)
            topic = "main/compressed"
            self.screen_main_pid = call_launch_file(name, dev, topic)
            rospy.logerr("AFTER THE PROCESS")
        elif screen is 2:
            kill_process(self.screen_top_pid)
            topic = "top/compressed"
            self.screen_top_pid = call_launch_file(name, dev, topic)
        elif screen is 3:
            kill_process(self.screen_bottom_pid)
            topic = "bottom/compressed"
            self.screen_bottom_pid = call_launch_file(name, dev, topic)

        return ChangeFeedResponse(200)


switcher = FeedSwitcher()
