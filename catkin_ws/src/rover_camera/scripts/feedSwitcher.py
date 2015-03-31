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
        self.camera_name = ["vid0", "vid1"]

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.handle_change_feed)

        rospy.loginfo('feed_switcher ready')
        print "Loaded"
        

    def handle_change_feed(self, req):
        screen = req.screenId
        camera = req.feedId

        name = self.camera_name[camera]
        dev = self.camera_list[camera]
        rospy.loginfo("Switching monitor %d to feed %d",screen,camera)
        if screen is 1:
            if self.screen_main_pid is not 0:
                kill_process(self.screen_main_pid)
            time.sleep(0.1)
            topic = "main/compressed"
            self.screen_main_pid = call_launch_file(name, dev, topic)
        elif screen is 2:
            if self.screen_top_pid is not 0:
                kill_process(self.screen_top_pid)
            time.sleep(0.1)
            topic = "top/compressed"
            self.screen_top_pid = call_launch_file(name, dev, topic)
        elif screen is 3:
            if self.screen_bottom_pid is not 0:
                kill_process(self.screen_bottom_pid)
            time.sleep(0.1)
            topic = "bottom/compressed"
            self.screen_bottom_pid = call_launch_file(name, dev, topic)

        return ChangeFeedResponse(200)

    def close(self):
        print
        print "Called close"
        kill_process(self.screen_main_pid)
        kill_process(self.screen_top_pid)
        kill_process(self.screen_bottom_pid)
        exit(0)

  

switcher = FeedSwitcher()
rospy.spin()
switcher.close()
