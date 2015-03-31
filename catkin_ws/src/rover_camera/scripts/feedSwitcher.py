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

        # screenId : PID
        self.process_map = {1:0, 2:0, 3:0}
        self.topic_list = {1:"main/compressed", 2:"top/compressed", 3:"bottom/compressed"}
        self.feed_map = {}

        self.camera_list = ["/dev/video0", "/dev/video1"]
        self.camera_name = ["vid0", "vid1"]

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.handle_change_feed)

        rospy.loginfo('feed_switcher ready')
        print "Loaded"

    def new_handle_change_feed(self, req):

        screen_id = req.screenId
        feed_id = req.feedId

        try:
            assigned_to_screen = self.feed_map[feed_id]
        except KeyError:
            assigned_to_screen = None

        if assigned_to_screen is None:
            # traditional switch
            if self.process_map[screen_id] is not 0:
                kill_process(self.process_map[screen_id])
            time.sleep(0.1)
            self.process_map[screen_id] = call_launch_file(self.camera_name[feed_id],
                                                    self.camera_list[feed_id],
                                                    self.topic_list[screen_id])
            self.feed_map[feed_id]=screen_id

        else:
            # screen interchange
            current_screen_feed = self.feed_map[screen_id]

            kill_process(self.process_map[assigned_to_screen]) # kill the already assigned pid
            kill_process(self.process_map[screen_id]) # kill the target screen

            self.process_map[screen_id] = call_launch_file(self.camera_name[feed_id],
                                                           self.camera_list[feed_id],
                                                           self.topic_list[screen_id])
            self.feed_map[feed_id]=screen_id

            self.process_map[assigned_to_screen] = call_launch_file(self.camera_name[current_screen_feed],
                                                                    self.camera_list[current_screen_feed],
                                                                    self.topic_list[assigned_to_screen])
            self.feed_map[current_screen_feed] = assigned_to_screen
            pass

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
