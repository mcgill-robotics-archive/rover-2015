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
    print "launched ", device_file
    return process.pid


def kill_process(pid):
    print "Killing pid ", pid
    pid_str = str(pid)
    subprocess.Popen(["kill", pid_str])


class FeedSwitcher():
    def __init__(self):

        self.screen_main_pid = 0
        self.screen_top_pid = 0
        self.screen_bottom_pid = 0

        # screenId : PID
        self.process_map = {1: 0, 2: 0, 3: 0}
        self.topic_list = {1: "main/compressed", 2: "top/compressed", 3: "bottom/compressed"}

        # feedId : screedId
        self.feed_map = {}

        self.camera_list = ["/dev/video0", "/dev/video1"]
        self.camera_name = ["vid0", "vid1"]

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.new_handle_change_feed)

        rospy.loginfo('feed_switcher ready')
        print "Loaded"

    def new_handle_change_feed(self, req):

        desired_screen_id = req.screenId
        desired_feed_id = req.feedId

        try:
            other_screen_id = self.feed_map[desired_feed_id]
        except KeyError:
            other_screen_id = None

        if other_screen_id is None:
            # traditional switch
            print "TRAD !!!!"
            print "TRAD !!!!"
            print "TRAD !!!!"
            print "TRAD !!!!"
            print "TRAD !!!!"
            if self.process_map[desired_screen_id] is not 0:
                print(self.process_map[desired_screen_id], "is what we kill")
                kill_process(self.process_map[desired_screen_id])
                for i in self.feed_map:
                    if self.feed_map[i] is desired_screen_id:
                        self.feed_map.pop(i)
                        break
            time.sleep(0.1)
            self.process_map[desired_screen_id] = call_launch_file(self.camera_name[desired_feed_id],
                                                                   self.camera_list[desired_feed_id],
                                                                   self.topic_list[desired_screen_id])
            print("launched process with pid: ", self.process_map[desired_screen_id])
            self.feed_map[desired_feed_id] = desired_screen_id

        else:
            # screen interchange
            print "NEW"
            print "NEW"
            print "NEW"
            print "NEW"
            print "NEW"
            other_feed_id = self.feed_map[desired_screen_id]

            kill_process(self.process_map[other_screen_id])  # kill the already assigned pid
            kill_process(self.process_map[desired_screen_id])  # kill the target screen

            self.process_map[desired_screen_id] = call_launch_file(self.camera_name[desired_feed_id],
                                                                   self.camera_list[desired_feed_id],
                                                                   self.topic_list[desired_screen_id])
            self.feed_map[desired_feed_id] = desired_screen_id

            self.process_map[other_screen_id] = call_launch_file(self.camera_name[other_feed_id],
                                                                 self.camera_list[other_feed_id],
                                                                 self.topic_list[other_screen_id])
            self.feed_map[other_feed_id] = other_screen_id
            pass
        return ChangeFeedResponse(self.process_map[desired_screen_id])

    def handle_change_feed(self, req):
        screen = req.screenId
        camera = req.feedId

        name = self.camera_name[camera]
        dev = self.camera_list[camera]
        rospy.loginfo("Switching monitor %d to feed %d", screen, camera)
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
