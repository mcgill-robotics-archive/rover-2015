#!/usr/bin/python

import rospy
from rover_camera.srv import *
import time

import subprocess


def call_launch_file(cam_name, device_file, output_topic):
    log = open("/home/artemis/"+cam_name,'a')
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
        topic_name], stdout=log, stderr=log)
    print "launched ", device_file
    return process.pid


def kill_process(pid):
    print "Killing pid ", pid
    if pid is not 0:
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

        self.camera_list = ["/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"]
        self.camera_name = ["vid0", "vid1", "vid2", "vid3"]

        rospy.init_node("feed_switcher", anonymous=False)
        s = rospy.Service('changeFeed', ChangeFeed, self.new_handle_change_feed)

        rospy.loginfo('feed_switcher ready')

    def new_handle_change_feed(self, req):

        desired_screen_id = req.screenId
        desired_feed_id = req.feedId

        try:
            other_screen_id = self.feed_map[desired_feed_id]
        except KeyError:
            other_screen_id = None

        if other_screen_id is None:
            # traditional switch
            if self.process_map[desired_screen_id] is not 0:
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
            other_feed_id = None
            for i in self.feed_map:
                if self.feed_map[i] is desired_screen_id:
                    other_feed_id = i
                    break

            try:
                kill_process(self.process_map[other_screen_id])  # kill the already assigned pid
                for i in self.feed_map:
                    if self.feed_map[i] is other_screen_id:
                        self.feed_map.pop(i)
                        break

                kill_process(self.process_map[desired_screen_id])  # kill the target screen
                for i in self.feed_map:
                    if self.feed_map[i] is desired_screen_id:
                        self.feed_map.pop(i)
                        break

            except KeyError:
                pass
            time.sleep(0.1)
            self.process_map[desired_screen_id] = call_launch_file(self.camera_name[desired_feed_id],
                                                                   self.camera_list[desired_feed_id],
                                                                   self.topic_list[desired_screen_id])
            self.feed_map[desired_feed_id] = desired_screen_id

            if other_feed_id is not None:
                self.process_map[other_screen_id] = call_launch_file(self.camera_name[other_feed_id],
                                                                     self.camera_list[other_feed_id],
                                                                     self.topic_list[other_screen_id])
                self.feed_map[other_feed_id] = other_screen_id

        return ChangeFeedResponse(self.process_map[desired_screen_id], None)

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
