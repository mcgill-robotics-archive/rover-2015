#TODO:Demonstration of desired operation MAKE FUNCTIONAL
def setFeed1Index(self, newIndex):
    rospy.wait_for_service('/changeFeed')
    try:
        resp = self.switch_feed(1, newIndex)
    except rospy.ServiceException:
        rospy.logwarn("Service call failed")
        self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
        return
    if resp.success is 200:
        self.feed1index = newIndex
    else:
        rospy.loginfo("Switch in camera1 failed")

def setFeed2Index(self, newIndex):
    rospy.wait_for_service('/changeFeed')
    try:
        resp = self.switch_feed(2, newIndex)
    except rospy.ServiceException:
        rospy.logwarn("Service call failed")
        self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
        return
    if resp.success is 200:
        self.feed1index = newIndex
    else:
        rospy.loginfo("Switch in camera1 failed")

def setFeed3Index(self, newIndex):
    rospy.wait_for_service('/changeFeed')
    try:
        resp = self.switch_feed(3, newIndex)
    except rospy.ServiceException:
        rospy.logwarn("Service call failed")
        self.ui.Camera1Feed.setCurrentIndex(self.feed1index)
        return
    if resp.success is 200:
        self.feed1index = newIndex
    else:
        rospy.loginfo("Switch in camera1 failed")


def get_image_topic(self):
    self.camera_topic_list.append(rospy.get_param("camera/topic_haz_front", "/hazcam_front/camera/image_raw"))
    self.camera_topic_list.append(rospy.get_param("camera/topic_haz_left", "/hazcam_left/camera/image_raw"))
    self.camera_topic_list.append(rospy.get_param("camera/topic_haz_right", "/hazcam_right/camera/image_raw"))
    self.camera_topic_list.append(rospy.get_param("camera/topic_haz_back", "/hazcam_back/camera/image_raw"))
    self.camera_topic_list.append(rospy.get_param("camera/topic_arm", "/camera_arm/camera/image_raw"))
    self.camera_topic_list.append(rospy.get_param("camera/topic_pan_tilt", "/camera_pan_tilt/camera/image_raw"))
    rospy.loginfo(self.camera_topic_list)
