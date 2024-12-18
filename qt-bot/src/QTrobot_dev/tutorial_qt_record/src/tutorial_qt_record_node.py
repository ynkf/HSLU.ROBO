#!/usr/bin/env python3
import sys
import rospy
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
from qt_motors_controller.srv import *

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gestureRecord = rospy.ServiceProxy('/qt_robot/gesture/record', gesture_record)
    gestureSave = rospy.ServiceProxy('/qt_robot/gesture/save', gesture_save)
    setControlMode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)


    try:
        name = "look_around_custom"
        parts = ["left_arm", "right_arm", "head"]
        res = setControlMode(parts, 1)
        if not res.status:
            rospy.logfatal("Could not set control mode of '%s'." % parts)
        input('Press enter to START recording ...\n')
        res = gestureRecord(parts, True, 0, 0)
        if not res.status:
            rospy.logfatal("Could not start recording gesture '%s' using '%s'." % (name, parts))
        speechSay('When you want to STOP recording, just press enter again')
        input('Press enter to STOP recording ...\n')
        res = gestureSave(name, "")
        if not res.status:
            rospy.logfatal("Could not save gesture '%s'." % name)
        else:
            rospy.loginfo("Gesture '%s' was recorded." % name)
        res = setControlMode(parts, 0)
        if not res.status:
            rospy.logfatal("Could not set control mode of '%s'." % parts)
        else:
            gesturePlay(name, 0)

    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")