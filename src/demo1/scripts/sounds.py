#!/usr/bin/env python

# FUN ZONE
# add sound when robot is running
import rospy

from kobuki_msgs.msg import Sound

sounds = [RUN]
texts = ["run, robot, run"]

rospy.init_node("test_sounds")
pub = rospy.Publisher('/mobile_base/commands/sound', Sound)
rate = rospy.Rate(0.5)

# Added below two line of code
# to wait until at least one subscriber is present or connected.
# Because rospy.Publisher will skip(or eat) first certain messages, if you publish just after rospy.init()
# Without this patch, first message - Sound.ON will never be published.
# I think this is a bug of rospy
# Younghun Ju
while not pub.get_num_connections():
  rate.sleep()

msg = Sound()
while not rospy.is_shutdown():
    for sound, text in zip(sounds, texts):
        msg.value = sound
        print text
        pub.publish(msg)
        rate.sleep()
    break
