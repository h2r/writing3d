#!/usr/bin/env python
#
# Publish to angular_vel_cmd
# Publish to velocity_vel_cmd

import rospy
from std_msgs.msg import Header
from movo_msgs.msg import JacoAngularVelocityCmd7DOF

SEQ = 1000

def pose_publisher(msg):
    pub = rospy.Publisher("movo/right_arm/angular_vel_cmd", JacoAngularVelocityCmd7DOF, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing %s" % msg)
        pub.publish(msg)
        rate.sleep()

def angular_pose(indices=[], new_vals=[]):
    global SEQ
    msg = JacoAngularVelocityCmd7DOF()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = SEQ; SEQ += 1

    vals = [0]*7
    if len(indices) > 0 and len(new_vals) > 0 and len(indices) == len(new_vals):
        for indx, i in enumerate(indices):
            vals[i] = new_vals[indx]
    
    msg.theta_shoulder_pan_joint = vals[0]
    msg.theta_shoulder_lift_joint = vals[1]
    msg.theta_arm_half_joint = vals[2]
    msg.theta_elbow_joint = vals[3]
    msg.theta_wrist_spherical_1_joint = vals[4]
    msg.theta_wrist_spherical_2_joint = vals[5]
    msg.theta_wrist_3_joint = vals[6]
    return msg


if __name__ == "__main__":
    rospy.init_node("movo_pose_node", anonymous=True)

    try:
        msg = angular_pose([3], [-0.4])
        pose_publisher(msg)
    except rospy.ROSInterruptException:
        pass
