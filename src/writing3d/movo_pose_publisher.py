#!/usr/bin/env python
#
# Publish to angular_vel_cmd
# Publish to velocity_vel_cmd

import rospy
from std_msgs.msg import Header
from movo_msgs.msg import JacoAngularVelocityCmd7DOF, JacoCartesianVelocityCmd

import argparse

SEQ = 1000

def pose_publisher(msg, arm="right", rate=1):
    if isinstance(msg, JacoCartesianVelocityCmd):
        pub = rospy.Publisher("movo/%s_arm/cartesian_vel_cmd" % arm, JacoCartesianVelocityCmd, queue_size=10)
    else:
        pub = rospy.Publisher("movo/%s_arm/angular_vel_cmd" % arm, JacoAngularVelocityCmd7DOF, queue_size=10)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing %s" % msg)
        pub.publish(msg)
        r.sleep()

def angular_vel(indices=[], new_vals=[]):
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

def cartesian_vel(indices=[], new_vals=[]):
    # cartesian velocity does not have joint-specific setting. For the entire arm,
    # there are:
    #
    # x, y, z, theta_x, theta_y, theta_z
    global SEQ
    msg = JacoCartesianVelocityCmd()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = SEQ; SEQ += 1

    vals = [0] * 6
    if len(indices) > 0 and len(new_vals) > 0 and len(indices) == len(new_vals):
        for indx, i in enumerate(indices):
            vals[i] = new_vals[indx]

    msg.x = vals[0]
    msg.y = vals[1]
    msg.z = vals[2]
    msg.theta_x = vals[3]
    msg.theta_y = vals[4]
    msg.theta_z = vals[5]
    return msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Publish velocity commands to MOVO arm joints.')
    parser.add_argument("side", type=str, help="Which arm to move. 'left' or 'right'")
    parser.add_argument('-i', '--indices', type=int, nargs='+',
                        help='Indices of joints, or indices of coordinates (if using cartesian).')
    parser.add_argument('-v', '--vals', type=float, nargs='+',
                        help='Values to set for the corresponding indices.')
    parser.add_argument('-c', '--cartesian', help='Cartesian velocity commands', action="store_true")
    parser.add_argument('-r', '--rate', type=float, help="Rate for publishing velocity command", default=1.0)
    args = parser.parse_args()

    rospy.init_node("movo_pose_node", anonymous=True)

    if args.side.lower() != "left" and args.side.lower() != "right":
        raise ValueError("side must be either left or right!")

    indices = args.indices if args.indices is not None else []
    vals = args.vals if args.vals is not None else []

    try:
        if args.cartesian:
            msg = cartesian_vel(indices, vals)
        else:
            msg = angular_vel(indices, vals)
        pose_publisher(msg, args.side.lower(), rate=args.rate)
    except rospy.ROSInterruptException:
        pass
