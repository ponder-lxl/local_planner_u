#!/usr/bin/env python3
import sys

import rospy
from geometry_msgs.msg import PoseStamped


def main():
  rospy.init_node("publish_final_goal")

  if len(sys.argv) < 3:
    print("Usage: rosrun polar_gap_planner publish_final_goal.py X Y [Z]")
    return

  x = float(sys.argv[1])
  y = float(sys.argv[2])
  z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.75

  pub = rospy.Publisher("/final_goal", PoseStamped, queue_size=1, latch=True)
  rospy.sleep(0.5)

  msg = PoseStamped()
  msg.header.frame_id = "map"
  msg.header.stamp = rospy.Time.now()
  msg.pose.position.x = x
  msg.pose.position.y = y
  msg.pose.position.z = z
  msg.pose.orientation.w = 1.0

  pub.publish(msg)
  rospy.loginfo("Published /final_goal: (%.2f, %.2f, %.2f)", x, y, z)


if __name__ == "__main__":
  main()
