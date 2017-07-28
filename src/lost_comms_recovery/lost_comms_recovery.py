#!/usr/bin/env python
import sys
import time
import rospy
import subprocess
import actionlib

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def ping_host(host):
  ping_fail_count = rospy.get_param('~ping_fail_count', 2)
  ping_command = "ping -c %s -n -W 1 %s" % (ping_fail_count, host)
  # TODO: don't shell out, use a more secure python library
  p = subprocess.Popen(ping_command,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE,
                                   shell=True)
  (output, error) = p.communicate()
  returncode = p.returncode
  return output, error, returncode

class RecorveryController():
  def __init__(self):
    # By default recover to a pose at the origin of the frame
    self.recovery_pose = PoseWithCovarianceStamped()
    self.recovery_pose.pose.pose.position.x = 0.0
    self.recovery_pose.pose.pose.position.y = 0.0
    self.recovery_pose.pose.pose.position.z =  0.0
    self.recovery_pose.pose.pose.orientation.x = 0.0
    self.recovery_pose.pose.pose.orientation.y = 0.0
    self.recovery_pose.pose.pose.orientation.z = 0.0
    self.recovery_pose.pose.pose.orientation.w = 1

    rospy.Subscriber('recovery_pose', PoseWithCovarianceStamped,
                     self.recovery_pose_callback)
    self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.joy_publisher = rospy.Publisher('joy', Joy, queue_size=10)

  def recovery_pose_callback(self, data):
    rospy.loginfo("Not connected to move_base.")
    self.recovery_pose = data

  def working_comms(self):
    working_comms = False
    for ip in self.ips.split(','):
      (output, error, returncode) = ping_host(ip)
      if returncode == 0:
        working_comms = True
    return working_comms

  def zero_joystick(self):
    rospy.loginfo('Zeroing joystick.')
    joy = Joy()
    joy.axes = [0] * 25 # Set axes to an array of zeros. Needed because
      # the default value here is an empty list which won't zero anything
    joy.buttons = [0] * 25 # Set buttons to an array of zeros
    self.joy_publisher.publish(joy)

  def stop_motors(self):
    rospy.loginfo('Stopping motors.')
    twist = Twist() # zero motion
    self.twist_publisher.publish(twist)

  def connect_to_move_base(self):
    self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    if self.move_base.wait_for_server(timeout=rospy.Duration(3)):
      rospy.loginfo("Connected to move_base.")
      self.connected_to_move_base = True
    else:
      rospy.loginfo("Not connected to move_base.")
      self.connected_to_move_base = False
    return self.connected_to_move_base

  def navigation_goal_to(self, pose):
    # Create move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.pose = pose.pose.pose
    goal.target_pose.header.frame_id = rospy.get_param('~goal_frame_id', 'map')
    rospy.loginfo('Executing move_base goal to position (x,y) %s, %s.' %
            (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))

    # Send goal
    self.move_base.send_goal(goal)
    rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
        rospy.loginfo(status)

    # Wait for goal result
    self.move_base.wait_for_result()
    rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
        rospy.loginfo(status)

  def goal_in_progress(self):
    data = rospy.wait_for_message('/move_base/status', GoalStatusArray)
    if data.status_list == []: # you see this if move_base just started
      return False
    # See possible statuses http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    PENDING = 0
    ACTIVE = 1
    status_codes = [status.status for status in data.status_list]
    if PENDING in status_codes:
      return True
    if ACTIVE in status_codes:
      return True
    return False

  def do_recovery(self):
    if rospy.is_shutdown(): return
    rospy.logerr('No connection to base station.')
    if self.connect_to_move_base():
      if self.goal_in_progress():
        rospy.loginfo("Navigation in progress, not recovering until finished...")
        return
      self.navigation_goal_to(self.recovery_pose)
    else:
      self.zero_joystick()
      self.stop_motors()

  def main_loop(self):
    while not rospy.is_shutdown():
      if not self.working_comms():
        self.do_recovery()
      else:
        rospy.loginfo('Connected to base station.')
      time.sleep(3)


def main():
  rospy.init_node("lost_comms_recovery")
  Controller = RecorveryController()
  Controller.ips = rospy.get_param('~ips_to_monitor')
  rospy.loginfo('Monitoring base station on IP(s): %s.' % Controller.ips)
  Controller.main_loop() # start monitoring
