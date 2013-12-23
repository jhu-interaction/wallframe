#!/usr/bin/env python
#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# # Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# # Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# # Neither the name of the Johns Hopkins University nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

###
# Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
###
"""
  Wallframe App_ Menu
  First prototype of wallframe_app_menu. Uses Model-View/Controller approach.
  View and controller are combined because Qt signal/slots intrinsically do not cater
  to a separated view and controller.
  @author Yifan Ge, Andy Tien, Kel Guerin, Zihan Chen
"""
# ROS import
import roslib; roslib.load_manifest('wallframe_core')
import rospy
from std_msgs.msg import String
# system import
from math import fabs
import os, collections, sys, math
# PySide import
from PySide.QtGui import *
from PySide.QtCore import *
from PySide import QtCore
# wallframe import
# msg
from wallframe_msgs.msg import WallframeUser
from wallframe_msgs.msg import WallframeUserArray
from wallframe_msgs.msg import WallframeUserEvent
from wallframe_extra.msg import WallframeMouseState
# srv
import wallframe_core
from wallframe_core import WallframeAppWidget
from wallframe_core.srv import *
from tileflow import TileflowWidget

class AppMenu(WallframeAppWidget):
  Y_THRES = 300
  X_LONG_THRES = 80
  X_MID_THRES = 80
  X_SHORT_THRES = 30

  signal_hide_ = QtCore.Signal()
  signal_show_ = QtCore.Signal()
  signal_click_ = QtCore.Signal()
  # constructor
  def __init__(self):
    self.qt_app_ = QApplication(sys.argv)
    QWidget.__init__(self)

    # member variables
    self.current_users_ = []     # list of users
    self.users_ = {}             # dict: (wallframe_id, user)
    self.num_users_ = 0          # total num of users
    self.focused_user_id_ = -1   # focused user
    self.app_menu_items_ = []         # list of the app names
    self.hidden_ = False
    self.run_ = False
    self.mouse_state = (0, 0)
    self.prev_user = None
    self.current_user = None
    self.state = "IDLE" #IDLE LEFT RIGHT
    # ROS
    rospy.init_node('wallframe_app_tileflow_menu', anonymous=True)

    # ---- ROS subscriber ---
    self.user_state_sub_ = rospy.Subscriber("/wallframe/users/state",
                                            WallframeUserArray,
                                            self.user_state_cb)

    self.user_event_sub_ = rospy.Subscriber("/wallframe/users/events",
                                            WallframeUserEvent,
                                            self.user_event_cb)

    self.mouse_state_sub = rospy.Subscriber("/wallframe/extra/mousestate",
                                            WallframeMouseState,
                                            self.mouse_state_cb)
    self.toast_pub_ = rospy.Publisher("/wallframe/info/toast", String)

    # ---- ROS get params -----
    # height
    if rospy.has_param("/wallframe/core/params/height"):
      self.height_ = rospy.get_param("/wallframe/core/params/height")
    else:
      rospy.logerr("WallframeAppMenu: parameter [height] not found on server")
    # width
    if rospy.has_param("/wallframe/core/params/width"):
      self.width_ = rospy.get_param("/wallframe/core/params/width")
    else:
      rospy.logerr("WallframeAppMenu: parameter [width] not found on server")
    ### x ###
    if rospy.has_param("/wallframe/core/params/x"):
      self.x_ = rospy.get_param("/wallframe/core/params/x")
    else:
      rospy.logerr("WallframeAppMenu: parameter [x] not found on server")
    ### y ###
    if rospy.has_param("/wallframe/core/params/y"):
      self.y_ = rospy.get_param("/wallframe/core/params/y")
    else:
      rospy.logerr("WallframeAppMenu: parameter [y] not found on server")
    ### border scale ###
    if rospy.has_param("/wallframe/menu/params/border_scale"):
      self.border_scale_ = rospy.get_param("/wallframe/menu/params/border_scale")
    else:
      rospy.logerr("WallframeAppMenu: parameter [border_scale] not found on server")
    self.border_ = int(self.width_ * self.border_scale_)
    ### Cursor Icon ###
    if rospy.has_param("/wallframe/menu/params/cursor_path"):
      self.cursor_path_ = rospy.get_param("/wallframe/menu/params/cursor_path")
    else:
      rospy.logerr("WallframeAppMenu: parameter [cursor_path] not found on server")
    ### Cursor AltIcon ###
    if rospy.has_param("/wallframe/menu/params/cursor_path_alt"):
      self.cursor_path_alt_ = rospy.get_param("/wallframe/menu/params/cursor_path_alt")
    else:
      rospy.logerr("WallframeAppMenu: parameter [cursor_path_alt] not found on server")
    ### Application Locations ###
    if rospy.has_param("/wallframe/core/available_apps"):
      self.app_paths_ = rospy.get_param("/wallframe/core/available_apps")
    else:
      rospy.logerr("WallframeAppMenu: parameter [available_apps] not found on server")
    ### Default App ###
    if rospy.has_param("/wallframe/core/default_app"):
      self.default_app_name_ = rospy.get_param("/wallframe/core/default_app")
    else:
      rospy.logerr("WallframeAppMenu: parameter [default_app] not found on server")
    ### Go to screensaver ###
    if rospy.has_param("/wallframe/menu/params/screensaver"):
      self.screensaver_ = rospy.get_param("/wallframe/menu/params/screensaver")
    else:
      rospy.logerr("WallframeAppMenu: parameter [screensaver] not found on server")
    if self.screensaver_ == True:
      rospy.logwarn("WallframeAppMenu: Configured to SHOW screensaver app when all users leave")
    else:
      rospy.logwarn("WallframeAppMenu: Configured to NOT default to screensaver app when all users leave")
    ### Workspace Limits ###
    if rospy.has_param("/wallframe/menu/params/workspace_size"):
      self.workspace_limits_ = rospy.get_param("/wallframe/menu/params/workspace_size")
    else:
      rospy.logerr("WallframeAppMenu: parameter [workspace_size] not found on server")
    ### Y Offset ###
    if rospy.has_param("/wallframe/menu/params/y_offset"):
      self.y_offset_ = rospy.get_param("/wallframe/menu/params/y_offset")
    else:
      rospy.logerr("WallframeAppMenu: parameter [y_offset] not found on server")
    ### Height Scaling ###
    if rospy.has_param("/wallframe/menu/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/wallframe/menu/params/height_percentage")
    else:
      rospy.logerr("WallframeAppMenu: parameter [height_percentage] not found on server")
    rospy.logwarn("WallframeAppMenu: height percentage set to " + str(self.height_perc_))
    self.height_ = int(self.height_*self.height_perc_)

    # Get app list
    self.app_list_ = self.app_paths_.keys()
    rospy.logwarn("WallframeMenu: found " + str(len(self.app_list_)) + " applications")

    self.grid_set_up_ = False
    self.setup_grid()
    self.setWindowTitle("Wallframe Main Menu")
    self.box_layout_ = QHBoxLayout()
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.show()
    self.resize(self.width_, self.height_)
    self.move(self.x_,self.y_)
    self.setLayout(self.box_layout_)


    # Init TileflowWidget
    self.init_tileflow()
    # Timers
    self.ok_timer_ = QtCore.QTimer()
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)
    self.run_ = True
    # Hide and Show Connections
    self.signal_show_.connect(self.show_menu)
    self.signal_hide_.connect(self.hide_menu)
    self.signal_click_.connect(self.click)

    self.show_tooltip("Place left hand on right elbow to click")

  def check_ok(self):
    if rospy.is_shutdown():
      self.qt_app_.exit()
    else:
      self.update_tiles()
      pass
    pass

  def setup_grid(self):
    self.max_x_ = 4
    self.max_y_ = 6
    rospy.logwarn('WallframeMenu:  Grid size is '+str(self.max_x_)+' (w) by , '+str(self.max_y_)+' (h)')
    self.cur_ind_x_ = 0
    self.cur_ind_y_ = 0
    self.grid_set_up_ = True


  def convert_workspace(self,user_pos):
    screen_pos = []
    x_min = self.workspace_limits_[0]
    x_max = self.workspace_limits_[1]
    y_min = self.workspace_limits_[2]
    y_max = self.workspace_limits_[3]
    x_total = fabs(x_max)+fabs(x_min)
    y_total = fabs(y_max)+fabs(y_min)
    x_center = int(self.width_/2)
    y_center = int(self.height_/2)
    x_pos = int(x_center + (self.width_/x_total)*user_pos[0])
    y_pos = int(y_center + (-(self.height_/y_total)*user_pos[1]) - self.y_offset_)

    if y_pos < 0.0:
      y_pos = 100
    if y_pos > self.height_:
      y_pos = self.height_-100
    if x_pos < 0.0:
      x_pos = 100
    if x_pos > self.width_:
      x_pos = self.width_-100

    screen_pos = [x_pos, y_pos]
    return screen_pos
    pass


  def init_tileflow(self):
    res_list = [item[1] + '/menu_icon.png' for item in self.app_paths_.items()]
    self.tileflow_widget = TileflowWidget(self, res_list)
    self.box_layout_.addWidget(self.tileflow_widget)
    self.app_menu_items_ = [item[0] for item in self.app_paths_.items()]
    print self.app_menu_items_

  def user_state_cb(self, msg):
    if self.run_:
      if self.focused_user_id_ != -1:
        self.prev_user = self.users_[self.focused_user_id_]

      current_users_ = msg.users
      self.num_users_ = len(current_users_)
      self.users_.clear()

      # To check if there is any focused user
      has_focused_user = False

      for user in current_users_:
        if user.focused == True:
          has_focused_user = True
          # if there is a new focused user then we need to delete the prev_user
          # data because it is for a diff user
          if(user.wallframe_id != self.focused_user_id_):
            self.prev_user = None
          self.focused_user_id_ = user.wallframe_id
        self.users_[user.wallframe_id] = user


      if(not has_focused_user):
        self.focused_user_id_ = -1
      else:
        self.current_user = self.users_[self.focused_user_id_]

  def mouse_state_cb(self, msg):
    if self.run_:
      self.mouse_state = (msg.x, msg.y)

  def user_event_cb(self, msg):
    if self.run_:
      ### Workspace Events ###
      if msg.event_id == 'workspace_event':
        if self.screensaver_:
          if msg.message == 'all_users_left':
            rospy.logdebug("WallframeMenu: ALL_USERS_LEFT received, should start default app")
            self.toast_pub_.publish(String('Launching Screensaver'))
            rospy.wait_for_service('wallframe/core/app_manager/close_all_apps')
            try:
              self.srv_close_all_apps = rospy.ServiceProxy('wallframe/core/app_manager/close_all_apps',
                                                     wallframe_core.srv.close_all_apps)
              ret_success = self.srv_close_all_apps('none')
              # If close all apps is successful, hide menu and run default app
              self.load_app(self.default_app_name_)
              # self.signal_hide_.emit()
            except rospy.ServiceException, e:
              rospy.logerr("Service call failed: %s" % e)
        else:
          if msg.message == 'all_users_left':
            rospy.logdebug("WallframeMenu: ALL_USERS_LEFT received, should close app and show menu")
            self.toast_pub_.publish(String('Closing Apps'))
            rospy.wait_for_service('wallframe/core/app_manager/close_all_apps')
            try:
              self.srv_close_all_apps = rospy.ServiceProxy('wallframe/core/app_manager/close_all_apps',
                                                     wallframe_core.srv.close_all_apps)
              ret_success = self.srv_close_all_apps('none')
              # If close all apps is successful, show menu
              self.signal_show_.emit()
              self.toast_pub_.publish(String('Apps Closed'))
            except rospy.ServiceException, e:
              rospy.logerr("Service call failed: %s" % e)

      ### User Events ###
      if msg.event_id == 'hand_event' and msg.user_id == self.focused_user_id_:
        # Hands on head to quit app
        if msg.message == 'hands_on_head':
          rospy.logdebug("WallframeMenu: HANDS_HEAD received, should resume menu")
          self.toast_pub_.publish(String('Closing All Apps'))
          rospy.wait_for_service('wallframe/core/app_manager/close_all_apps')
          try:
            self.srv_close_all_apps = rospy.ServiceProxy('wallframe/core/app_manager/close_all_apps',
                                                   wallframe_core.srv.close_all_apps)
            ret_success = self.srv_close_all_apps('none')
            print ret_success
            self.signal_show_.emit()
            self.toast_pub_.publish(String('Apps Closed'))
          except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        # Click to start app
        if msg.message == 'left_elbow_click':
          print "captured left_elbow_click"
          self.signal_click_.emit()
          if self.hidden_ == False:
            print msg.message
            rospy.logwarn("WallframeMenu: LEFT_ELBOW_CLICK received, let's launch app")
            self.tileflow_widget.click()


  def clicked_on(self, ind):
    current_app_name = self.app_menu_items_[ind]
    print "clicked on " + current_app_name
    self.load_app(current_app_name)

  def load_app(self, app_name):
    self.toast_pub_.publish(String('Loading App ' + app_name))
    self.signal_hide_.emit()
    rospy.wait_for_service('wallframe/core/app_manager/load_app')
    try:
      self.srv_load_app = rospy.ServiceProxy('wallframe/core/app_manager/load_app',
                                             wallframe_core.srv.load_app)
      ret_success = self.srv_load_app(app_name)
      print ret_success
      self.toast_pub_.publish(String(app_name + " running"))
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s" % e)

  def show_tooltip(self, text):
    rospy.wait_for_service('tooltip/update_text')
    update_text_cb = rospy.ServiceProxy('tooltip/update_text', update_text)
    try:
        success = update_text_cb("Menu", text)
    except rospy.ServiceException as exc:
        rospy.logerr("WallframeTooltip: update_text service could not update the text")

  def hide_tooltip_from_menu(self):
    rospy.wait_for_service('tooltip/hide_tooltip')
    hide_cb = rospy.ServiceProxy('tooltip/hide_tooltip', hide_tooltip)
    try:
        success = hide_cb("Menu")
    except rospy.ServiceException as exc:
        rospy.logerr("WallframeTooltip: hide service could not hide the tooltip")


  def hide_menu(self):
    self.hide_tooltip_from_menu()
    self.hide()
    self.update()
    self.hidden_ = True
    rospy.logwarn("WallframeMenu: setting to hidden")
    pass

  def show_menu(self):
    self.show_tooltip("Place left hand on right elbow to click")
    self.show()
    self.update()
    self.hidden_ = False
    rospy.logwarn("WallframeMenu: setting to visible")
    pass

  def get_cursor_position_sensor(self):
    pos = self.users_[self.focused_user_id_].translations_mm[8]
    return (pos.x, pos.y)

  def get_cursor_position_mouse(self):
    pos = self.mouse_state
    return self.mouse_state

  def joint_position(self,user,name):
    return user.translations_body_mm[user.frame_names.index(name)]

  # Return whether or not there is a focused user with hands in cursor position.
  def user_has_cursor(self):
    if self.focused_user_id_ == -1:
      return False
    try:
        user = self.users_[self.focused_user_id_]
        right_hand = self.joint_position(user, 'left_hand')
        left_hand = self.joint_position(user, 'right_hand')
        head = self.joint_position(user, 'head')
        torso = self.joint_position(user, 'torso')
        midpoint = 0.75 * torso.y + 0.25 * head.y

        return right_hand.y > midpoint
    # The user id does not exist in the user map
    except IndexError as e:
        return False

  def check_gesture(self, prev_user, current_user):
    # LONG_RIGHT_SWIPE, SHORT_RIGHT_SWIPE, LONG_LEFT_SWIPE, SHORT_LEFT_SWIPE
    # old joint position values of the current user
    if prev_user.wallframe_id == current_user.wallframe_id:
      swipe_gesture = self.check_for_swipe(prev_user, current_user)
      if swipe_gesture:
        return swipe_gesture
    else:
      return None


  def check_for_right_swipe(self, prev_user, current_user):
    dx = self.joint_position(current_user, 'left_hand').x - self.joint_position(prev_user, 'left_hand').x

    if self.state == "RIGHT":
      if -dx > self.X_SHORT_THRES:
        self.state = "IDLE"
    else:
      if dx > self.X_LONG_THRES:
        self.state = "RIGHT"
        return "LONG_RIGHT_SWIPE"

      elif dx > self.X_SHORT_THRES:
        self.state = "RIGHT"
        return "SHORT_RIGHT_SWIPE"
    return None



  def check_for_left_swipe(self, prev_user, current_user):
    dx = self.joint_position(prev_user, 'right_hand').x - self.joint_position(current_user, 'right_hand').x

    if self.state == "LEFT":
      if -dx > self.X_SHORT_THRES:
        self.state = "IDLE"
    else:
      if dx > self.X_LONG_THRES:
        self.state = "LEFT"
        return "LONG_LEFT_SWIPE"
      elif dx > self.X_SHORT_THRES:
        self.state = "LEFT"
        return "SHORT_LEFT_SWIPE"
    return None

  def validate_y_for_swipe(self, prev_hand_y, current_hand_y,user):
    head = self.joint_position(user, 'head')
    torso = self.joint_position(user, 'torso')

    if current_hand_y >= torso.y and current_hand_y <= head.y and abs(current_hand_y - prev_hand_y) <= self.Y_THRES:
      return True
    else:
      return False

  def check_for_swipe(self, prev_user, current_user):
    current_left_hand = self.joint_position(current_user, 'right_hand')
    current_right_hand = self.joint_position(current_user, 'left_hand')
    prev_left_hand = self.joint_position(prev_user, 'right_hand')
    prev_right_hand = self.joint_position(prev_user, 'left_hand')
    # TODO think about what both the two swipe gestures happen together
#    print "Difference y ", prev_left_hand.y - current_left_hand.y
#    print "Difference x ", prev_left_hand.x - current_left_hand.x
#    print "Absolute val y", prev_left_hand.y
#    print "Absolute val x", prev_left_hand.x

    right_gesture = None
    left_gesture = None
    if self.validate_y_for_swipe(prev_right_hand.y, current_right_hand.y,current_user):
      right_gesture = self.check_for_right_swipe(prev_user, current_user)
    if self.validate_y_for_swipe(prev_left_hand.y, current_left_hand.y,current_user):
      left_gesture = self.check_for_left_swipe(prev_user, current_user)
    return left_gesture or right_gesture

  def update_tiles(self):
    if self.run_:

      if self.prev_user and self.current_user :
        gesture = self.check_for_swipe(self.prev_user,self.current_user)
        if(gesture):
          print gesture
        if gesture == "LONG_RIGHT_SWIPE":
          self.tileflow_widget.move_right(2)
        elif gesture == "SHORT_RIGHT_SWIPE":
          self.tileflow_widget.move_right(1)
        elif gesture == "LONG_LEFT_SWIPE":
          self.tileflow_widget.move_left(2)
        elif gesture == "SHORT_LEFT_SWIPE":
          self.tileflow_widget.move_left(1)


        #cursorx, cursory = self.get_cursor_position_sensor()
        #cursor_position = self.convert_workspace([cursorx,cursory])

        #self.tileflow_widget.update_cursor(cursor_position)

      #cursorx, cursory = self.get_cursor_position_mouse()
      #cursor_position = self.convert_workspace([cursorx,cursory])

      #self.tileflow_widget.update_cursor(cursor_position)


  def click(self):
    print "click"

  def run(self):
    self.show()
    self.qt_app_.exec_()
    pass


### MAIN ###
if __name__ == "__main__":
  menu = AppMenu()
  menu.run()
