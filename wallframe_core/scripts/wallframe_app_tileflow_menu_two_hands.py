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
from wallframe_msgs.msg import WallframeAppEvent
from wallframe_extra.msg import WallframeMouseState
# srv
import wallframe_core
from wallframe_core import WallframeAppWidget
from wallframe_image_widget import WallframeImageWidget
from wallframe_core.srv import *
from tileflow import TileflowWidget

class AppMenu(WallframeAppWidget):
  Y_THRES = 50
  X_LONG_THRES = 130
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

    # Indicates whether or not menu responds to user
    self.is_active = False

    # Currently running app or empty for menu
    self.current_app_id = ""

    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

    # ROS
    rospy.init_node('wallframe_app_tileflow_menu', anonymous=True)

    # ---- ROS subscriber ---
    self.user_state_sub_ = rospy.Subscriber("/wallframe/users/state",
                                            WallframeUserArray,
                                            self.user_state_cb)

    self.user_event_sub_ = rospy.Subscriber("/wallframe/users/events",
                                            WallframeUserEvent,
                                            self.user_event_cb)

    self.app_event_sub_ = rospy.Subscriber("/app/event",
                                            WallframeAppEvent,
                                            self.app_event_cb)


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

    ### User Friendly Application names ###
    if rospy.has_param("/wallframe/core/app_ids"):
      self.app_ids_ = rospy.get_param("/wallframe/core/app_ids")
    else:
      rospy.logerr("WallframeAppMenu: parameter [app_ids] not found on server")

    ### get the assets path
    if rospy.has_param("/wallframe/core/tooltip/assets"):
      self.assets_path = rospy.get_param("/wallframe/core/tooltip/assets")
    else:
      rospy.logerr("WallframeAppMenu: parameter [assets] not found on server")

    ### Default App ###
    if rospy.has_param("/wallframe/core/default_app"):
      self.default_app_id_ = rospy.get_param("/wallframe/core/default_app")
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
    #rospy.logwarn("WallframeAppMenu: height percentage set to " + str(self.height_perc_))
    self.height_ = int(self.height_*self.height_perc_)

    self.setWindowTitle("Wallframe Main Menu")
    self.layout_ = QVBoxLayout()
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.show()
    self.resize(self.width_, self.height_)
    self.move(self.x_,self.y_)
    self.setLayout(self.layout_)

    # Init TileflowWidget
    self.init_menu_widget()
    # Timers
    self.ok_timer_ = QtCore.QTimer()
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)
    self.run_ = True
    # Hide and Show Connections
    self.signal_show_.connect(self.show_menu)
    self.signal_hide_.connect(self.hide_menu)

    # Launch the screensaver
    self.load_app(self.default_app_id_)
    self.update_tooltip("tooltip_menu", "", "menu.png")

  def check_ok(self):
    if rospy.is_shutdown():
      self.qt_app_.exit()
    else:
      self.update_tiles()
      pass
    pass


#TODO this function should be in utilies
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


  def init_menu_widget(self):
    # Initialize the help tool tips
    self.left_swipe_tool_tip = WallframeImageWidget(1280, 400, self.assets_path + "/left.png")
    self.right_swipe_tool_tip = WallframeImageWidget(1280, 400, self.assets_path + "/right.png")
    self.click_tool_tip = WallframeImageWidget(1280, 400, self.assets_path + "/click.png")
    # add the tooltip widgets
    self.top_widget = QWidget()
    self.top_widget_layout = QHBoxLayout(self.top_widget)
    self.top_widget_layout.addWidget(self.left_swipe_tool_tip)
    self.top_widget_layout.addWidget(self.click_tool_tip)
    self.top_widget_layout.addWidget(self.right_swipe_tool_tip)
    self.top_widget_layout.setSpacing(0)
    self.top_widget.setFixedHeight(400)
    #self.top_widget.setStyleSheet("border-width: 0;")
    self.top_widget.setAutoFillBackground(True)
    p = self.top_widget.palette()
    p.setColor(self.top_widget.backgroundRole(), Qt.black)
    self.top_widget.setPalette(p)
    self.layout_.addWidget(self.top_widget)
    self.layout_.setSpacing(0)
    res_list = [item[1] + '/menu_icon.png' for item in self.app_paths_.items()]
    self.tileflow_widget = TileflowWidget(self, res_list)
    self.app_menu_items_ = [item[0] for item in self.app_paths_.items()]

    self.layout_.addWidget(self.tileflow_widget)


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

  def activate_menu(self):
    # show the menu
    self.signal_show_.emit()

  def deactivate_menu(self):
    # hide the menu
    self.signal_hide_.emit()

  def app_event_cb(self, msg):
    print "APP EVENT " + msg.app_id + " " + msg.status 
    print "CURRENT APP " + self.current_app_id
    print "MENU ACTIVE " + str(self.is_active)

    if msg.status == "ready":
      if msg.app_id == self.current_app_id:
        self.deactivate_menu()
        self.update_tooltip("tooltip_menu", "", "menu.png")
    elif msg.status == "exit":
      if msg.app_id == self.default_app_id_:
        pass
        # TODO: Restart default app? 

      if msg.app_id == self.current_app_id:
        self.toast_pub_.publish(String('App exited ' + self.app_ids_[msg.app_id]))
        self.is_active = True
        self.current_app_id = ""
        self.activate_menu()

  def terminate_current_app(self):
    if (self.current_app_id != self.default_app_id_ and self.current_app_id):
      self.toast_pub_.publish(String("Closing " + self.app_ids_[self.current_app_id]))
      self.terminate_app(self.current_app_id)
      self.current_app_id = ""

  def user_event_cb(self,msg):
    if self.run_:
      if msg.event_id == 'workspace_event':
        if msg.message == "all_users_left":
          # Go to screen saver
          self.terminate_current_app();
          self.deactivate_menu()
          self.resume_app(self.default_app_id_)
          self.is_active = False
      elif msg.event_id == 'hand_event' and msg.user_id == self.focused_user_id_:
        if msg.message == 'hands_on_head':
          # Go to menu
          self.terminate_current_app()
          self.pause_app(self.default_app_id_);
          self.activate_menu()
          self.is_active = True
        elif msg.message == 'left_elbow_click' and self.is_active and not self.current_app_id:
          # Load the app triggering either a "ready" or "exit" event.
          self.is_active = False
          self.tileflow_widget.click()

  def clicked_on(self, ind):
    app_id = self.app_menu_items_[ind]
    self.load_app(app_id)

  def load_app(self, app_id):
    if (app_id != self.default_app_id_):
        self.update_tooltip("tooltip_menu", "", "loading.gif")
    self.toast_pub_.publish(String('Loading App ' + self.app_ids_[app_id]))

    rospy.wait_for_service('wallframe/core/app_manager/load_app')
    self.current_app_id = app_id

    try:
      self.srv_load_app = rospy.ServiceProxy('wallframe/core/app_manager/load_app',
                                             wallframe_core.srv.load_app)
      ret_success = self.srv_load_app(app_id)

      self.toast_pub_.publish(String(self.app_ids_[app_id] + " running"))
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s" % e)

  # TODO refactor to merge terminate / pause / resume services into one
  def terminate_app(self, app_id):
    rospy.wait_for_service("wallframe/core/app_manager/terminate_app")
    try:
      self.srv_terminate_app = rospy.ServiceProxy('wallframe/core/app_manager/terminate_app',
                                              wallframe_core.srv.terminate_app)
      ret_success = self.srv_terminate_app(app_id)
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s" % e)

  def pause_app(self, app_id):
    rospy.wait_for_service("wallframe/core/app_manager/pause_app")
    try:
      self.srv_pause_app = rospy.ServiceProxy('wallframe/core/app_manager/pause_app',
                                              wallframe_core.srv.pause_app)
      ret_success = self.srv_pause_app(app_id)
      self.current_app_id = ""
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s" % e)

  def resume_app(self, app_id):
    #self.toast_pub_.publish(String("Resuming " + self.app_ids_[app_id]))
    rospy.wait_for_service("wallframe/core/app_manager/resume_app")
    try:
      self.srv_resume_app = rospy.ServiceProxy('wallframe/core/app_manager/resume_app',
                                              wallframe_core.srv.resume_app)
      ret_success = self.srv_resume_app(app_id)
      self.current_app_id = app_id
      #self.toast_pub_.publish(String(self.app_ids_[app_id] + ' Resumed'))
    except rospy.ServiceException, e:
      rospy.logerr("Service call failed: %s" % e)

  def update_tooltip(self, tooltip_name, text, background_path):
    rospy.wait_for_service(tooltip_name + '/update_tooltip')
    update_tooltip_cb = rospy.ServiceProxy(tooltip_name + '/update_tooltip', update_tooltip)
    try:
        success = update_tooltip_cb("Menu", text, background_path)
    except rospy.ServiceException as exc:
        rospy.logerr("WallframeTooltip: [tooltip " + tooltip_name + "] update_tooltip service could not update the text")

  def hide_tooltip_from_menu(self, tooltip_name):
    rospy.wait_for_service(tooltip_name + '/hide_tooltip')
    hide_cb = rospy.ServiceProxy(tooltip_name + '/hide_tooltip', hide_tooltip)
    try:
        success = hide_cb("Menu")
    except rospy.ServiceException as exc:
        rospy.logerr("WallframeTooltip: [tooltip " + tooltip_name + "] hide service could not hide the tooltip")


  def hide_menu(self):
    self.update_tooltip("tooltip_menu", "", "menu.png")
    self.hide()
    self.update()
    rospy.logwarn("WallframeMenu: setting to hidden")
    pass

  def show_menu(self):
    self.hide_tooltip_from_menu("tooltip_menu")
    self.show()
    self.update()
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
    if self.joint_position(prev_user, 'left_hand').x > self.joint_position(prev_user, 'torso').x:
      dx = self.joint_position(current_user, 'left_hand').x - self.joint_position(prev_user, 'left_hand').x

      if self.state == "RIGHT":
        if -dx > self.X_SHORT_THRES:
          self.state = "IDLE"
      elif self.validate_y_for_swipe(self.joint_position(prev_user, 'left_hand').y, self.joint_position(current_user, 'left_hand').y, current_user):
        if dx > self.X_LONG_THRES:
          self.state = "RIGHT"
          return "LONG_RIGHT_SWIPE"

        elif dx > self.X_SHORT_THRES:
          self.state = "RIGHT"
          return "SHORT_RIGHT_SWIPE"
    return None



  def check_for_left_swipe(self, prev_user, current_user):
    if self.joint_position(prev_user, 'right_hand').x < self.joint_position(prev_user, 'torso').x:
      dx = self.joint_position(prev_user, 'right_hand').x - self.joint_position(current_user, 'right_hand').x

      if self.state == "LEFT":
        if -dx > self.X_SHORT_THRES:
          self.state = "IDLE"
      elif self.validate_y_for_swipe(self.joint_position(prev_user, 'right_hand').y, self.joint_position(current_user, 'right_hand').y, current_user):
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
    #current_left_hand = self.joint_position(current_user, 'right_hand')
    #current_right_hand = self.joint_position(current_user, 'left_hand')
    #prev_left_hand = self.joint_position(prev_user, 'right_hand')
    #prev_right_hand = self.joint_position(prev_user, 'left_hand')
    # TODO think about what both the two swipe gestures happen together
    right_gesture = None
    left_gesture = None
    right_gesture = self.check_for_right_swipe(prev_user, current_user)
    left_gesture = self.check_for_left_swipe(prev_user, current_user)
    #if self.validate_y_for_swipe(prev_right_hand.y, current_right_hand.y,current_user):
      #right_gesture = self.check_for_right_swipe(prev_user, current_user)
    #if self.validate_y_for_swipe(prev_left_hand.y, current_left_hand.y,current_user):
      #left_gesture = self.check_for_left_swipe(prev_user, current_user)
    return left_gesture or right_gesture

  def update_tiles(self):
    if self.run_:

      if self.prev_user and self.current_user and self.is_active:
        gesture = self.check_for_swipe(self.prev_user,self.current_user)
        if gesture == "LONG_RIGHT_SWIPE":
          self.tileflow_widget.move_right(2)
        elif gesture == "SHORT_RIGHT_SWIPE":
          self.tileflow_widget.move_right(1)
        elif gesture == "LONG_LEFT_SWIPE":
          self.tileflow_widget.move_left(2)
        elif gesture == "SHORT_LEFT_SWIPE":
          self.tileflow_widget.move_left(1)

  # This is for testing with mouse
  def mouse_state_cb(self, msg):
    if self.run_:
      self.mouse_state = (msg.x, msg.y)


  def run(self):
    self.show()
    self.qt_app_.exec_()
    pass


### MAIN ###
if __name__ == "__main__":
  menu = AppMenu()
  menu.run()
