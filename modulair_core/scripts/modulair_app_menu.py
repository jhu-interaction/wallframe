#!/usr/bin/env python
"""
  Modulair App_ Menu
  First prototype of modulair_app_menu. Uses Model-View/Controller approach.
  View and controller are combined because Qt signal/slots intrinsically do not cater
  to a separated view and controller.
  @author Andy Tien, Kel Guerin, Zihan Chen
"""
# ROS import
import roslib; roslib.load_manifest('modulair_core')
import rospy
from std_msgs.msg import String
# system import
from math import fabs
import os, collections, sys, math
# PySide import
from PySide.QtGui import * 
from PySide.QtCore import *
from PySide import QtCore
# modulair import
# msg
from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
# srv
import modulair_core
from modulair_core.srv import *


class ModulairCursor(QWidget):
  def __init__(self,image,image_alt,parent):
    super(ModulairCursor,self).__init__(parent)
    self.w_ = 350
    self.h_ = 350
    self.resize(self.w_,self.h_)
    self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
    self.setStyleSheet("background:transparent;")
    self.label_ = QLabel(self)
    self.label_alt_ = QLabel(self)

    self.label_.setPixmap(image)
    self.label_.setAutoFillBackground(True)
    self.label_.setScaledContents(True)
    self.label_.setFixedSize(self.w_,self.h_)
    self.label_.setAttribute(QtCore.Qt.WA_TranslucentBackground)
    self.label_.setStyleSheet("background:transparent;")
    self.label_.move(0,0)
    self.label_.show()    

    self.label_alt_.setPixmap(image_alt)
    self.label_alt_.setAutoFillBackground(True)
    self.label_alt_.setScaledContents(True)
    self.label_alt_.setFixedSize(self.w_,self.h_)
    self.label_alt_.setAttribute(QtCore.Qt.WA_TranslucentBackground)
    self.label_alt_.setStyleSheet("background:transparent;")
    self.label_alt_.move(0,0)
    self.label_alt_.hide()

  def click(self):
    print "click"
    self.label_.hide()
    self.label_alt_.show()
    self.update()
    QTimer.singleShot(250, self.reset)

  def reset(self):
    self.label_.show()
    self.label_alt_.hide()
    self.update()

  def set_position(self,pos):
    self.move(pos[0]-self.w_/2, pos[1]-self.h_/2)
    pass

class ModularMenu(QWidget):
  signal_hide_ = QtCore.Signal()
  signal_show_ = QtCore.Signal()
  signal_click_ = QtCore.Signal()
  # constructor
  def __init__(self):
    self.qt_app_ = QApplication(sys.argv)
    QWidget.__init__(self)

    # member variables
    self.current_users_ = []     # list of users
    self.users_ = {}             # dict: (modulair_id, user)
    self.num_users_ = 0          # total num of users
    self.focused_user_id_ = -1   # focused user
    self.app_menu_items_ = {}         # dict: (app_name, qwidget)
    self.hidden_ = False
    self.run_ = False

    # ROS
    rospy.init_node('modulair_app_menu', anonymous=True)

    # ---- ROS subscriber ---
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state",
                                            ModulairUserArray,
                                            self.user_state_cb)

    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events",
                                            ModulairUserEvent,
                                            self.user_event_cb)
    self.toast_pub_ = rospy.Publisher("/modulair/info/toast", String)
    
    # ---- ROS get params -----
    # height
    if rospy.has_param("/modulair/core/params/height"):
      self.height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr("ModulairAppMenu: parameter [height] not found on server")
    # width
    if rospy.has_param("/modulair/core/params/width"):
      self.width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr("ModulairAppMenu: parameter [width] not found on server")
    ### x ###
    if rospy.has_param("/modulair/core/params/x"):
      self.x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr("ModulairAppMenu: parameter [x] not found on server")
    ### y ###
    if rospy.has_param("/modulair/core/params/y"):
      self.y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr("ModulairAppMenu: parameter [y] not found on server")
    ### border scale ###
    if rospy.has_param("/modulair/menu/params/border_scale"):
      self.border_scale_ = rospy.get_param("/modulair/menu/params/border_scale")
    else:
      rospy.logerr("ModulairAppMenu: parameter [border_scale] not found on server")
    self.border_ = int(self.width_ * self.border_scale_)
    ### Cursor Icon ###
    if rospy.has_param("/modulair/menu/params/cursor_path"):
      self.cursor_path_ = rospy.get_param("/modulair/menu/params/cursor_path")
    else:
      rospy.logerr("ModulairAppMenu: parameter [cursor_path] not found on server")
    ### Cursor AltIcon ###
    if rospy.has_param("/modulair/menu/params/cursor_path_alt"):
      self.cursor_path_alt_ = rospy.get_param("/modulair/menu/params/cursor_path_alt")
    else:
      rospy.logerr("ModulairAppMenu: parameter [cursor_path_alt] not found on server")
    ### Background Image ###
    if rospy.has_param("/modulair/menu/params/background_path"):
      self.background_path_ = rospy.get_param("/modulair/menu/params/background_path")
    else:
      rospy.logerr("ModulairAppMenu: parameter [background_path] not found on server")
    ### Application Locations ###
    if rospy.has_param("/modulair/core/available_apps"):
      self.app_paths_ = rospy.get_param("/modulair/core/available_apps")
    else:
      rospy.logerr("ModulairAppMenu: parameter [available_apps] not found on server")
    ### Default App ###
    if rospy.has_param("/modulair/core/default_app"):
      self.default_app_name_ = rospy.get_param("/modulair/core/default_app")
    else:
      rospy.logerr("ModulairAppMenu: parameter [default_app] not found on server")
    ### Go to screensaver ###
    if rospy.has_param("/modulair/menu/params/screensaver"):
      self.screensaver_ = rospy.get_param("/modulair/menu/params/screensaver")
    else:
      rospy.logerr("ModulairAppMenu: parameter [screensaver] not found on server")
    if self.screensaver_ == True:
      rospy.logwarn("ModulairAppMenu: Configured to SHOW screensaver app when all users leave")
    else:
      rospy.logwarn("ModulairAppMenu: Configured to NOT default to screensaver app when all users leave")
    ### Workspace Limits ###
    if rospy.has_param("/modulair/menu/params/workspace_size"):
      self.workspace_limits_ = rospy.get_param("/modulair/menu/params/workspace_size")
    else:
      rospy.logerr("ModulairAppMenu: parameter [workspace_size] not found on server")
    ### Y Offset ###
    if rospy.has_param("/modulair/menu/params/y_offset"):
      self.y_offset_ = rospy.get_param("/modulair/menu/params/y_offset")
    else:
      rospy.logerr("ModulairAppMenu: parameter [y_offset] not found on server")
    ### Height Scaling ###
    if rospy.has_param("/modulair/menu/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/modulair/menu/params/height_percentage")
    else:
      rospy.logerr("ModulairAppMenu: parameter [height_percentage] not found on server")
    rospy.logwarn("ModulairAppMenu: height percentage set to " + str(self.height_perc_))
    self.height_ = int(self.height_*self.height_perc_)

    self.bg = QWidget()
    self.bg.resize(self.width_, self.height_)
    self.bg.move(self.x_,self.y_)
    self.bg.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.bg.show()
    self.bgl = QLabel(self.bg)
    self.bgl.move(0,0)
    self.bgl.resize(self.width_,self.height_)
    self.bgl.setScaledContents(True)
    self.bgl.setAutoFillBackground(True)
    self.bgl.setPixmap(self.background_path_)
    self.bgl.show()
    
    # Get app list
    self.app_list_ = self.app_paths_.keys()
    rospy.logwarn("ModulairMenu: found " + str(len(self.app_list_)) + " applications")

    self.grid_set_up_ = False
    self.setup_grid()
    self.setWindowTitle("Modulair Main Menu")
    self.gridLayout_ = QGridLayout() 
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.resize(self.width_, self.height_)
    self.move(self.x_,self.y_)
    self.setLayout(self.gridLayout_)
    self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
    # self.setStyleSheet("background:transparent;")

    self.background_ = QLabel(self)
    self.background_.move(0,0)
    self.background_.resize(self.width_,self.height_)
    self.background_.setScaledContents(True)
    self.background_.setAutoFillBackground(True)
    self.background_.setPixmap(self.background_path_)
    self.background_.show()

    # Create App Widgets
    self.assignWidgets() # create widget
    # Timers
    self.ok_timer_ = QtCore.QTimer()
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)
    # Cursor
    self.cursor_ = ModulairCursor(self.cursor_path_,self.cursor_path_alt_,self)
    self.cursor_.set_position([self.width_/2,self.height_/2])
    self.cursor_.show()
    self.run_ = True
    # Hide and Show Connections
    self.signal_show_.connect(self.show_menu)
    self.signal_hide_.connect(self.hide_menu)
    self.signal_click_.connect(self.cursor_.click)

  def check_ok(self):
    if rospy.is_shutdown():
      self.qt_app_.exit()
    else:
      self.update_cursor()      
    pass

  def setup_grid(self):
    self.max_x_ = 3
    self.max_y_ = 2
    rospy.logwarn('ModulairMenu:  Grid size is '+str(self.max_x_)+' (w) by , '+str(self.max_y_)+' (h)')
    self.cur_ind_x_ = 0 
    self.cur_ind_y_ = 0
    self.grid_set_up_ = True

  def next_pos(self):
    # check if grid ind has been set
    if not self.grid_set_up_:
      self.setup_grid()
    ind_x = self.cur_ind_x_
    ind_y = self.cur_ind_y_
    # change line if necessary
    self.cur_ind_y_ += 1
    if self.cur_ind_y_ == self.max_y_:
      self.cur_ind_y_ = 0
      self.cur_ind_x_ += 1
    # return current grid index
    return ind_x, ind_y

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

  def assignWidgets(self):
    self.app_menu_items_.clear()
    for app, app_path in self.app_paths_.items():
      label = QLabel(app,self)
      image = app_path + '/menu_icon.png'
      rospy.logwarn('ModulairMenu:  Adding button for '+app+" app.")
      label.setPixmap(image)
      label.setAutoFillBackground(True)
      label.setScaledContents(True)
      label.setFixedSize((self.width_/(16.0/4.0))-self.border_, (self.height_/(9.0/4.0))-self.border_)
      label.setAttribute(QtCore.Qt.WA_TranslucentBackground)
      label.setStyleSheet("background:transparent;")
      label.show()
      nextx, nexty = self.next_pos()
      self.gridLayout_.addWidget(label, nextx, nexty )
      self.app_menu_items_[app] = label

  def user_state_cb(self, msg):
    if self.run_:
      self.current_users_ = msg.users
      self.num_users_ = len(self.current_users_)
      self.users_.clear()
      self.focused_user_id_ = -1
      for user in self.current_users_:
        if user.focused == True:
          self.focused_user_id_ = user.modulair_id
        self.users_[user.modulair_id] = user
    pass
  
  def user_event_cb(self, msg):
    if self.run_:
      ### Workspace Events ###
      if msg.event_id == 'workspace_event':
        if self.screensaver_:
          if msg.message == 'all_users_left':
            rospy.logdebug("ModulairMenu: ALL_USERS_LEFT received, should start default app")
            self.toast_pub_.publish(String('Launching Screensaver'))
            rospy.wait_for_service('modulair/core/app_manager/close_all_apps')
            try:
              self.srv_close_all_apps = rospy.ServiceProxy('modulair/core/app_manager/close_all_apps',
                                                     modulair_core.srv.close_all_apps)
              ret_success = self.srv_close_all_apps('none')
              # If close all apps is successful, hide menu and run default app
              self.signal_hide_.emit()
              rospy.wait_for_service('modulair/core/app_manager/load_app')
              try:
                self.srv_load_app = rospy.ServiceProxy('modulair/core/app_manager/load_app',
                                                       modulair_core.srv.load_app)
                ret_success = self.srv_load_app(self.default_app_name_)
                self.toast_pub_.publish(String('Screensaver Running'))
              except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException, e:
              rospy.logerr("Service call failed: %s" % e)
        else:
          if msg.message == 'all_users_left':
            rospy.logdebug("ModulairMenu: ALL_USERS_LEFT received, should close app and show menu")
            self.toast_pub_.publish(String('Closing Apps'))
            rospy.wait_for_service('modulair/core/app_manager/close_all_apps')
            try:
              self.srv_close_all_apps = rospy.ServiceProxy('modulair/core/app_manager/close_all_apps',
                                                     modulair_core.srv.close_all_apps)
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
          rospy.logdebug("ModulairMenu: HANDS_HEAD received, should resume menu")
          self.toast_pub_.publish(String('Closing All Apps'))
          rospy.wait_for_service('modulair/core/app_manager/close_all_apps')
          try:
            self.srv_close_all_apps = rospy.ServiceProxy('modulair/core/app_manager/close_all_apps',
                                                   modulair_core.srv.close_all_apps)
            ret_success = self.srv_close_all_apps('none')
            print ret_success
            self.signal_show_.emit()
            self.toast_pub_.publish(String('Apps Closed'))
          except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        # Click to start app  
        if all( [ msg.message == 'left_elbow_click', 
                  self.current_app_name_ != "NONE"] ):
          self.signal_click_.emit()
          if self.hidden_ == False:
            print msg.message
            rospy.logwarn("ModulairMenu: LEFT_ELBOW_CLICK received, let's launch app")
            self.toast_pub_.publish(String('Loading App ' + self.current_app_name_))
            rospy.wait_for_service('modulair/core/app_manager/load_app')
            try:
              self.srv_load_app = rospy.ServiceProxy('modulair/core/app_manager/load_app',
                                                     modulair_core.srv.load_app)
              ret_success = self.srv_load_app(self.current_app_name_)
              print ret_success
              self.signal_hide_.emit()
              self.toast_pub_.publish(String(self.current_app_name_ + ' Running'))
            except rospy.ServiceException, e:
              rospy.logerr("Service call failed: %s" % e)
        elif all( [ msg.message == 'left_elbow_click', 
                  self.current_app_name_ == "NONE"] ):
          self.signal_click_.emit()

  def hide_menu(self):
    self.hide()
    self.update()
    self.hidden_ = True
    rospy.logwarn("ModulairMenu: setting to hidden")
    pass

  def show_menu(self):
    self.show()
    self.update()
    self.hidden_ = False
    rospy.logwarn("ModulairMenu: setting to visible")
    pass

  def update_cursor(self):
    if self.run_:
      if self.focused_user_id_ != -1:
        cursorx = self.users_[self.focused_user_id_].translations_mm[8].x
        cursory = self.users_[self.focused_user_id_].translations_mm[8].y
        cursor_position = self.convert_workspace([cursorx,cursory])
        self.cursor_.set_position(cursor_position)
        # Update which app is under cursor (mouse)
        self.current_app_name_ = "NONE"
        for appname, appwidget in self.app_menu_items_.items():
          if appwidget.geometry().contains(cursor_position[0],cursor_position[1]):
            self.current_app_name_ = appname
      
  def run(self):
    self.show()
    self.qt_app_.exec_()
    pass


### MAIN ###
if __name__ == "__main__":
  menu = ModularMenu()
  menu.run() 
