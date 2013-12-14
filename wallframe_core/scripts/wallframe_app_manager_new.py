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
# Authors: Yifan Ge, Kelleher Guerin, Vineeta Khatuja
# Johns Hopkins University
###

import roslib; roslib.load_manifest('wallframe_core')
import rospy, os, sys, glob, fnmatch
import rosgraph.masterapi
from ros import roslaunch
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
import subprocess

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from wallframe_msgs.msg import WallframeUser as user_msg
from wallframe_msgs.msg import WallframeUserArray as user_array_msg
from wallframe_msgs.msg import WallframeUserEvent as user_event_msg
from wallframe_msgs.msg import TrackerUser
from wallframe_msgs.msg import TrackerUserArray as tracker_msg

import wallframe_core
import ConfigParser
from wallframe_core.srv import *

# currently we are using only lauch name , package name and active member
# variables TODO clean up
class AppLaunchFile():
  def __init__(self,name,launch_name,pack,launch_path,package_path,active):
    self.launch_file_path_ = launch_path
    self.package_path_ = package_path
    self.name_ = name
    self.launch_name_ = launch_name
    self.package_ = pack
    self.active_ = active

class WallframeAppManager():
  def __init__(self):
    # Member variables
    # this hash contains the appname as key and the launch file object as value
    self.apps_ = {}
    self.active_app_launchers_ = {}
    # Roslaunch
    self.roslaunch_master_ = ROSLaunch()
    # ROS Init
    rospy.init_node('wallframe_app_manager',anonymous=True)
    # ROS Subscribers
#    self.wallframe_event_sub = rospy.Subscriber("/wallframe/events", String, self.wallframe_event_cb)

    # Config parser
    self.config_parser_ = ConfigParser.RawConfigParser()
    # Load Apps
    self.load_application_configs()

    # ROS Services

    self.load_app_srv_ = rospy.Service('app_manager/load_app', wallframe_core.srv.load_app, self.load_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ load_app ]")
    self.close_app_srv_ = rospy.Service('app_manager/close_app', wallframe_core.srv.close_app, self.close_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_app ]")
    self.close_all_apps_srv_ = rospy.Service('app_manager/close_all_apps', wallframe_core.srv.close_all_apps, self.close_all_apps_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_all_apps ]")

    # Running
    rospy.logwarn("WallframeAppManager: Started")
    rospy.spin()

    # Quitting
    rospy.logwarn("WallframeAppManager: Cleaning up running applications")
    self.shutdown_all_apps()
    self.clean_up()
    rospy.logwarn("WallframeAppManager: Finished")

  def load_app_service(self,req):
    message = "WallframeAppManager: Service Call to LOAD APP ["+req.app_name+"]"
    if self.launch_app(req.app_name) == True:
      return "LOAD APP -- SUCCESS"
    else:
      return "LOAD APP -- FAILED -- App not found or could not load"
    pass

  def close_app_service(self,req):
    message = "WallframeAppManager: Service Call to CLOSE APP ["+req.app_name+"]"
    app_full_name = "wallframe_app_"+req.app_name
    if app_full_name in self.active_app_launchers_.keys():
      rospy.logwarn(message + "SUCCESS")
      self.shutdown_app(req.app_name)
      return "CLOSE APP -- SUCCESS"
    else:
      rospy.logwarn(message + "FAILED -- App not running")
      return "CLOSE APP -- FAIL -- App not running"
    pass

  def close_all_apps_service(self,req):
    message = "WallframeAppManager: Service Call to CLOSE ALL APPS -- "
    if len(self.active_app_launchers_) == 0:
      rospy.logwarn(message + "FAILED -- No apps are running")
      return "CLOSE ALL APPS -- FAIL -- No apps are running"
    else:
      self.shutdown_all_apps()
      rospy.logwarn(message + "SUCCESS")
      return "CLOSE ALL APPS -- SUCCESS"
    pass


  def clean_up(self):
    if rospy.has_param("/wallframe/core/available_apps"):
      rospy.delete_param("/wallframe/core/available_apps")
      print("Remaining parameters cleaned up")

  def shutdown_all_apps(self):
    for app_name, app_process in self.active_app_launchers_.items():
      rospy.logwarn("Killing: " + app_name)
      app_process.terminate()
      while app_process.poll() == None:
        pass
      if rospy.has_param("/wallframe/core/apps/running/" + app_name):
        rospy.delete_param("/wallframe/core/apps/running/" + app_name)
      self.apps_[app_name].active_ = False
      rospy.logwarn("WallframeAppManager: App [" + app_name + "] shutdown successfully")
    self.active_app_launchers_.clear()

  def shutdown_app(self, app_name):
    print "Shutting down " + app_name
    app_process  = self.active_app_launchers_[app_name]
    app_process.terminate()
    while app_process.poll() == None:
      pass
    if rospy.has_param("/wallframe/core/apps/running/" + app_name):
      rospy.delete_param("/wallframe/core/apps/running/" + app_name)
    del self.active_app_launchers_[app_name]
    self.apps_[app_name].active_ = False
    rospy.logwarn("WallframeAppManager: App [" + app_name + "] shutdown successfully")

  def launch_app(self, app_name):
    if app_name not in self.apps_.keys():
      rospy.logerr("AppManager App: " + app_name + " not found!")
      return False
    launch_name = self.apps_[app_name].launch_name_
    launch_package = self.apps_[app_name].package_
    launch_args = ['roslaunch', launch_package, launch_name]

    process = subprocess.Popen(launch_args)
    # (stdoutdata, stde\r\rdata) = p\rocess.communicate()
    self.active_app_launchers_[app_name] = process

    self.apps_[app_name].active_ = True

    rospy.logwarn("Launching " + app_name)
    # \rospy.set_pa\ram("/wallf\rame/co\re/apps/\running/" + app_name, self.apps_[app_name])
    rospy.set_param("/wallframe/core/apps/running/" + app_name, {})
    return True

  # find which apps have the menu.cfg file and add the app to the
  # available_apps rosparam
  def load_application_configs(self):
    # find the wallframe_apps directory path using the rosparam
    if rospy.has_param('/wallframe/core/paths/application_path'):
      self.app_path_ = rospy.get_param('/wallframe/core/paths/application_path')
    else:
      rospy.logerr("WallframeAppManager: application path not found on parameter server")

    rospy.logwarn("WallframeAppManager: Loading Applications from[" + self.app_path_ + "]")

    available_app_list = {}
    for config_full_path in self.find_files(self.app_path_, "menu.cfg"):
      print config_full_path
      self.config_parser_.read(config_full_path)
      app_name = self.config_parser_.get("app", "name")
      app_launch_path = self.config_parser_.get("app", "launch")
      app_package_path = os.path.dirname(config_full_path)
      app_package_name = os.path.basename(app_package_path)
      abs_app_launch_path = os.path.join(app_package_path, app_launch_path)
      app_launch_file_name = os.path.basename(app_launch_path)
      available_app_list[app_name] = app_package_path

      # launch_file = AppLaunchFile(app_name, app_launch_file_name, app_package_name, abs_app_launch_path, app_package_path, False)
      launch_file = AppLaunchFile(None, app_launch_file_name, app_package_name, None, None, False)
      rospy.logwarn("Loading app: " + app_name + ": " + app_launch_file_name + " -> " + abs_app_launch_path)
      rospy.logwarn("Package: " + app_package_name + " Launch: " + app_launch_file_name)
      self.apps_[app_name] = launch_file

    rospy.set_param("/wallframe/core/available_apps", available_app_list)

  #this function returns a generator of absolute file paths of the given file name
  def find_files(self, directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

# MAIN
if __name__ == '__main__':
  m = WallframeAppManager()
