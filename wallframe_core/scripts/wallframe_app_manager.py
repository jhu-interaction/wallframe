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
import signal

class WallframeAppManager():
  def __init__(self):
    # Member variables
    # this hash contains the appname as key and the launch file object as value
    self.apps = {}
    self.active_apps = {}
    # Roslaunch
    self.roslaunch_master_ = ROSLaunch()
    # ROS Init
    rospy.init_node('wallframe_app_manager', anonymous=True)
    # ROS Subscribers
#    self.wallframe_event_sub = rospy.Subscriber("/wallframe/events", String, self.wallframe_event_cb)

    # Config parser
    self.config_parser = ConfigParser.RawConfigParser()
    # Load Apps
    self.load_application_configs()

    # ROS Services

    self.load_app_srv_ = rospy.Service('app_manager/load_app', wallframe_core.srv.load_app, self.load_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ load_app ]")
    self.close_app_srv_ = rospy.Service('app_manager/close_app', wallframe_core.srv.close_app, self.close_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_app ]")
    self.close_all_apps_srv_ = rospy.Service('app_manager/close_all_apps', wallframe_core.srv.close_all_apps, self.close_all_apps_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_all_apps ]")
    self.pause_app_srv = rospy.Service('app_manager/pause_app', wallframe_core.srv.pause_app, self.pause_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ pause_app ]")

    # Running
    rospy.logwarn("WallframeAppManager: Started")
    rospy.spin()

    # Quitting
    rospy.logwarn("WallframeAppManager: Cleaning up running applications")
    self.shutdown_all_apps()
    self.clean_up()
    rospy.logwarn("WallframeAppManager: Finished")

  def load_app_service(self, req):
    message = "WallframeAppManager: Service Call to LOAD APP ["+req.app_name+"]"
    if self.launch_app(req.app_name, req.default) == True:
      return "LOAD APP -- SUCCESS"
    else:
      return "LOAD APP -- FAILED -- App not found or could not load"
    pass

  def close_app_service(self, req):
    message = "WallframeAppManager: Service Call to CLOSE APP ["+req.app_name+"]"
    if req.app_name in self.active_apps.keys():
      rospy.logwarn(message + "SUCCESS")
      self.shutdown_app(req.app_name)
      return "CLOSE APP -- SUCCESS"
    else:
      rospy.logwarn(message + "FAILED -- App not running")
      return "CLOSE APP -- FAIL -- App not running"

  def pause_app_service(self, req):
    message = "WallframeAppManager: Service Call to CLOSE APP [" + req.app_name + "]"
    if req.app_name in self.active_apps.keys():
      self.pause_app(req.app_name)
      return "PAUSE APP -- SUCCESS"
    else:
      rospy.logwarn(message + "FAILED -- APP not running")
      return "PAUSE APP -- FAIL App not running"

  def continue_app_service(self, req):
    message = "WallframeAppManager: Service Call to CONTINUE APP [" + req.app_name + "]"
    if req.app_name in self.active_apps.keys():
      self.continue_app(req.app_name)
      return "CONTINUE APP -- SUCCESS"
    else:
      rospy.logwarn(message + "FGAILED -- APP not running")
      return "CONTINUE APP -- FAIL App not running"

  def close_all_apps_service(self,req):
    message = "WallframeAppManager: Service Call to CLOSE ALL APPS -- "
    if len(self.active_apps) == 0:
      rospy.logwarn(message + "FAILED -- No apps are running")
      return "CLOSE ALL APPS -- FAIL -- No apps are running"
    else:
      self.shutdown_all_apps()
      rospy.logwarn(message + "SUCCESS")
      return "CLOSE ALL APPS -- SUCCESS"


  def clean_up(self):
    if rospy.has_param("/wallframe/core/available_apps"):
      rospy.delete_param("/wallframe/core/available_apps")
      print("Remaining parameters cleaned up")

  def shutdown_all_apps(self):
    for app_name, app in self.active_apps.keys():
      if not app["default"]:
        self.shutdown_app(app_name)
      else:
        self.continue_app(app_name)
        app["paused"] = False

  def shutdown_app(self, app_name):
    print "Shutting down " + app_name
    app_process  = self.active_apps[app_name]["process"]
    app_process.terminate()
    while app_process.poll() == None:
      pass
    del self.active_apps[app_name]
    rospy.logwarn("WallframeAppManager: App [" + app_name + "] shutdown successfully")

  def pause_app(self, app_name):
    print "Pausing " + app_name
    if not self.active_apps[app_name]["paused"]:
      app_process = self.active_apps[app_name]["process"]
      app_process.send_signal(signal.SIGSTOP)
      self.active_apps[app_name]["paused"] = True
      rospy.logwarn("WallframeAppManager: App [" + app_name + "] paused successfully")
    else:
      rospy.logwarn("WallframeAppManager: App [" + app_name + "] is already paused")


  def continue_app(self, app_name):
    print "Continuing " + app_name
    if self.active_apps[app_name]["paused"]:
      app_process = self.active_apps[app_name]["process"]
      app_process.send_signal(signal.SIGCONT)
      rospy.logwarn("WallframeAppManager: App [" + app_name + "] continued successfully")
      self.active_apps[app_name]["paused"] = False
    else:
      rospy.logwarn("WallframeAppManager: App [" + app_name + "] is not paused")

  def launch_app(self, app_name, default=False):
    for app_name, app in self.active_apps.keys():
      if app["default"] == True:
        self.pause_app(app_name)
    if app_name not in self.apps.keys():
      rospy.logerr("AppManager App: " + app_name + " not found!")
      return False
    launch_name = self.apps[app_name]["launch_name"]
    launch_package = self.apps[app_name]["package_name"]
    launch_args = ['roslaunch', launch_package, launch_name]

    process = subprocess.Popen(launch_args)

    self.active_apps[app_name] = {"process": process, "default": default, "paused": False}

    rospy.logwarn("Launching " + app_name)
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
      self.config_parser.read(config_full_path)
      app_name = self.config_parser.get("app", "name")
      app_launch_path = self.config_parser.get("app", "launch")
      app_package_path = os.path.dirname(config_full_path)
      app_package_name = os.path.basename(app_package_path)
      app_launch_file_name = os.path.basename(app_launch_path)
      available_app_list[app_name] = app_package_path

      launch_file = {"launch_name": app_launch_file_name, "package_name": app_package_name}
      rospy.logwarn("Package: " + app_package_name + " Launch: " + app_launch_file_name)
      self.apps[app_name] = launch_file

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
