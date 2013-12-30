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


from wallframe_msgs.msg import WallframeRequestApp as request_app_msg

import wallframe_core
import ConfigParser
from wallframe_core.srv import *
import signal

class WallframeAppManager():
  def __init__(self):
    # Member variables
    # this hash contains the appname as key and the launch file object as value
    self.apps = {}

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

    # load the applications
    self.load_app_srv_ = rospy.Service('app_manager/load_app', wallframe_core.srv.load_app, self.load_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ load_app ]")

    # terminate the app
    self.terminate_app_srv_ = rospy.Service('app_manager/terminate_app', wallframe_core.srv.terminate_app, self.terminate_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ terminate_app ]")

    # pause the app
    self.pause_app_srv_ = rospy.Service('app_manager/pause_app', wallframe_core.srv.pause_app, self.pause_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ pause_app ]")

    ## resume the app
    self.resume_app_srv_ = rospy.Service('app_manager/resume_app', wallframe_core.srv.resume_app, self.resume_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ resume_app ]")


    # ROS Publishers
    self.request_app_pub_ = rospy.Publisher("/app/request",request_app_msg)

    # Running
    rospy.logwarn("WallframeAppManager: Started")
    rospy.spin()

    # Quitting
    rospy.logwarn("WallframeAppManager: Cleaning up running applications")
    self.clean_up()
    rospy.logwarn("WallframeAppManager: Finished")

  def load_app_service(self, req):
    rospy.loginfo("WallframeAppManager: Service Call to LOAD APP ["+req.app_id+"]")
    return self.load_app(req.app_id)


  def terminate_app_service(self, req):
    rospy.loginfo("WallframeAppManager: Service Call to TERMINATE ["+req.app_id+"]")
    return self.request_app(req.app_id, "terminate")


    rospy.logerr("WallFrameAppManager: Terminate service call for inactive app ["+req.app_id+"]")
    return False
    pass

  def pause_app_service(self, req):
    rospy.loginfo("WallframeAppManager: Service Call to PAUSE ["+req.app_id+"]")
    return self.request_app(req.app_id, "pause")

  def resume_app_service(self, req):
    rospy.loginfo("WallframeAppManager: Service Call to RESUME ["+req.app_id+"]")
    return self.request_app(req.app_id, "resume")

  def request_app(self, app_id, command):
    app_request = request_app_msg()
    app_request.command = command
    app_request.app_id = app_id
    self.request_app_pub_.publish(app_request)
    return True



  def clean_up(self):
    if rospy.has_param("/wallframe/core/available_apps"):
      rospy.delete_param("/wallframe/core/available_apps")
      print("Remaining parameters cleaned up")

  def load_app(self, app_id):
    launch_name = self.apps[app_id]["launch_name"]
    launch_package = self.apps[app_id]["package_name"]
    launch_args = ['roslaunch', launch_package, launch_name]

    process = subprocess.Popen(launch_args)

    rospy.logwarn("Launching " + app_id)
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
    app_ids = {}
    for config_full_path in self.find_files(self.app_path_, "menu.cfg"):
      self.config_parser.read(config_full_path)
      app_id = self.config_parser.get("app", "id")
      app_name = self.config_parser.get("app", "name")
      app_launch_path = self.config_parser.get("app", "launch")
      app_package_path = os.path.dirname(config_full_path)
      app_package_name = os.path.basename(app_package_path)
      app_launch_file_name = os.path.basename(app_launch_path)

      # Probably this is not the best way to do this
      # We do not want to add the screen saver to the available apps for menu
      if app_id != "screensaver":
        available_app_list[app_id] = app_package_path


      app_ids[app_id] = app_name
      launch_file = {"launch_name": app_launch_file_name, "package_name": app_package_name}
      rospy.logwarn("Package: " + app_package_name + " Launch: " + app_launch_file_name)
      self.apps[app_id] = launch_file

    rospy.set_param("/wallframe/core/available_apps", available_app_list)
    rospy.set_param("/wallframe/core/app_ids", app_ids)
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
