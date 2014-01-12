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
# Authors: Yifan Ge, Vineeta Khatuja
# Johns Hopkins University
#
# AppManager
# 1) Reads the menu.cfg file and sets the ros parameters
# 2) It provides the load , terminate , pause and resume app services
# These services are used by the menu
###

import roslib; roslib.load_manifest('wallframe_core')
import rospy, os, sys, glob, fnmatch
import rosgraph.masterapi
from ros import roslaunch
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
import subprocess, thread

from wallframe_msgs.msg import WallframeRequestApp as request_app_msg
from wallframe_msgs.msg import WallframeAppEvent as app_event_msg

import wallframe_core
import ConfigParser
from wallframe_core.srv import *

class WallframeAppManager():
  def __init__(self):
    # app_id -> (config param -> config value) 
    self.apps = {}

    # app_id -> most recently launched process
    self.procs = {}

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
    self.app_event_pub_ = rospy.Publisher("/app/event", app_event_msg)

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
    # TODO Monitor and kill if it takes a long time to quit
    return self.request_app(req.app_id, "terminate")

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

  def app_event(self, app_id, status):
    app_event = app_event_msg()
    app_event.status = status
    app_event.app_id = app_id
    self.app_event_pub_.publish(app_event)

  def clean_up(self):
    if rospy.has_param("/wallframe/core/available_apps"):
      rospy.delete_param("/wallframe/core/available_apps")

  # Ensures that only one process with a given app_id is running
  def load_app(self, app_id):
    launch_name = self.apps[app_id]["launch_name"]
    launch_package = self.apps[app_id]["package_name"]
    launch_args = ['roslaunch', launch_package, launch_name]

    # Kill any existing process with same app_id
    if app_id in self.procs:
      proc = self.procs[app_id]
      if proc.poll() is not None: 
        rospy.logwarn("Killing old " + app_id + " pid " + str(proc.pid))
        try:
          proc.kill()
          proc.wait()
        except OSError:
          pass

    rospy.logwarn("Launching " + app_id)

    proc = subprocess.Popen(launch_args)
    self.procs[app_id] = proc

    thread.start_new_thread(self.wait_for_app_exit, (proc, app_id))
    return True

  def wait_for_app_exit(self, proc, app_id):
    returncode = proc.wait()
    rospy.logwarn("App exit " + app_id + " code " + str(returncode))
    self.app_event(app_id, "exit")
    
  # TODO menu.cfg -> app.cfg given how its used
  # TODO Think about app_id. Could not not simply be the ros node name?
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

      # TODO Probably this is not the best way to do this
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
