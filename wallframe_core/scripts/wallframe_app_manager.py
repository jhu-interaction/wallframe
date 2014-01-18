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
# 2) Responds to launch app requests.
# 3) Monitors apps in order to send exit events.
###

import roslib; roslib.load_manifest('wallframe_core')
import rospy, os, sys, glob, fnmatch
import rosgraph.masterapi
import subprocess, thread

from wallframe_msgs.msg import WallframeRequestApp as app_request_msg
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

    self.log_prefix = "App Manager: "

    rospy.init_node('wallframe_app_manager', anonymous=True)

    self.config_parser = ConfigParser.RawConfigParser()
    self.load_application_configs()

    self.app_event_pub = rospy.Publisher("/app/event", app_event_msg)
    self.app_request_pub = rospy.Publisher("/app/request", app_request_msg)
    self.app_request_sub = rospy.Subscriber("/app/request", app_request_msg, self.app_request_cb)    
 
    rospy.loginfo(self.log_prefix + "Running")
    rospy.spin()

    rospy.loginfo(self.log_prefix + "Closing")
    self.clean_up()

  def app_request_cb(self, msg):
    rospy.loginfo(self.log_prefix + "Request %s %s", msg.command, msg.app_id)

    if msg.command == "launch":
      self.launch_app(msg.app_id)
    elif msg.command == "terminate":
      self.terminate_app(msg.app_id)
      
  def send_app_event(self, app_id, status):
    app_event = app_event_msg()
    app_event.status = status
    app_event.app_id = app_id
    self.app_event_pub.publish(app_event)

  def clean_up(self):
    if rospy.has_param("/wallframe/core/available_apps"):
      rospy.delete_param("/wallframe/core/available_apps")
    # TODO: Send kill to all running apps?

  # Launch an app ensuring that only one process with a given app_id is running
  def launch_app(self, app_id):
    launch_name = self.apps[app_id]["launch_name"]
    launch_package = self.apps[app_id]["package_name"]
    launch_args = ['roslaunch', launch_package, launch_name]

    # Kill any existing process with same app_id
    if app_id in self.procs:
      proc = self.procs[app_id]
      if proc.poll() is not None: 
        rospy.logwarn(self.log_prefix + "Killing old %s pid %d", app_id, proc.pid)

        try:
          proc.kill()
          proc.wait()
        except OSError:
          pass

    rospy.loginfo(self.log_prefix + "Launching %s", app_id)

    proc = subprocess.Popen(launch_args)
    self.procs[app_id] = proc

    thread.start_new_thread(self.wait_for_app_exit, (proc, app_id))

  def wait_for_app_exit(self, proc, app_id):
    returncode = proc.wait()
    rospy.loginfo(self.log_prefix + "App exit %s code %d", app_id, returncode)
    self.send_app_event(app_id, "exit")

  # App should terminate itself. Start a thread to kill app if needed.
  def terminate_app(self, app_id):
    if app_id in self.procs:
      proc = self.procs[app_id]
      if proc.poll() is not None:
        thread.start_new_thread(self.ensure_app_terminates, (proc, app_id))

  def ensure_app_terminates(self, proc, app_id):
    time.sleep(30)

    if proc.poll() is not None: 
      rospy.loginfo(self.log_prefix + "Killing %s pid %d because it has not terminated itself.", app_id, proc.pid)

      try:
        proc.kill()
      except OSError:
        pass
    
  # TODO menu.cfg -> app.cfg given how its used
  # TODO Think about app_id. Could not not simply be the ros node name?
  # TODO Look at RosPack. Maybe can just use manifest?

  # find which apps have the menu.cfg file and add the app to the
  # available_apps rosparam
  def load_application_configs(self):
    # find the wallframe_apps directory path using the rosparam
    if rospy.has_param('/wallframe/core/paths/application_path'):
      self.app_path_ = rospy.get_param('/wallframe/core/paths/application_path')
    else:
      rospy.logerr(self.log_prefix + "Application path not found on parameter server")

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

      rospy.logdebug(self.log_prefix + "Package: " + app_package_name + " Launch: " + app_launch_file_name)

      # TODO Probably this is not the best way to do this
      # We do not want to add the screen saver to the available apps for menu
      if app_id != "screensaver":
        available_app_list[app_id] = app_package_path


      app_ids[app_id] = app_name
      launch_file = {"launch_name": app_launch_file_name, "package_name": app_package_name}

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
