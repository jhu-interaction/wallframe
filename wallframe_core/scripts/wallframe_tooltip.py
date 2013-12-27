#!/usr/bin/env python
################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the Johns Hopkins University nor the names of its
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
################################################################################
__author__ = "Kelleher Guerin"
__email__ = "futureneer@gmail.com"
__copyright__ = "2013, The Johns Hopkins University"
__license__ = "BSD"
################################################################################

import roslib; roslib.load_manifest('wallframe_core')
import rospy,sys
### PySide ###
import PySide
from PySide.QtGui import *
from PySide.QtCore import *
from PySide import QtCore

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from wallframe_msgs.msg import WallframeUser
from wallframe_msgs.msg import WallframeUserArray
from wallframe_msgs.msg import WallframeUserEvent
from wallframe_msgs.msg import TrackerUser
from wallframe_msgs.msg import TrackerUserArray as tracker_msg

import wallframe_core
from wallframe_core.srv import *
import signal

################################################################################
class WallframeTooltip(QWidget):
    SIGNAL_HIDE = QtCore.Signal()
    SIGNAL_SHOW = QtCore.Signal()
    def __init__(self, node_name):
        QWidget.__init__(self)
        self.node_name = node_name
        self.wall_height = rospy.get_param("/wallframe/core/params/height", 3197)
        self.wall_width = rospy.get_param("/wallframe/core/params/width", 5760)

        self.x = rospy.get_param("/wallframe/core/params/x", 1680)
        self.y = rospy.get_param("/wallframe/core/params/y", 24)

        self.height = rospy.get_param("/wallframe/" + self.node_name + "/params/height_percentage", 0.08) * self.wall_height
        self.width = rospy.get_param("/wallframe/" + self.node_name + "/params/width_percentage", 0.2) * self.wall_width

        self.x_position = rospy.get_param("/wallframe/"+ self.node_name + "/params/x_percentage", 0.7) * self.wall_width + self.x
        self.y_position = rospy.get_param("/wallframe/" + self.node_name + "/params/y_percentage", 0.05) * self.wall_height + self.y
        self.name = rospy.get_param("/wallframe/" + self.node_name + "/params/name")
        self.setStyleSheet("background-color:#ffffff;color:#222222")
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint )
        # the tool tip always stays on top
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)
        self.move(self.x_position, self.y_position)
        self.show()
        #self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
        self.setAutoFillBackground(True)
        self.setWindowOpacity(.9)
        self.resize(self.width, self.height)
        print "Tool tip height" , self.height , "width " , self.width
        print "xpos " ,  self.x_position , " ypos" , self.y_position


        self.text_label = QLabel('Welcome')
        layout = QHBoxLayout()
        layout.addWidget(self.text_label)
        self.text_label.setStyleSheet("QLabel { color : blue; font-size: 30px; }")
        self.setLayout(layout)


        # ROS Services

        self.update_text_srv = rospy.Service(self.name + '/update_text', wallframe_core.srv.update_text, self.update_text_service)
        rospy.logwarn("WallframeTooltip: Service Ready [" + self.name + "/update_text ]")
        self.hide_tooltip_srv = rospy.Service(self.name + '/hide_tooltip', wallframe_core.srv.hide_tooltip, self.hide_tooltip_service)
        rospy.logwarn("WallframeTooltip: Service Ready [" + self.name + "/hide_tooltip ]")

        self.SIGNAL_HIDE.connect(self.hide_tooltip)
        self.SIGNAL_SHOW.connect(self.show_tooltip)
        # Running
        rospy.logwarn("WallframeTooltip: Started")
    def hide_tooltip(self):
        self.hide()
    def show_tooltip(self):
        self.show()
    def update_text_service(self, request):
        message = "WallframeTooltip: Service Call to update the text ["+request.app_name+"]"
        print message
        self.text_label.setText(request.text)
        self.SIGNAL_SHOW.emit()
        return True

    def hide_tooltip_service(self, request):
        message = "WallframeTooltip: Service Call to hide ["+request.app_name+"]"
        print message
        self.SIGNAL_HIDE.emit()
        return True
# MAIN
if __name__ == '__main__':
    try:
        node_name = sys.argv[1]
    except:
        node_name = "wallframe_tooltip"
    rospy.init_node(node_name, anonymous=True)
    app = QApplication(sys.argv)
    tooltip = WallframeTooltip(node_name)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    # Running
    app.exec_()
    # Done
    rospy.logwarn('WallframeInfobar: Finished')
