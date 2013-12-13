#!/usr/bin/env python
import roslib; roslib.load_manifest('wallframe_extra')
import rospy
from std_msgs.msg import Float32, String
from PySide import QtGui, QtCore
import sys
from wallframe_extra.msg import WallframeMouseState
class WallTrackpad(QtGui.QWidget):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.__pub = rospy.Publisher("/wallframe/extra/mousestate", WallframeMouseState)
        rospy.init_node("wallframe_extra_mouse_state")
        self.__layout = QtGui.QHBoxLayout()
        self.setLayout(self.__layout)
        self.setFixedSize(300, 300)
        self.setWindowTitle("Trackpad")
        self.setMouseTracking(True)


    def mouseMoveEvent(self, event):
        self.__pub.publish(WallframeMouseState(event.x(), event.y()))

if __name__ == "__main__":
    try:
        app = QtGui.QApplication(sys.argv)
        trackpad = WallTrackpad()
        trackpad.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass




