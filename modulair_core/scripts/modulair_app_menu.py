#!/usr/bin/env python
"""
  Modulair App_ Menu
  First prototype of modulair_app_menu. Uses Model-View/Controller approach.
  View and controller are combined because Qt signal/slots intrinsically do not cater
  to a separated view and controller.


    
  @author Andy Tien, Kel Guerin, Zihan Chen
"""

# TODO ZC
# 
# use QLabel for now + may use modulair_app_menu_button class later
# use QPushbutton
# call launch file when button is pushed
# Ask kel:
#   1. how to get application names & protocol
#   2. try to get application name dynamically 


# ROS import
import roslib; roslib.load_manifest('modulair_core')
import rospy

# system import
import os, collections, sys, math

# PySide import
from PySide.QtGui import QWidget, QApplication, QGridLayout, QPushButton, QLabel
from PySide.QtCore import QSize

# modulair import
# msg
from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
# srv
import modulair_core
from modulair_core.srv import *

from modulair_app_menu_button import ModulairAppButton


#=========== MODEL =======================
class ModulairMenuModel:
  def __init__(self, app_path):
    self.app_path = app_path
    self.applist = []
    self.App = collections.namedtuple('App', ['name', 'folderpath', 'launchpath'])
    self.Viewers = [] 
  
  def getAppList(self):
    """ Clears the applist, and returns an updated version via a call to scan. """
    self.applist = [] #return latest version
    self.scan()
    return self.applist
  
  def scan(self):
    """ Scan the path for application instances. Return a list of paths
    for all the applications contained inside. Uses os.walk to traverse subdirectories
    from the app_path passed to the model. 
    
    Assumes standard ROS package file organization, which has .launch files in a subdirectory
    named "launch".
    """   
    for root, unused_dirs, files in os.walk(self.app_path):
      for f in files:
        if f.endswith(".launch"): #recognized this directory as an application directory
          assert os.path.basename(root) == "launch"
          appname = os.path.basename(os.path.dirname(root))
          launchpath = os.path.join(root, f)
          self.applist.append(self.App(appname, root, launchpath))


# ============ VIEW =======================                    
class ModulairMenuView(QWidget):

  # constructor
  def __init__(self, filelocation):
    self.qt_app_ = QApplication(sys.argv)
    QWidget.__init__(self)

    # ROS
    rospy.init_node('modulair_app_menu', anonymous=True)

    # ---- ROS subscriber ---
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state",
                                            ModulairUserArray,
                                            self.user_state_cb)

    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events",
                                            ModulairUserEvent,
                                            self.user_event_cb)
    

    # ---- ROS get params -----
    # height
    if rospy.has_param("/modulair/core/params/height"):
      self.height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr("ModulairInfobar: parameter [height] not found on server")

    # width
    if rospy.has_param("/modulair/core/params/width"):
      self.width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr("ModulairInfobar: parameter [width] not found on server")

    # x
    if rospy.has_param("/modulair/core/params/x"):
      self.x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr("ModulairInfobar: parameter [x] not found on server")

    # y
    if rospy.has_param("/modulair/core/params/y"):
      self.y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr("ModulairInfobar: parameter [y] not found on server")

    # create model
    self.model_ = ModulairMenuModel(filelocation)
    # Applications List
    self.app_list_ = sorted(self.model_.getAppList())
    self.gridSet_ = False    

    # setup Qt GUI
#    self.setMinimumSize(QSize(self.width_,self.height_))
#    self.setMaximumSize(QSize(self.width_,self.height_))
    
    self.setWindowTitle("Modulair")
    self.gridLayout_ = QGridLayout() 
    self.assignWidgets() # create widget
    self.setLayout(self.gridLayout_)

  
  def setup_grid(self):
    """
    Sets up the grid size that will represent the applications in the menu.
    """
    length = len(self.app_list_) + 1
    n = int(math.ceil(math.sqrt(length)))
    self.max_x_ = self.max_y_ = n
    self.cur_ind_x_ = 0 
    self.cur_ind_y_ = 0
    self.gridSet_ = True

  def next_pos(self):
    """ Return a tuple of the next position """

    # check if grid ind has been set
    if not self.gridSet_:
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


  # create QLabel for each app + refresh button
  def assignWidgets(self):
    for app in self.app_list_:
      # widget = QLabel(app.name + '\n' + app.launchpath)
      widget = QLabel(app.name)
      # set label icon for testing 
      image = app.folderpath + '/../menu_icon.png'
      widget.setPixmap(image)
      widget.setFixedSize(self.height_/10.0, self.width_/10.0)
      nextx, nexty = self.next_pos()
      self.gridLayout_.addWidget(widget, nextx, nexty )
    
    widget = QPushButton('Refresh')
    widget.clicked.connect(self.refresh)
    nextx, nexty = self.next_pos()
    self.gridLayout_.addWidget(widget, nextx, nexty)

  # delete all widget + recreate all widget ???
  def refresh(self):
    for unused_i in range(0, len(self.app_list_)+1):
      g = self.gridLayout_.takeAt(0)
      g.widget().deleteLater()
    self.app_list_ = sorted(self.model_.getAppList())
    self.gridSet_ = False
    self.assignWidgets()

  # user_state_cb callback
  def user_state_cb(self, msg):
    pass

  def user_event_cb(self, msg):
    print msg.user_id
    # assume HANDS_HEAD = PAUSE
    if msg.event_id == 'hand_event':
      print msg.event_id
      if msg.message == 'hands_on_head':
        rospy.logdebug("ModulairMenu: HANDS_HEAD received, should resume menu")
        # wait or check current app status
        # when app is paused
        # then raise modulair_menu again
        # now just refresh & raise menu
        self.refresh()
        self.show()
        
      # assume RIGHT_ELBOW_CLICK
      if msg.message == 'right_elbow_click':
        print msg.message
        rospy.logdebug("ModulairMenu: RIGHT_ELBOW_CLICK received, let's launch app")
        rospy.wait_for_service('modulair/core/app_manager/load_app')
        print "here"
        try:
          self.srv_load_app = rospy.ServiceProxy('modulair/core/app_manager/load_app',
                                                 modulair_core.srv.load_app)
          ret_success = self.srv_load_app("Load_sample_app")
          print ret_success
        except rospy.ServiceException, e:
          print "Service call failed: %s"%e
      
  # show widget and Qt.exec()
  def run(self):
    self.show()
    self.qt_app_.exec_()

        
def test():
  filelocation = rospy.get_param('/modulair/core/paths/application_path')
  menuview = ModulairMenuView(filelocation)
  menuview.setup_grid()
  menuview.run()

if __name__ == "__main__":
  test() 
