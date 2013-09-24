'''
UserBar Tabs
'''

from PySide.QtGui import QWidget, QLabel
from PySide.QtCore import Qt, QSize

class ModulairAppUserBarTab(QLabel):
  def __init__(self, text, parent):
    super(ModulairAppUserBarTab, self).__init__(text, parent)
    self.width_ = 50
    self.height_ = 30
    self.setFixedSize(QSize(self.width_, self.height_))
  def update_position(self, x, y):
    self.move(x,y)
    
#class ModulairAppUserBarPopUp(QWidget):
#  def __init__(self, parent):
#    super(ModulairAppUserBarPopUp, self).__init__(parent)
#    self.width_ = 50
#    self.height_ = 90
#    self.setFixedSize(QSize(self.width_, self.height_))
#    self.tab.setWindowFlags(Qt.FramelessWindowHint)
#    self.tabx = 200
#    self.taby = 300
#    self.tab.move(self.tabx, self.taby)