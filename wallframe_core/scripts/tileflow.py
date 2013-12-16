import math
import sys
from PySide import QtCore, QtGui, QtOpenGL
from OpenGL import GLU, GL


class TileflowWidget(QtOpenGL.QGLWidget):

    SCALE = 0.7
    SPREAD_IMAGE = 0.14
    FLANK_SPREAD = 0.4
    VISIBLE_TILES = 10
    DIRECTION = 1


    def __init__(self, parent, res_list=[]):
        QtOpenGL.QGLWidget.__init__(self, parent)

        self.parent = parent
        self.width = 0
        self.clearColor = QtCore.Qt.black
        self.lastPos = QtCore.QPoint()
        self.lastCursor = (0, 0)
        self.res_list = res_list
        self.tiles = []
        self.steps_in_direction = 0
        self.current_direction = 0 # -1: left 1: right 0:empty
        self.offset = 3
        self.mouseDown = False
        self.responding = True
        # self.width = 533
        focus_tile_timer = QtCore.QTimer(self)
        focus_tile_timer.timeout.connect(self.focus_tile)
        focus_tile_timer.start(20)


    def set_resources(self, res_list):
        self.res_list = res_list
        self.paintGL()

    def minimumSizeHint(self):
        return QtCore.QSize(533, 270)

    def sizeHint(self):
        return QtCore.QSize(533, 270)

    def setClearColor(self, color):
        self.clearColor = color
        self.updateGL()

    def initializeGL(self):
        for res_path in self.res_list:
            self.tiles.append(Tile(self.bindTexture(QtGui.QPixmap(res_path))))
        self.first_tile = self.make_tiles()


    def make_tiles(self):
        ind = list_start = GL.glGenLists(len(self.res_list))

        for tile in self.tiles:
            texture = tile.texture
            GL.glNewList(ind, GL.GL_COMPILE)
            GL.glBindTexture(GL.GL_TEXTURE_2D, tile.texture)

            GL.glBegin(GL.GL_QUADS)
            GL.glTexCoord2d(1, 0)
            GL.glVertex3d(1, -1, 0)
            GL.glTexCoord2d(0, 0)
            GL.glVertex3d(-1, -1, 0)
            GL.glTexCoord2d(0, 1)
            GL.glVertex3d(-1, 1, 0)
            GL.glTexCoord2d(1, 1)
            GL.glVertex3d(1, 1, 0)
            GL.glEnd()

            GL.glTranslatef(0, -2.0, 0)
            GL.glScalef(1, -1, 1)
            GL.glColor4f(1, 1, 1, 0.5)

            GL.glBegin(GL.GL_QUADS)
            GL.glTexCoord2d(1, 0)
            GL.glVertex3d(1, -1, 0)
            GL.glTexCoord2d(0, 0)
            GL.glVertex3d(-1, -1, 0)
            GL.glTexCoord2d(0, 1)
            GL.glVertex3d(-1, 1, 0)
            GL.glTexCoord2d(1, 1)
            GL.glVertex3d(1, 1, 0)
            GL.glEnd()

            GL.glColor4f(1, 1, 1, 1)

            GL.glEndList()

            ind += 1

        return list_start

    def paintGL(self):
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        GLU.gluLookAt(0, 0, 2, 0, 0, 0, 0, 1, 0)
        GL.glDisable(GL.GL_DEPTH_TEST)

        self.qglClearColor(self.clearColor)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glPushMatrix()
        GL.glEnable(GL.GL_TEXTURE_2D)
        GL.glEnable(GL.GL_BLEND)
        GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)

        offset = self.offset
        if offset <= 0:
            offset = 0
        if offset > len(self.res_list) - 1:
            offset = len(self.res_list) - 1
        mid = int(math.floor(offset + 0.5))
        start_pos = mid - TileflowWidget.VISIBLE_TILES
        if start_pos < 0:
            start_pos = 0
        end_pos = mid + TileflowWidget.VISIBLE_TILES
        if end_pos > len(self.res_list):
            end_pos = len(self.res_list)
        for i in range(start_pos, mid)[::TileflowWidget.DIRECTION]:
            self.drawTile(i, i - offset, self.tiles[i])
        for i in range(mid, end_pos)[::-TileflowWidget.DIRECTION]:
            self.drawTile(i, i - offset, self.tiles[i])

        GL.glPopMatrix()

    def resizeGL(self, width, height):
        self.width = width
        imagew = width * 0.45 / TileflowWidget.SCALE / 2.0
        imageh = height * 0.45 / TileflowWidget.SCALE / 2.0

        GL.glViewport(0, 0, width, height)
        ratio = float(width) / height
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(-ratio * TileflowWidget.SCALE, ratio * TileflowWidget.SCALE, -1 * TileflowWidget.SCALE, 1 * TileflowWidget.SCALE, 1, 3)

    def drawTile(self, position, offset, tile):
        matrix = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        trans = offset * TileflowWidget.SPREAD_IMAGE
        f = offset * TileflowWidget.FLANK_SPREAD
        if (f > TileflowWidget.FLANK_SPREAD):
            f = TileflowWidget.FLANK_SPREAD
        elif (f < -TileflowWidget.FLANK_SPREAD):
            f = -TileflowWidget.FLANK_SPREAD

        matrix[3] = -1 * TileflowWidget.DIRECTION * f
        matrix[0] = 1 - abs(f)
        scale = 0.38 * matrix[0]
        trans += f * 1
        GL.glPushMatrix()
        GL.glTranslatef(trans, 0, 0)
        GL.glScalef(scale, scale, 1.0)
        GL.glMultMatrixf(matrix)
        GL.glCallList(self.first_tile + position)
        GL.glPopMatrix()


    # def focus_tile(self):
    #     if not self.mouseDown:
    #         target = math.floor(self.offset + 0.5)
    #         if not abs(target - self.offset) <= 0.01:
    #             self.offset += (target - self.offset) / 3
    #             self.updateGL()
    #         else:
    #           self.responding = False
    #           QtCore.QTimer.singleShot(350, self.setResponding)

    def focus_tile(self):
        if not abs(self.target - self.offset) <= 0.01:
            self.offset += (self.target - self.offset) / 3
            self.updateGL()

    def get_target(self, offset):
        if offset < 0:
            offset_thres = 0
        elif offset > len(self.res_list) - 1:
            offset_thres = len(self.res_list) - 1
        else:
            offset_thres = offset
        target = math.floor(offset_thres + 0.5)
        return target

    def update_cursor(self, cursor_position):
        cur_x, cur_y = cursor_position
        last_x, last_y = self.lastCursor
        dx = cur_x - last_x
        d = float(dx) / self.width
        if abs(d) >= 0.01:
        # if abs(d) >= 0.025:
            self.current_direction = d // abs(d)
            # offset = self.offset - 5 * d
            offset = self.offset - 8 * d
            self.target = self.get_target(offset)
            print "new target: ", self.target
        self.updateGL()
        self.lastCursor = (cur_x, cur_y)

    def click(self):
        offset = self.offset
        if offset <= 0:
            offset = 0
        if offset > len(self.res_list) - 1:
            offset = len(self.res_list) - 1
        mid = int(math.floor(offset + 0.5))
        self.parent.clicked_on(mid)


    def releaseControl(self):
        self.mouseDown = False


    def mousePressEvent(self, event):
        self.lastPos = QtCore.QPoint(event.pos())
        self.mouseDown = True

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        offset = self.offset - float(dx) * 6 / (self.width * 0.6)
        if offset < 0:
            self.offset = 0
        elif offset > len(self.res_list) - 1:
            self.offset = len(self.res_list) - 1
        else:
            self.offset = offset
        self.updateGL()

        self.lastPos = QtCore.QPoint(event.pos())

    def mouseReleaseEvent(self, event):
        self.mouseDown = False

class Tile:
    def __init__(self, texture):
        self.texture = texture
