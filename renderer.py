import pyglet
from pyglet import window

class RobotWindow(window):
    default_visible= 6
    zoom_ratio = 1.05
    obstacle_color= (150, 150, 240)
    robot_color = (200, 50, 50)

    def __init__(self, **kargs):
        super(RobotWindow, self).__init__(**kargs)
        self.robot2d = kargs.get('robot2d', None)
        self.__robot = None
        self.batchrender = None
        self.robotautodraw = False

    def on_draw():
        if self.__robot is not self.robot2d:
            self.__robot = self.robot2d

            # check if robot offers its own rendering
            self.robotautodraw = hasattr(self.robot2d.robot, 'draw')
            self.robotautodraw &= callable(getattr(self.robot2d.robot, 'draw'))

            # generate obstacle list
            self.obstaclerender = pyglet.graphics.Batch()
            obsvert = self.robot2d.obstacle_vertices

        if self.robot2d != None:
            if self.obstaclerender != None:
                self.obstaclerender.draw()

            pos = self.robot2d.position
            angle = self.robot2d.angle

            pyglet.gl.glMatrixPush()

            pyglet.gl.glTranslate(pos[0], pos[1], 0)
            pyglet.gl.glRotate(0, 0, angle)
            if self.robotautodraw:
                self.robot2d.robot.draw()
            elif self.robotrender != None:
                self.robotrender.draw()

            pyglet.gl.glMatrixPop()


    def on_resize(width, height):
        pyglet.gl.glViewport(0, 0, width, height)
        self.setProjection(width, height)

    def on_mouse_scroll(x, y, scroll_x, scroll_y):
        (mx, my)= self.getMouseCoord(x, y)
        pyglet.gl.glTranslatef(mx, my, 0)
        pyglet.gl.glScalef(self.zoom_ratio**scroll_y, self.zoom_ratio**scroll_y, 1)
        pyglet.gl.glTranslatef(-mx, -my, 0)

    def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
        mcoord1 = self.getMouseCoord(x, y)
        mcoord2 = self.getMouseCoord(x + dx, y+ dy)
        pyglet.gl.glTranslatef(mcoord2[0] - mcoord1[0], mcoord2[1] - mcoord1[1], 0)


    def set_projection(self, width, height):
        pyglet.gl.glMatrixMode(pyglet.gl.GL_PROJECTION)
        pyglet.gl.glLoadIdentity()

        ratio = float(height)/width
        lx = self.default_visible
        ly = self.default_visible

        if lx*ratio >= ly:
            dy = lx*ratio - ly
            pyglet.gl.glOrtho(0, self.default_visible, -dy/2,
                     self.default_visible+dy/2, -1, 1)
        else:
            dx = ly/ratio - lx
            pyglet.gl.glOrtho(-dx/2, self.default_visible + dx/2,
                    0, self.default_visible, -1, 1)


        pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)

    def getMouseCoord(self, x, y):
        vp = (pyglet.gl.GLint * 4)()
        mvm = (pyglet.gl.GLdouble * 16)()
        pm = (pyglet.gl.GLdouble * 16)()

        pyglet.gl.glGetIntegerv(pyglet.gl.GL_VIEWPORT, vp)
        pyglet.gl.glGetDoublev(pyglet.gl.GL_MODELVIEW_MATRIX, mvm)
        pyglet.gl.glGetDoublev(pyglet.gl.GL_PROJECTION_MATRIX, pm)

        wx = pyglet.gl.GLdouble()
        wy = pyglet.gl.GLdouble()
        wz = pyglet.gl.GLdouble()

        pyglet.gl.gluUnProject(x, y, 0, mvm, pm, vp, wx, wy, wz)
        mcoord = (wx.value, wy.value)

        return mcoord

configTemp = pyglet.gl.Config(sample_buffers=1,
    samples=4,
    double_buffer=True,
    alpha_size=0)

platform = pyglet.window.get_platform()
display = platform.get_default_display()
screen = display.get_default_screen()

try:
  config= screen.get_best_config(configTemp)
except:
  config=pyglet.gl.Config(double_buffer=True)

window = RobotWindow(config=config, resizable=True)

if __name__ == '__main__':
    pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
    pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)
    pyglet.gl.glEnable(pyglet.gl.GL_LINE_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POLYGON_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POINT_SMOOTH )
    pyglet.gl.glClearColor(0, 0, 0, 1.0)
    pyglet.app.run()