import pyglet
from pyglet import window
from pyglet.window import key
from robot2d import Robot2d, DefaultRobot
import math
import numpy


class DefaultRobotWithDraw(DefaultRobot):
    robot_color = (200, 50, 50)

    def __init__(self):
        DefaultRobot.__init__(self)
        self.robotrender = pyglet.graphics.Batch()
        vertices = self.vertices
        indices = [[0]] + [[x, x] for x in range(1, len(vertices)) ] + [[0]]
        indices = [x for l in indices for x in l]
        vertices = [ x for p in vertices for x in p]

        self.robotrender.add_indexed(len(vertices)/2, pyglet.gl.GL_LINES,
                                     None, indices,
                                     ('v2f', vertices),
                                     ('c3B', self.robot_color*(len(vertices)/2)))

    def draw(self):
        self.robotrender.draw()
        vertices = [ self.getToFromLaser(a)[0] for a in
                            numpy.linspace(0, math.pi*2, self.nlasers+1)[:-1]]
        vertices = [ (pfrom, pfrom * (self.output[self.nbumpers + i]/ numpy.linalg.norm(pfrom) + 1))
                      for i, pfrom in enumerate(vertices)]
        vertices = [ x for l in vertices for p in l for x in p]

        pyglet.graphics.draw(len(vertices)/2, pyglet.gl.GL_LINES,
                             ('v2f', vertices),
                             ('c4B', (230, 20, 20, 100)*(len(vertices)/2)))


class RobotWindow(window.Window):
    default_visible= 6
    zoom_ratio = 1.05
    obstacle_color= (150, 150, 240)
    robot_color = (200, 50, 50)

    def __init__(self, robot2d= None, **kargs):
        super(RobotWindow, self).__init__(**kargs)
        self.robot2d = robot2d
        self.__robot = None
        self.batchrender = None
        self.robotautodraw = False

    def update(self, dt):
        self.robot2d.step(timestep = dt)
        o = self.robot2d.output

    def on_draw(self):
        self.clear()
        if self.__robot is not self.robot2d:
            self.__robot = self.robot2d

            # check if robot offers its own rendering
            self.robotautodraw = hasattr(self.robot2d.robot, 'draw')
            if self.robotautodraw:
                self.robotautodraw &= callable(getattr(self.robot2d.robot, 'draw'))

            # generate obstacle list
            self.obstaclerender = pyglet.graphics.Batch()
            obsvert = self.robot2d.obstacle_vertices
            vertices = [ v for p in obsvert for v in p]

            offset = 0
            indices = []
            for vertex in obsvert:
                tmpind = [ [x+offset, x+offset] for x in range(1, len(vertex)) ]
                ind =  [offset] + [j for i in tmpind for j in i] + [offset]
                indices.append(ind)
                offset += len(vertex)
            indices = [x for l in indices for x in l]
            vertices = [ x for p in vertices for x in p]
            self.obstaclerender.add_indexed(len(vertices)/2, pyglet.gl.GL_LINES,
                                            None, indices,
                                            ('v2f', vertices),
                                            ('c3B', self.obstacle_color*(len(vertices)/2)))

            if not self.robotautodraw:
                self.robotrender = pyglet.graphics.Batch()
                vertices = self.robot2d.robot.vertices
                indices = [[0]] + [[x, x] for x in range(1, len(vertices)) ] + [[0]]
                indices = [x for l in indices for x in l]
                vertices = [ x for p in vertices for x in p]

                self.robotrender.add_indexed(len(vertices)/2, pyglet.gl.GL_LINES,
                                             None, indices,
                                             ('v2f', vertices),
                                             ('c3B', self.robot_color*(len(vertices)/2)))


        if self.robot2d != None:
            if self.obstaclerender != None:
                self.obstaclerender.draw()

            pos = self.robot2d.position
            angle = self.robot2d.angle

            pyglet.gl.glPushMatrix()

            pyglet.gl.glTranslatef(pos[0], pos[1], 0)
            pyglet.gl.glRotatef(math.degrees(angle), 0, 0, 1)
            if self.robotautodraw:
                self.robot2d.robot.draw()
            elif self.robotrender != None:
                self.robotrender.draw()

            pyglet.gl.glPopMatrix()


    def on_resize(self, width, height):
        pyglet.gl.glViewport(0, 0, width, height)
        self.set_projection(width, height)

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        (mx, my)= self.getMouseCoord(x, y)
        pyglet.gl.glTranslatef(mx, my, 0)
        pyglet.gl.glScalef(self.zoom_ratio**scroll_y, self.zoom_ratio**scroll_y, 1)
        pyglet.gl.glTranslatef(-mx, -my, 0)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
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

    def on_key_press(self, symbol, modifiers):
        if self.robot2d != None:
            if symbol == key.LEFT:
                self.robot2d.inputs += (-0.7,0.7)
            if symbol == key.RIGHT:
                self.robot2d.inputs += (0.7,-0.7)
            if symbol == key.UP:
                self.robot2d.inputs += (0.7,0.7)
            if symbol == key.DOWN:
                self.robot2d.inputs -= (0.7,0.7)
        super(RobotWindow, self).on_key_press(symbol, modifiers)

    def on_key_release(self, symbol, modifiers):
        if self.robot2d != None:
            if symbol == key.LEFT:
                self.robot2d.inputs -= (-0.7,0.7)
            if symbol == key.RIGHT:
                self.robot2d.inputs -= (0.7,-0.7)
            if symbol == key.UP:
                self.robot2d.inputs -= (0.7,0.7)
            if symbol == key.DOWN:
                self.robot2d.inputs += (0.7,0.7)


if __name__ == '__main__':
    robot = Robot2d([[ (1,1), (2,5), (2,3)], [ (10,10), (20,50), (20,30)]],
                    robot = DefaultRobotWithDraw())

    configTemp = pyglet.gl.Config(sample_buffers=1,
        samples=4,
        double_buffer=True)

    platform = pyglet.window.get_platform()
    display = platform.get_default_display()
    screen = display.get_default_screen()

    try:
        config= screen.get_best_config(configTemp)
    except:
        config=pyglet.gl.Config(double_buffer=True)

    w = RobotWindow(config=config, resizable=True, robot2d = robot)
    pyglet.clock.schedule_interval(w.update, 1.0/20)

    pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
    pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)
    pyglet.gl.glEnable(pyglet.gl.GL_LINE_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POLYGON_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POINT_SMOOTH )
    pyglet.gl.glLineWidth(2)
    pyglet.gl.glPointSize(5)
    pyglet.gl.glClearColor(0, 0, 0, 1.0)
    pyglet.app.run()