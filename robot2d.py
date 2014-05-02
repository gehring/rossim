from Box2D import b2World, b2ChainShape
import Box2D
import numpy
import math

# basic robot constructor to build a circular robot
class DefaultRobot(object):

    min_force=-1
    max_force= 1
    vertex_count = 40

    def __init__(self, radius=0.5):
        self.radius = radius
        self.inputs = self.getDefaultInput()
        self.applypoints = ((-radius, 0), (radius, 0))
        self.vertices = numpy.array([(0,0)]+[(math.sin(x), math.cos(x))
                        for x in numpy.linspace(0, math.pi*2, self.vertex_count)])

    def createBody(self, world, **kargs):
        robotBody = world.CreateDynamicBody(**kargs)
        robotBody.CreateFixture( shape = Box2D.b2CircleShape(pos=(0,0),
                    radius = self.radius), density = 1.0, **kargs)
        return robotBody

    def applyForces(self, robotBody, inputs):
        pass
#         numpy.clip(inputs, self.min_force, self.max_force, self.inputs)
#         for i in range(2):
#             robotBody.ApplyForce(force = (0, self.inputs[i]),
#                                  point = self.applypoints[i] )

    def getDefaultInput(self):
        return numpy.zeros(2);

class Robot2d(object):

    def __init__(self, obstacles, **kargs):
        # --------------------------------------------------- #
        # set parameters
        # --------------------------------------------------- #
        robot_pos = kargs.get('robot_pos', (0,0))
        robot_angle = kargs.get('robot_angle', 0)
        robot = kargs.get('robot', DefaultRobot())
        robot_linear_damping = kargs.get('robot_linear_damping', 0.3)
        robot_angluar_damping = kargs.get('robot_angluar_damping', 0.3)
        robot_restitution = kargs.get('robot_restitution', 0.1)

        # world limit should be a list with two tuple. The coordinates
        # of the bottom left limit and of the top right limit
        world_limits = kargs.get('world_limit', None)

        obstacle_restitution = kargs.get('obstacle_restitution', 0)

        # Parameters for the Simulation, avoid changing these once the
        # simulation is running.

        # Simulated elapsed time per step, increasing this sacrifices accuracy
        # for more performance
        self.timestep = kargs.get('timestep', 1/10)
        # number of iterations of the solvers, lowering this sacrifices accuracy
        # for more performance
        self.vel_iters = kargs.get('vel_iters', 10)
        self.pos_iters = kargs.get('pos_iters', 8)
        # --------------------------------------------------- #

        # initialize an empty world
        self.world = b2World(gravity = (0.2,1), doSleep = True)

        # create obstacle shapes from list of vertex list
        self.__obstacle_vertices = obstacles
        shapes = [ b2ChainShape(vertices_loop = v) for v in obstacles]


        # attach all obstacle shape to the obstacle entity

#         map(lambda s: self.obstaclebody.CreateFixture(shape=s, density = 0,
#                         restitution = obstacle_restitution), shapes)

        # if limits were specified, create obstacle boundary
        if world_limits != None:
            v= [world_limits[0], (world_limits[1][0], world_limits[0][1]),
                   world_limits[1],  (world_limits[0][0], world_limits[1][1])]
            shapes.append(b2ChainShape(vertices_loop = v))
#             self.obstaclebody.CreateFixture(shape = limits, density = 0,
#                                             restitution = obstacle_restitution)

        # create an unmovable entity. This will represent all obstacles
        self.obstaclebody = self.world.CreateStaticBody(shapes = shapes)

        # package the robot physical properties (for collisions)
        # The robot constructor can choose to ignore them
        param = {'position': robot_pos,
                    'angle': robot_angle,
                    'linearDamping': robot_linear_damping,
                    'angularDamping': robot_angluar_damping,
                    'restitution': robot_restitution}

        # create the robot entity
        self.robotbody = robot.createBody(self.world, **param)
        self.robot = robot

    def step(self, inputs=None, timestep=None):
        self.inputs = inputs

        # apply various forces (e.g. robot actuator)
        self.applyforces()

        # simulate one step of the world
        if timestep == None:
            timestep = self.timestep
        self.world.Step(timestep, self.vel_iters, self.pos_iters)

        # clear forces
        self.world.ClearForces()

    def applyforces(self):
        self.robot.applyForces(self.robotbody, self.inputs)

    @property
    def inputs(self):
        self.robot.inputs

    @inputs.setter
    def inputs(self, value):
        self.robot.inputs[:] = value

    @property
    def velocity(self):
        return self.robotbody.linearVelocity

    @velocity.setter
    def velocity(self, value):
        self.robotbody.linearVelocity = value

    @property
    def angular_velocity(self):
        return self.robotbody.angularVelocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        self.robotbody.angularVelocity = value

    @property
    def position(self):
        return self.robotbody.position

    @position.setter
    def position(self, value):
        self.robotbody.position = value

    @property
    def angle(self):
        return self.robotbody.angle

    @angle.setter
    def angle(self, value):
        self.robotbody.angle = value

    @property
    def obstacle_vertices(self):
        return self.__obstacle_vertices



