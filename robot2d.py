from Box2D import b2World, b2ChainShape, b2Dot, b2ContactListener
import Box2D
import numpy
import math

# basic circular robot with differential drive
class DefaultRobot(b2ContactListener):

    min_force=-1
    max_force= 1
    vertex_count = 40

    bumper_h = 0.05
    bumper_w = 0.1

    def __init__(self, radius=0.5, nbumpers = 6, nlasers= 0):
        b2ContactListener.__init__(self)

        self.radius = radius
        self.inputs = self.getDefaultInput()
        self.applypoints = ((-radius, 0), (radius, 0))
        self.vertices = numpy.array([(0,0)]+[(math.sin(x)*radius, math.cos(x)*radius)
                        for x in numpy.linspace(0, math.pi*2, self.vertex_count)])
        self.nlasers = nlasers
        self.nbumpers = nbumpers

        self.output = numpy.zeros(nlasers + nbumpers)

    def createBody(self, world, **kargs):
        robotBody = world.CreateDynamicBody(**kargs)
        robotBody.CreateFixture( shape = Box2D.b2CircleShape(pos=(0, 0),
                    radius = self.radius), density = 1.0, **kargs)

        for i, angle in enumerate(numpy.linspace(0, math.pi*2, self.nbumpers+1)[:-1]):
            c = numpy.array((self.radius * math.sin(angle),
                              self.radius * math.cos(angle)))
            robotBody.CreateFixture(shape = Box2D.b2PolygonShape(
                                                        box = (self.bumper_w/2,
                                                               self.bumper_h/2,
                                                        (c[0], c[1]),
                                                        angle)),
                                    density = 0.01,
                                    isSensor = True,
                                    userData = 'b'+ str(i))

        return robotBody

    def applyForces(self, robotBody, inputs):
        numpy.clip(inputs, self.min_force, self.max_force, self.inputs)
        normal = robotBody.GetWorldVector((1,0))
        lateralimp = -robotBody.mass* b2Dot(normal, robotBody.linearVelocity) * normal
        robotBody.ApplyLinearImpulse(impulse= lateralimp,
                                      point= robotBody.worldCenter,
                                      wake = True)
        for i in range(2):
            robotBody.ApplyForce(force = robotBody.GetWorldVector((0, self.inputs[i])),
                                 point = robotBody.GetWorldPoint(self.applypoints[i]),
                                 wake = True )

    def getDefaultInput(self):
        return numpy.zeros(2);

    def BeginContact(self, contact):
        uA = contact.fixtureA.userData
        uB = contact.fixtureB.userData
        index = -1
        if isinstance(uA, basestring) and uA[0] == 'b':
            index = int(uA[1:])
        elif isinstance(uB, basestring) and uB[0] == 'b':
            index = int(uB[1:])

        if index > -1:
            self.output[index] = 1

    def EndContact(self, contact):
        uA = contact.fixtureA.userData
        uB = contact.fixtureB.userData
        index = -1
        if isinstance(uA, basestring) and uA[0] == 'b':
            index = int(uA[1:])
        elif isinstance(uB, basestring) and uB[0] == 'b':
            index = int(uB[1:])

        if index > -1:
            self.output[index] = 0





class Robot2d(object):

    def __init__(self, obstacles, **kargs):
        # --------------------------------------------------- #
        # set parameters
        # --------------------------------------------------- #
        robot_pos = kargs.get('robot_pos', (0,0))
        robot_angle = kargs.get('robot_angle', 0)
        robot = kargs.get('robot', DefaultRobot())
        robot_linear_damping = kargs.get('robot_linear_damping', 0.7)
        robot_angluar_damping = kargs.get('robot_angluar_damping', 0.9)
        robot_restitution = kargs.get('robot_restitution', 0.1)

        # world limit should be a list with two tuple. The coordinates
        # of the bottom left limit and of the top right limit
        world_limits = kargs.get('world_limit', None)


        # Parameters for the Simulation, avoid changing these while the
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
        if isinstance(robot, b2ContactListener):
            self.world = b2World(contactListener = robot,
                                  gravity = (0,0),
                                  doSleep = True)
        else:
            self.world = b2World(gravity = (0,0), doSleep = True)

        # create obstacle shapes from list of vertex list
        self.__obstacle_vertices = obstacles
        shapes = [ b2ChainShape(vertices_loop = v) for v in obstacles]

        # if limits were specified, create obstacle boundary
        if world_limits != None:
            v= [world_limits[0], (world_limits[1][0], world_limits[0][1]),
                   world_limits[1],  (world_limits[0][0], world_limits[1][1])]
            shapes.append(b2ChainShape(vertices_loop = v))

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
        self.inputs = robot.getDefaultInput()


    def step(self, inputs=None, timestep=None):
        if inputs != None:
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

#     @property
#     def inputs(self):
#         self.robot.inputs
#
#     @inputs.setter
#     def inputs(self, value):
#         self.robot.inputs[:] = value

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



