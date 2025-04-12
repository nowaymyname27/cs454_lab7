from dataclasses import dataclass # To create simple classes for data
import pybullet as p # physics simulation library
import time # standard system time functions
import numpy as np # for python number calculations (written in C btw)
import utils # imports utils.py file
import argparse # parses command-line args when script is run


@dataclass # decorator to instantiate a class quickly (I hate decorators and objects)
class Point: # acts as point in an X and Y axis
    x: float # x-coordinate
    y: float # y-coordinate

@dataclass # decorator to instantiate a class quickly (I hate decorators and objects)
class World:
    """A class to hold the world objects"""
    plate: int # ID number that PyBullet assigns the plate object
    sphere: int # ID number that PyBullet assigns the sphere object

def parse_args():
    '''
    parses command-line arguments using argparse library
    '''
    parser = argparse.ArgumentParser() # Creates and ArgumentParser object (Objects suck!)
    # add_argument will be used to add different arguments accepted by the script in the Command Line
    parser.add_argument(
        "--setpoint", # argument
        type=float, # expects a float
        nargs=2, # two vals after the arg
        default=(0.0,0.0) # if --setpoint is not provided it will default to a tuple (0.0,0.0)
        )
    parser.add_argument(
        "--kp",  # argument
        type=float, # expects a float
        default=0.0 #defaults to 0.0 if arg not provided
        ) # set these values to tune the PD controller
    parser.add_argument(
        "--kd",  # argument
        type=float, # expects a float
        default=0.0 # defaults to 0.0  if arg not provided
        ) # set these values to tune the PD controller
    parser.add_argument(
        "--noise",  # argument
        action="store_true", # acts like a flag, True if arg is present False otherwise
        help="Add noise to the measurements" # description for is the user asks for help
        )
    parser.add_argument(
        "--filtered",  # argument
        action="store_true", # acts like a flag, True if arg is present False otherwise
        help="filter the measurements" # description for is the user asks for help
        )
    cmd_args = parser.parse_args() # parses through the command arguments and stores them as an object
    print(cmd_args) # shows the command arguments selected
    cmd_args.setpoint = Point(*cmd_args.setpoint) # Converts the setpoint argument into a point object
    return cmd_args # returns cmd_args object which contains all the parsed and processed config settings


def run_controller(kp, kd, setpoint, noise, filtered, world: World):
    '''
    Responsible for the main control loop which contains
    the logic for continously controlling the plate based
    on the ball's position
    
    kp: float representing the Proportional Gain (how strongly the controller reacts to the ball's current position)
    kd: flaot representing the Derivative Gain (how strongly the controller reacts to the ball's rate of change)
    setpoint: Point object representing the ball's position
    noise: boolean determined by the --noise command line argument, if True the controller adds artificial noise
    filtered: boolean determined by the --filtered command line arg, if True the controller applies filter to the ball's position.
    world: World object, contains IDs of plate and sphere assigned by PyBullet
    '''
    def set_plate_angles(theta_x, theta_y):
        '''
        Takes the desired tilt angle of the plate and translates them into commands
        for PyBullet simulation's motor.
        
        theta_x: float, calculated desired tilt angle of the plate around its local Y-axis
        theta_y: float, calculated desired tilt angle of the plate around its local X-axis
        '''
        p.setJointMotorControl2( # Function used to control a joint in a multi-body object (the plate)
            world.plate, # ID of the object we want to control
            1, # which joint we are controlling (X-axis via tilt around Y-axis)
            p.POSITION_CONTROL, # Tells PyBullet how to control the joint, POSITION_CONTROL is for a target angle
            targetPosition=np.clip(theta_x, -0.1, 0.1), # Sets desired angle between a range(-0.1,0.1)
            force=5, # Maximum force that the simulation motor is allowed to use
            maxVelocity=2 # Max velocity for the joint motor when moving towards a desired angle
            )
        p.setJointMotorControl2( # Function used to control a joint in a multi-body object (the plate)
            world.plate, # ID of the object we want to control (same plate)
            0, # Which joint we are controlling (Y-axis movement via tilt around X-axis)
            p.POSITION_CONTROL, # Tells PyBullet how to control the joint, POSITION_CONTROL is for a target angle
            targetPosition=np.clip(-theta_y, -0.1, 0.1), # Sets desired angle, using range [-0.1, 0.1] radians
            force=5, # Maximum force that the simulation motor is allowed to use (same as other joint)
            maxVelocity=2 # Max velocity for the joint motor when moving towards a desired angle (same as other joint)
            )

    # you can set the variables that should stay accross control loop here
    

    def pd_controller(x, y, kp, kd, setpoint):
        """Implement a PD controller, you can access the setpoint via setpoint.x and setpoint.y
        the plate is small around 0.1 to 0.2 meters. You will have to calculate the error and change in error and 
        use those to calculate the angle to apply to the plate."""
        return 0.0, 0.0


    def filter_val(val):
        """Implement a filter here, you can use scipy.signal.butter to compute the filter coefficients and then scipy.signal.lfilter to apply the filter.but we recommend you implement it yourself instead of using lfilter because you'll have to do that on the real system later.
        Take a look at the butterworth example written by Renato for inspiration."""
        pass

    def every_10ms(i: int, t: float):
        '''This function is called every ms and performs the following:
        1. Get the measurement of the position of the ball
        2. Calculate the forces to be applied to the plate
        3. Apply the forces to the plate
        '''
        (x,y,z), orientation = p.getBasePositionAndOrientation(world.sphere)
        if noise:
            x += utils.noise(t) # the noise added has a frequency between 30 and 50 Hz
            y += utils.noise(t, seed = 43) # so that the noise on y is different than the one on x
        
        if filtered:
            x = filter_val(x)
            y = filter_val(y)

        (angle_x, angle_y) = pd_controller(x, y, kp, kd, setpoint)
        set_plate_angles(angle_x, angle_y)

        if i%10 == 0: # print every 100 ms
            print(f"t: {t:.2f}, x: {x:.3f},\ty: {y:.3f},\tax: {angle_x:.3f},\tay: {angle_y:.3f}")

    utils.loop_every(0.01, every_10ms) # we run our controller at 100 Hz using a linux alarm signal

def run_simulation( initial_ball_position = Point(np.random.uniform(-0.2, 0.2),
                                                  np.random.uniform(-0.2, 0.2))):
    p.connect(p.GUI)
    p.setAdditionalSearchPath("assets")
    plate = p.loadURDF("plate.urdf")
    sphere = p.createMultiBody(0.2
        , p.createCollisionShape(p.GEOM_SPHERE, radius=0.04)
        , basePosition = [initial_ball_position.x,initial_ball_position.y,0.5]
    )

    #zoom to the plate
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0,0,0])

    p.setJointMotorControl2(plate, 0, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2)
    p.setJointMotorControl2(plate, 1, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2)

    p.setGravity(0, 0, -9.8)

    #update the simulation at 100 Hz
    p.setTimeStep(0.01)
    p.setRealTimeSimulation(1)
    return World(plate=plate, sphere=sphere)


if __name__ == "__main__":
    cmd_args = parse_args()
    world = run_simulation()
    run_controller(**vars(cmd_args), world=world)
    time.sleep(10000)
