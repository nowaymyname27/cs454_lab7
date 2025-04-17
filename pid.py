from dataclasses import dataclass # To create simple classes for data
import pybullet as p # physics simulation library
import time # standard system time functions
import numpy as np # for python number calculations (written in C btw)
import utils # imports utils.py file
import argparse # parses command-line args when script is run
from collections import deque
from scipy import signal


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
    Responsible for the main control loop which contains the logic for continously controlling the plate based
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
    last_error_x = 0.0
    last_error_y = 0.0
    dt = 0.01
    
    # Filter Parameters
    filter_order = 2
    cutoff_freq_hz = 20.0 # Hz
    sampling_freq_hz = 100.0 # Hz (must match loop rate dt=0.01)

    # Calculate normalized cutoff frequency (fraction of Nyquist frequency)
    nyquist_freq_hz = sampling_freq_hz / 2.0
    normalized_cutoff = cutoff_freq_hz / nyquist_freq_hz

    # Calculate filter coefficients using scipy.signal.butter
    # b: numerator coefficients (feedforward)
    # a: denominator coefficients (feedback)
    # Note: a[0] will be 1 for Butterworth filters from scipy
    b_coeffs, a_coeffs = signal.butter(filter_order, normalized_cutoff, btype='lowpass', analog=False)

    # State storage for the filters using deques
    # We need N previous inputs and N previous outputs for an Nth order filter
    # Input buffer stores: x[n], x[n-1], ..., x[n-N] (N+1 elements)
    # Output buffer stores: y[n-1], y[n-2], ..., y[n-N] (N elements needed for calculation)
    # Initialize with zeros
    input_buffer_x = deque([0.0] * (filter_order + 1), maxlen=(filter_order + 1))
    output_buffer_x = deque([0.0] * filter_order, maxlen=filter_order) # Store N previous outputs
    input_buffer_y = deque([0.0] * (filter_order + 1), maxlen=(filter_order + 1))
    output_buffer_y = deque([0.0] * filter_order, maxlen=filter_order) # Store N previous outputs

    def pd_controller(current_x, current_y, kp, kd, target_setpoint):
        '''
        Calculates the  desired plate tilt angles using PD controller.
        
        current_x: current ball position (X-axis)
        current_y: current ball position (Y-axis)
        kp: proportional gain
        kd: derivative gain
        target_setpoint: Point object (target_setpoint.x, target_setpoint.y)
        
        returns: tuple (x,y) desired tilt angles in radians
        '''
        # Allow modification of variables from the enclosing 'run_controller' scope
        nonlocal last_error_x, last_error_y

        # --- Proportional Term ---
        # Calculate the current error (difference between target and current position)
        error_x = target_setpoint.x - current_x
        error_y = target_setpoint.y - current_y

        # Calculate the proportional output (how much to correct based on current error)
        proportional_term_x = kp * error_x
        proportional_term_y = kp * error_y

        # --- Derivative Term ---
        # Calculate the change in error since the last time step
        change_in_error_x = error_x - last_error_x
        change_in_error_y = error_y - last_error_y

        # Approximate the derivative of the error (rate of change)
        # Divide the change in error by the time step (dt)
        error_derivative_x = change_in_error_x / dt
        error_derivative_y = change_in_error_y / dt

        # Calculate the derivative output (how much to correct based on rate of change)
        derivative_term_x = kd * error_derivative_x
        derivative_term_y = kd * error_derivative_y

        # --- Update State for Next Iteration ---
        # Store the current error to be used as 'last_error' in the next call
        last_error_x = error_x
        last_error_y = error_y

        # --- Combine Terms ---
        # Calculate the final control output (angle) by summing P and D terms
        output_angle_x = proportional_term_x + derivative_term_x
        output_angle_y = proportional_term_y + derivative_term_y

        # Return the calculated angles
        return output_angle_x, output_angle_y

    def filter_val(new_input, input_buffer: deque, output_buffer: deque):
        """Implement a filter here, you can use scipy.signal.butter to compute the filter coefficients and then scipy.signal.lfilter to apply the filter.but we recommend you implement it yourself instead of using lfilter because you'll have to do that on the real system later.
        Take a look at the butterworth example written by Renato for inspiration."""
        # Allow modification of coefficients defined in run_controller scope
        # (Technically not needed if only reading, but good practice if you might adapt coeffs later)
        nonlocal b_coeffs, a_coeffs

        # 1. Add the new input to the input buffer
        # input_buffer now holds [x[n], x[n-1], ..., x[n-N]]
        input_buffer.appendleft(new_input)

        # 2. Calculate the feedforward term (sum(b[i]*x[n-i]))
        # Ensure buffer has enough elements (it should due to initialization)
        feedforward_sum = sum(b_coeffs[i] * input_buffer[i] for i in range(filter_order + 1))

        # 3. Calculate the feedback term (sum(a[j]*y[n-j]))
        # output_buffer holds [y[n-1], y[n-2], ..., y[n-N]]
        # We need a[1]*y[n-1] + a[2]*y[n-2] + ... + a[N]*y[n-N]
        feedback_sum = sum(a_coeffs[j] * output_buffer[j-1] for j in range(1, filter_order + 1))

        # 4. Calculate the new output y[n] (assuming a[0]=1)
        new_output = feedforward_sum - feedback_sum

        # 5. Add the new output to the output buffer for the next iteration
        # output_buffer will now hold [y[n], y[n-1], ..., y[n-N+1]]
        output_buffer.appendleft(new_output)

        return new_output

    def every_10ms(i: int, t: float):
        '''This function is called every ms and performs the following:
        1. Get the measurement of the position of the ball
        2. Calculate the forces to be applied to the plate
        3. Apply the forces to the plate
        
        i: iteration count
        t: elapsed time form loop_every()
        '''
        # Queries PyBullet for the position of the object with the ID in world.sphere
        (x,y,z), orientation = p.getBasePositionAndOrientation(world.sphere) # returns x,y,z and orientation, we care about x and y, the rest is captured but not used

        if noise: # Checks if the noise flag is True
            x += utils.noise(t) # the noise added has a frequency between 30 and 50 Hz
            y += utils.noise(t, seed = 43) # so that the noise on y is different than the one on x
        
        if filtered: # Checks is the filtered flag is True
            x = filter_val(x, input_buffer_x, input_buffer_y) # filters the noise from the x coordinate
            y = filter_val(y, input_buffer_y, output_buffer_y) # filters the noise from the y coordinate

        (angle_x, angle_y) = pd_controller(x, y, kp, kd, setpoint) 
        set_plate_angles(angle_x, angle_y) # Sets angle of x and y based on pd_controller

        if i%10 == 0: # print every 100 ms
            print(f"t: {t:.2f}, x: {x:.3f},\ty: {y:.3f},\tax: {angle_x:.3f},\tay: {angle_y:.3f}") # ball position (x,y) plate angle (ax,ay)

    utils.loop_every(0.01, every_10ms) # we run our controller at 100 Hz using a linux alarm signal

def run_simulation( initial_ball_position = Point(np.random.uniform(-0.2, 0.2),
                                                  np.random.uniform(-0.2, 0.2))):
    '''
    Sets up the PyBullet environment
    
    initial_ball_position: (optional) provides the initial position of the ball, by default the ball starts at random between -0.2 and 0,2
    '''
    p.connect(p.GUI) # Function to open the window to see the simulation
    p.setAdditionalSearchPath("assets") # This tells PyBullet to look for asset files (ex: plate.urdf)
    plate = p.loadURDF("plate.urdf") # Loads the plate.urdf model into the sim
    
    sphere = p.createMultiBody( # Creates the sphere object directly without a URDF file, stores an ID
        0.2 # mass of the sphere
        , p.createCollisionShape(p.GEOM_SPHERE, radius=0.04) # defines collision as a sphere with radius 0.04
        , basePosition = [initial_ball_position.x,initial_ball_position.y,0.5] # Sets the initial position of the ball based on initial_ball_position()
    )

    #zoom to the plate
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0,0,0]) # sets the view of the camera in the GUI window

    p.setJointMotorControl2(plate, 0, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2) # initialize joint motor x axis
    p.setJointMotorControl2(plate, 1, p.POSITION_CONTROL, targetPosition=0, force=5, maxVelocity=2) # initialize joint motor y axis

    p.setGravity(0, 0, -9.8) # sets the gravitational acceleration (-9.8 m/s^2)

    #update the simulation at 100 Hz
    p.setTimeStep(0.01) # physics engine updates on 10 millisecond increments
    p.setRealTimeSimulation(1) # tries to synchronize the sim to clock
    return World(plate=plate, sphere=sphere) # returns the world object containing ID for plate and sphere


if __name__ == "__main__": # will only execute when you run pid.py, will not run of imported to another file
    cmd_args = parse_args() # Reads the command line args
    world = run_simulation() # sets up the PyBullet environment and stores the world object
    run_controller( # starts the actual control loop
        **vars(cmd_args), # converts the parsed command line args object with attribute into a dictionary, the ** unpacks it as args for this function
        world=world #  passes the world object containing the simulation IDs
        ) 
    time.sleep(10000) # script pauses to allow the control loop to run indefinitely
