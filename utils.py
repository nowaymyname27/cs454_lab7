import signal # Allows the script to interact with OS signals
import time # Functions to work with system time
import numpy as np # Used for working with numbers in Python
from typing import Callable, Union # Allows for type hints for function args and return vals

start_time = 0 # Stores time stamp
i = 0 # Counter for every call of loop_every function
def loop_every(interval: float, callback: Callable[[int, float], None]):
    """Call the callback every interval seconds.
    The callback will be called with two arguments:
    1. interval: The number of times the callback has been called.
    2. callback (like a function that can be used as an arg): The time elsapsed since the first call.
    and the time since the first call.
    """
    def handler(signum, frame):
        """
        handles the signal(signum)
        signum: receive the signal number that triggered the handler
        frame: receives current execution stack frame at the moment 
        the signal was delivered
        """
        global i, start_time # accesses the global vars declared above
        if i == 0: # Only true the first time the handler gets executed
            start_time = time.time() # Stores current time
        elapsed_time = time.time() - start_time # Calculates elapsed time since the handler firs ran
        callback(i, elapsed_time) # Calls the callback function and passes the counter and elapsed time as args
        i += 1 # increment global counter
    signal.signal(signal.SIGALRM, handler) # Registers handler to trigger for the SIGALRM signal
    signal.setitimer(signal.ITIMER_REAL, interval, interval) # Sets up an internal timer


def noise( t: Union[float, np.ndarray], min_freq: float = 30.0, max_freq: float = 50.0, num_waves: int = 30, seed=42):
    """ Generate a random noise signal, and evaluate it at time t in seconds. 
        t can be a float or a numpy array
        The noise signal is a sum of sinusoids with random frequencies, 
        amplitudes and phases.
        
        t: time point(s) when to calculate noise vals
        min_freq: minimum frequency for the sine wave components (30hz)
        max_freq: maximum frequency for the sine wave components (50hz)
        num_waves: number of sine waves to sum (default: 30)
        seed: fixed seed for random number generator
    """
    rng = np.random.default_rng(seed) # creates a Numpy random number generator instance (an object, ew I hate objects)
    freqs = rng.uniform(min_freq, max_freq, size=num_waves) # Uses rng object to generate uniform numbers between min and max freq
    amplitudes = rng.uniform(0.0, 0.01, size=num_waves) # Uses rng object to generate aplitudes, between 0.0 and 0.1
    phases = rng.uniform(0, 2*np.pi, size=num_waves) # Uses rng object to generate phases, between 0 and 2*pi
    angles = np.multiply.outer(freqs*2*np.pi, t).T + phases # Calculate where each wave is in it's cycle
    return np.sum(amplitudes*np.sin(angles), axis=-1) # returns the noise value(s)
