class Discrete_PID:
    """
    Adam just sent me this: https://www.scilab.org/discrete-time-pid-controller-implementation.
    Idk anymore mannn...
    """
    def __init__(self, sample_period, Kp=1, Ki=0, Kd=0, n=0):
        
        self.sample_period = sample_period
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.n = n
        
        self.prev_output1 = 0.
        self.prev_output2 = 0.
        
        self.prev_error1 = 0.
        self.prev_error2 = 0.
        
    
    def __call__(self, error):
        
        a0 = 1 + self.n * self.sample_period
        a1 = - (2 + self.n * self.sample_period)
        a2 = 1
        
        b0 = self.Kp * (1 + self.n * self.sample_period) + self.Ki * self.sample_period * (1 + self.n * self.sample_period) + self.Kd * self.n
        b1 = - (self.Kp * (2 + self.n * self.sample_period) + self.Ki * self.sample_period + 2 * self.Kd * self.n)
        b2 = self.Kp + self.Kd * self.n
        
        output = - (a1/a0) * self.prev_output1 - (a2/a0) * self.prev_output2 + (b0/a0) * error + (b1/a0) * self.prev_error1 + (b2/a0) * self.prev_error2

        self.prev_output2 = self.prev_output1
        self.prev_output1 = output
        self.prev_error2 = self.prev_error1
        self.prev_error1 = error

        return output
    