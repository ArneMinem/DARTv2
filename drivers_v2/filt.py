
class LowPassFilter():

    def __init__(self): 
        self.a = 0.2  # here you define some useful variables
        self.PFV = 0
    
    def simpleLowPassFilter(self,v):
        """
        Add your filtering code here, now the filter does nothing
        """
        print('nouveau tour')
        vf = self.a*v + (1-self.a)*self.PFV
        self.PFV = vf
        return vf

class PID():

    def __init__(self, Kp, Ki, Kd, itirationTime):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.iterationTime = itirationTime
        self.priorErr = 0
        self.priorIntegral = 0

    def calculate(self, value, desiredvalue=0, bias=0):
        err = desiredvalue - value
        integral = self.priorIntegral + err * self.iterationTime
        derivative = (err - self.priorErr) / self.iterationTime
        coefco = self.Kp * err + self.Ki * integral + self.Kd * derivative + bias
        self.priorErr = err
        self.priorIntegral = integral
        return coefco