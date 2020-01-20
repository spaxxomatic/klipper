from Tkinter import *

class MotorDrive:

    MIN_SPEED = 40
    MAX_SPEED = 1400
    
    def __init__(self, axis, step_increment=20):
        self.axis = axis
        self.speed = self.MIN_SPEED
        self.jogspeeddisplay = StringVar()
        self._update()
    
    def get_state(self):
        return {'speed':self.speed}
        
    def set_state(self, state):
        self.speed = state.get('speed', 100)
        self._update()
        
    def _update(self):
        self.jogspeeddisplay.set(str(self.speed))
    
    def inc_jog_speed(self, *args):
        self.speed += int(self.speed*0.1) 
        if (self.speed > self.MAX_SPEED): self.speed = self.MAX_SPEED
        self._update()
    
    def dec_jog_speed(self, *args):
        self.speed -= int(self.speed*0.1)
        if (self.speed < self.MIN_SPEED): self.speed = self.MIN_SPEED
        self._update()
    
    def jogup(self, *args):
        print "Jogup %s"%self.axis
    
    def jogdn(self, *args):
        print "Jogdn %s"%self.axis

xmotor = MotorDrive('X')
ymotor = MotorDrive('Y')
