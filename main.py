import machine
from machine import Pin, PWM
import math


class Servo():
    def __init__(self, servo_pin:int, home_angle:float=0, freq=50, min_us:int=500, max_us:int=2500, max_angle:float=180.0):
        self.servo_pin = Pin(servo_pin, Pin.OUT)
        self.max_angle = max_angle
        self.home_angle = home_angle
        self.servo = PWM(self.servo_pin)

        self._frequency = freq
        self.servo.freq(self._frequency)

        self.period_us = 1000000 // self._frequency

        self._min_duty_us = min_us
        self._max_duty_us = max_us
    
    def _us_to_duty(self, us):
        """Internal helper: Converts microseconds to a 16-bit duty cycle."""
        return int((us / self.period_us) * 65535)

    def set_angle(self, angle:float):
        """
        Set's the input angle (degrees)
        """
        angle = max(0, min(self.max_angle, angle))
        
        pulse_us = self.min_us + (angle / self.max_angle) * (self.max_us - self.min_us)        

        self.servo.duty_u16(self._us_to_duty(pulse_us))
    
    def home(self):
        self.set_angle(self.home_angle)



class Leg():
    def __init__(self, s1_pin:int, s2_pin:int, s3_pin:int):
        self.s1 = Servo(s1_pin)
        self.s2 = Servo(s2_pin)
        self.s3 = Servo(s3_pin)

        self.theta_1_base = 0.0
        self.theta_2_base = 0.0
        self.theta_3_base = 0.0

        self.theta_1_range = (-15.0, 15.0)
        self.theta_2_range = (-45.0, 45.0)
        self.theta_3_range = (0.0, 90.0)
        
        self.home_angles = [0.0, 20.0, 40.0]
        self.current_angles = self.home_angles.copy()

        self.l1 = 43.0
        self.l2 = 70.0
        self.l3 = 70.0
    
    def forward_kinematics(self, theta_1:float , theta_2:float, theta_3:float):
        # Clipping
        theta_1 = min(self.theta_1_range[1], max(self.theta_1_range[0], theta_1))
        theta_2 = min(self.theta_2_range[1], max(self.theta_2_range[0], theta_2))
        theta_3 = min(self.theta_3_range[1], max(self.theta_3_range[0], theta_3))

        # Setting the Angles
        self.s1.set_angle(self.theta_1_base + theta_1)
        self.s2.set_angle(self.theta_1_base + theta_2)
        self.s3.set_angle(self.theta_1_base + theta_3)
    
    def inverse_kinematics_2d(self, x:float, y:float):

        # Calculating the angles
        c = math.sqrt(x**2 + y**2)
        theta_1 = self.theta_1_base
        theta_2 = math.acos((self.l3**2 - c**2 - self.l2**2) / -2*c*self.l2)
        theta_3 = math.acos((x**2 + y**2 - self.l2**2 - self.l3**2)/2*self.l2*self.l3)

        self.current_angles = [theta_1, theta_2, theta_3]
    
        # Clipping
        theta_1 = min(self.theta_1_range[1], max(self.theta_1_range[0], theta_1))
        theta_2 = min(self.theta_2_range[1], max(self.theta_2_range[0], theta_2))
        theta_3 = min(self.theta_3_range[1], max(self.theta_3_range[0], theta_3))

        # Setting the Angles
        self.s1.set_angle(self.theta_1_base + theta_1)
        self.s2.set_angle(self.theta_1_base + theta_2)
        self.s3.set_angle(self.theta_1_base + theta_3)

