import machine
from machine import Pin, PWM
import math
import time

class Servo():
    def __init__(self, servo_pin:int, home_angle:float=90.0, freq=50, min_us:int=500, max_us:int=2500, max_angle:float=180.0, invert:bool=False):
        self.servo_pin = Pin(servo_pin, Pin.OUT)
        self.max_angle = max_angle
        self.home_angle = home_angle
        self.invert = invert  # Added invert option
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
        Sets the servo to the input angle (degrees).
        Handles inversion if self.invert is True.
        """
        # Constrain angle between 0 and max_angle
        angle = max(0, min(self.max_angle, angle))
        
        # Calculate pulse width based on inversion setting
        if self.invert:
            pulse_us = self._max_duty_us - (angle / self.max_angle) * (self._max_duty_us - self._min_duty_us)
        else:
            pulse_us = self._min_duty_us + (angle / self.max_angle) * (self._max_duty_us - self._min_duty_us)        

        self.servo.duty_u16(self._us_to_duty(pulse_us))
    
    def home(self):
        self.set_angle(self.home_angle)



class Leg():
    def __init__(self, s1_pin:int, s2_pin:int, s3_pin: int, invert_s1:bool=False, invert_s2:bool=False, invert_s3:bool=False):
        self.s1 = Servo(s1_pin, invert=invert_s1, home_angle=90.0)
        self.s2 = Servo(s2_pin, invert=invert_s2, home_angle=90.0)
        self.s3 = Servo(s3_pin, invert=invert_s3, home_angle=90.0)

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


    def _joint_angle_to_servo_angles(self, theta_1, theta_2, theta_3):
        s1_angle = theta_1
        s2_angle = theta_2
        
        # 2.58352758e-04 -3.83624709e-02  2.91070319e+00  3.98349650e+01
        s3_angle = 0.00028*theta_3**3 -0.038*theta_3**2 + 3*theta_3 + 40
        print(f"s3_angle: {s3_angle}")

        return s1_angle, s2_angle, s3_angle
    
    def forward_kinematics(self, theta_1:float , theta_2:float, theta_3:float):
        # Clipping
        theta_1 = min(self.theta_1_range[1], max(self.theta_1_range[0], theta_1))
        theta_2 = min(self.theta_2_range[1], max(self.theta_2_range[0], theta_2))
        theta_3 = min(self.theta_3_range[1], max(self.theta_3_range[0], theta_3))

        

        s1_angle, s2_angle, s3_angle = self._joint_angle_to_servo_angles(theta_1, theta_2, theta_3)

        # Setting the Angles
        self.s1.set_angle(self.theta_1_base)
        self.s2.set_angle(self.theta_1_base + s2_angle)
        self.s3.set_angle(self.theta_1_base + s3_angle)
    
    


    def inverse_kinematics_2d(self, x:float, z:float):
        D = (x*x + z*z - self.l2*self.l2 - self.l3*self.l3) / (2 * self.l2 * self.l3)
        D = max(min(D, 1), -1)
        t3 = math.acos(D)
        t2 = math.atan2(z, x) - math.atan2(
            self.l3 * math.sin(t3),
            self.l2 + self.l3 * math.cos(t3)
        )
        theta_1 = 0.0
        theta_2 = math.degrees(t2)
        theta_3 = math.degrees(t3)

        print(f"for (x, z)= ({x}, {z}), t1 = {theta_1}, t2 = {theta_2}, t3 = {theta_3}")

        s1_angle, s2_angle, s3_angle = self._joint_angle_to_servo_angles(theta_1, theta_2, theta_3)

        self.s1.set_angle(s1_angle)
        self.s2.set_angle(s2_angle)
        self.s3.set_angle(s3_angle)


class StepGenerator:
    """Generates the X, Z trajectory for a single leg with phase offset support."""
    def __init__(self, leg: Leg, step_length: float = 80.0, z_ground: float = 120.0, cycle_time_ms: int = 2000, phase_offset: float = 0.0):
        self.leg = leg
        self.step_length = step_length
        self.radius = step_length / 2.0
        self.z_ground = z_ground
        self.cycle_time_ms = cycle_time_ms
        self.phase_offset = phase_offset
        self.direction = 1 
        self.start_time = time.ticks_ms()

    def set_direction(self, forward: bool):
        self.direction = 1 if forward else -1

    def update(self):
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self.start_time)
        
        # 1. Get raw phase (0.0 to 1.0)
        raw_phase = (elapsed % self.cycle_time_ms) / self.cycle_time_ms
        
        # 2. Apply offset and wrap around using modulo 1.0
        phase = (raw_phase + self.phase_offset) % 1.0
        
        if phase < 0.5:
            # --- SWING PHASE: Semicircle in the air ---
            t_swing = phase * 2.0 
            angle = math.pi * (1.0 - t_swing)
            x = self.radius * math.cos(angle)
            z = self.z_ground - (self.radius * math.sin(angle))
        else:
            # --- STANCE PHASE: Straight line on the ground ---
            t_stance = (phase - 0.5) * 2.0
            x = self.radius - (self.step_length * t_stance)
            z = self.z_ground

        # Apply direction multiplier
        x = x * self.direction
        self.leg.inverse_kinematics_2d(x, z)


class QuadrupedController:
    """Manages all 4 legs to create synchronized gaits."""
    def __init__(self, leg_fl: Leg, leg_fr: Leg, leg_bl: Leg, leg_br: Leg, cycle_time_ms: int = 2000):
        # Pair A (Front-Left & Back-Right) start at 0% phase
        self.fl_stepper = StepGenerator(leg_fl, cycle_time_ms=cycle_time_ms, phase_offset=0.0)
        self.br_stepper = StepGenerator(leg_br, cycle_time_ms=cycle_time_ms, phase_offset=0.0)
        
        # Pair B (Front-Right & Back-Left) start at 50% phase
        self.fr_stepper = StepGenerator(leg_fr, cycle_time_ms=cycle_time_ms, phase_offset=0.5)
        self.bl_stepper = StepGenerator(leg_bl, cycle_time_ms=cycle_time_ms, phase_offset=0.5)

        self.all_steppers = [self.fl_stepper, self.fr_stepper, self.bl_stepper, self.br_stepper]

    def set_direction(self, forward: bool):
        """Changes the walking direction for all legs."""
        for stepper in self.all_steppers:
            stepper.set_direction(forward)

    def update(self):
        """Updates inverse kinematics for all 4 legs based on the current time."""
        for stepper in self.all_steppers:
            stepper.update()


if __name__ == "__main__":

    front_left  = Leg(s1_pin=1,  s2_pin=2,  s3_pin=3,  invert_s2=True, invert_s3=False)
    front_right = Leg(s1_pin=1,  s2_pin=12,  s3_pin=11,  invert_s2=False,  invert_s3=True)
    back_left   = Leg(s1_pin=1,  s2_pin=4,  s3_pin=5,  invert_s2=True, invert_s3=False)
    back_right  = Leg(s1_pin=10, s2_pin=8, s3_pin=9, invert_s2=False,  invert_s3=True)
    
    robot_dog = QuadrupedController(
        leg_fl=front_left, 
        leg_fr=front_right, 
        leg_bl=back_left, 
        leg_br=back_right, 
        cycle_time_ms=2500
    )
    
    back_right.inverse_kinematics_2d(0, 100)
    front_right.inverse_kinematics_2d(0, 100)
    front_left.inverse_kinematics_2d(0, 100)
    back_left.inverse_kinematics_2d(0, 100)
    time.sleep(1)
    print("Starting Quadruped Trot Gait...")
    robot_dog.set_direction(forward=False)
    
    try:
        while True:
            robot_dog.update()
            time.sleep_ms(10)
            
    except KeyboardInterrupt:
        print("Robot stopped safely.")