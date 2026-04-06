from machine import Pin, PWM
import math
import time

# ================= SERVO =================
class Servo:
    def __init__(self, pin, offset=0):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)
        self.offset = offset

    def write(self, angle):
        angle = angle + self.offset
        angle = max(0, min(180, angle))
        duty = int((500 + (angle / 180) * 2000) * 65535 / 20000)
        self.pwm.duty_u16(duty)

# ================= CONSTANTS =================
L1             = 7.0
L2             = 7.0
STEP_LENGTH    = 4
STEP_HEIGHT    = 2
DEFAULT_HEIGHT = -11

# ================= SERVOS =================
# FR — front right
hipFR  = Servo(2,  offset=120)
kneeFR = Servo(3,  offset=120)

# FL — front left
hipFL  = Servo(5,  offset=120)
kneeFL = Servo(6,  offset=80)

# RR — rear right
hipRR  = Servo(8, offset=140)
kneeRR = Servo(9, offset=140)

# RL — rear left
hipRL  = Servo(12, offset=60)
kneeRL = Servo(13, offset=120)

# ================= IK =================
def compute_ik_2d(x, z):
    D = (x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2)
    D = max(min(D, 1), -1)
    t2 = math.acos(D)
    t1 = math.atan2(z, x) - math.atan2(
        L2 * math.sin(t2),
        L1 + L2 * math.cos(t2)
    )
    return math.degrees(t1), math.degrees(t2)

# ================= MOVE =================
def move_FR(x, z):
    t1, t2 = compute_ik_2d(x, z)
    hipFR.write(90 + t1)
    kneeFR.write(90 - t2)

def move_FL(x, z):
    t1, t2 = compute_ik_2d(x, z)
    hipFL.write(90 + t1)
    kneeFL.write(90 - t2)

def move_RR(x, z):
    t1, t2 = compute_ik_2d(x, z)
    hipRR.write(90 + t1)
    kneeRR.write(90 - t2)

def move_RL(x, z):
    t1, t2 = compute_ik_2d(x, z)
    hipRL.write(90 + t1)
    kneeRL.write(90 - t2)

# ================= STAND =================
def stand():
    for _ in range(20):
        move_FR(0, DEFAULT_HEIGHT)
        move_FL(0, DEFAULT_HEIGHT)
        move_RR(0, DEFAULT_HEIGHT)
        move_RL(0, DEFAULT_HEIGHT)
        time.sleep(0.02)

# ================= TRAJECTORY =================
def gait_cycle(t, direction=1):
    x = direction * STEP_LENGTH * math.cos(t)
    z = DEFAULT_HEIGHT + STEP_HEIGHT * math.sin(t)
    return x, z

# ================= WALK =================
def walk(duration, direction):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration * 1000:
        for i in range(30):
            t = i * 0.2
            x1, z1 = gait_cycle(t, direction)
            x2, z2 = gait_cycle(t + math.pi, direction)
            # trot: FR+RL in phase, FL+RR opposite phase
            move_FR(x1, z1)
            move_RL(x1, z1)
            move_FL(x2, z2)
            move_RR(x2, z2)
            time.sleep(0.08)

# ================= MAIN =================
stand()
time.sleep(1)

while True:
    walk(5,  1)
    stand()
    time.sleep(1)
    walk(5, -1)
    stand()
    time.sleep(2)