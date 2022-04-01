'''
import RPi.GPIO as GPIO
import time

tilt_servoPIN = 17
pan_servoPIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(tilt_servoPIN, GPIO.OUT)
GPIO.setup(pan_servoPIN, GPIO.OUT)

tilt_pwm = GPIO.PWM(tilt_servoPIN, 50) # GPIO 17 for PWM with 50Hz
pan_pwm = GPIO.PWM(pan_servoPIN, 50)

tilt_pwm.start(2.5) # Initialization
pan_pwm.start(2.5)

def SetAngle(type, angle):
  duty = angle / 18 + 2
  if type == 'tilt':
    tilt_pwm.ChangeDutyCycle(duty)
  elif type == 'pan':
    pan_pwm.ChangeDutyCycle(duty)
    
try:
  while True:
    for i in range(0, 180):
        SetAngle(type='pan', angle=i)
        SetAngle(type='tilt', angle=i)
        time.sleep(0.002)

except KeyboardInterrupt:
  tilt_pwm.stop()
  pan_pwm.stop()
  GPIO.cleanup()
'''

from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

def setAngle(type, angle):
    if angle >= 90:
        value = angle / 90 - 1
    elif angle < 90:
        value = - (1 - (angle/90))
    if type == 'tilt':
        tilt.value = value
    elif type == 'pan':
        pan.value = value

factory = PiGPIOFactory()

tilt = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
pan = Servo(27, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

tilt.mid()
pan.mid()

tilt.max()
sleep(1)
tilt.mid()
sleep(1)
tilt.min()
sleep(1)
pan.max()
sleep(1)
pan.mid()
sleep(1)
pan.min()
sleep(1)

# for i in np.linspace(-1,1,20,endpoint=False):
#     tilt.value = i
#     pan.value = i
#     print(i)
#     sleep(0.05)

for i in range (0, 180):
    setAngle(type='tilt',angle=i)
    setAngle(type='pan', angle=i)
    sleep(0.005)

tilt.mid()
pan.mid()