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