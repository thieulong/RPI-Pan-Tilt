import cv2
import mediapipe as mp
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

tilt_servoPIN = 17
pan_servoPIN = 27

factory = PiGPIOFactory()

tilt = Servo(tilt_servoPIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
pan = Servo(pan_servoPIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

tilt.mid()
pan.mid()

pan_angle = 90
tilt_angle = 90

pan_range = 2
tilt_range = 2

def SetAngle(type, angle):
    if angle > 180: angle = 180
    if angle >= 90:
      value = angle / 90 - 1
      if type == 'tilt':
          tilt.value = value
          # print(value)
      if type == 'pan':
          pan.value = value
          # print(value)
    if angle < 90:
      value = - (1 - (angle/90))
      if type == 'tilt':
          tilt.value = value
          # print(value)
      if type == 'pan':
          pan.value = value
          # print(value)

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(2)

with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.7 ) as face_detection:
  
  while cap.isOpened():
    success, image = cap.read()
    
    window_width=image.shape[1]
    window_height=image.shape[0]
    # print("Window width: {}".format(window_width))
    # print("Window height: {}".format(window_height))
    
    window_center_x = int(window_width / 2)
    window_center_y = int(window_height / 2)
    
    cv2.line(img=image,
             pt1=(window_center_x + 20, window_center_y),
             pt2=(window_center_x - 20, window_center_y),
             color=(0, 255, 0),
             thickness= 2)
    
    cv2.line(img=image,
             pt1=(window_center_x, window_center_y + 20),
             pt2=(window_center_x, window_center_y - 20),
             color=(0, 255, 0),
             thickness= 2)
    
    if not success:
      print("Ignoring empty camera frame.")
      continue

    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = face_detection.process(image)

    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    if results.detections:
      for detection in results.detections:
            
        # face_center_x = int(mp_face_detection.get_key_point(
        #   detection, mp_face_detection.FaceKeyPoint.NOSE_TIP).x * window_width)
        # face_center_y = int(mp_face_detection.get_key_point(
        #   detection, mp_face_detection.FaceKeyPoint.NOSE_TIP).y * window_height)

        # print(face_center_x, face_center_y)
        
        # cv2.circle(image, 
        #            (face_center_x, face_center_y), 
        #            radius=2,
        #            color=(0, 0, 255),
        #            thickness= -1)
        
        location_data = detection.location_data
        
        if location_data.format == location_data.RELATIVE_BOUNDING_BOX:
            bb = location_data.relative_bounding_box
            bb_box = [
                int(bb.xmin * window_width), int(bb.ymin * window_height),
                int((bb.xmin + bb.width) * window_width), int((bb.ymin + bb.height) * window_height)
            ]
            
            cv2.rectangle(img = image,
                          pt1 = (bb_box[0], bb_box[1]),
                          pt2 = (bb_box[2], bb_box[3]),
                          color = (0, 0, 255),
                          thickness = 2)
    
        x_angle = pan_angle + pan_range    
        y_angle = tilt_angle + tilt_range 
        
        # print(window_center_x, int((bb.xmin + bb.width) * window_width))
        if window_center_x > int((bb.xmin + bb.width) * window_width):
          if x_angle > 180: 
            x_angle = 180
            pan_range = 88
          print("Turning right, Pan range = {}, x angle = {}".format(pan_range, x_angle))
          SetAngle(type = "pan", angle = x_angle)
          pan_range += 2
        
        # print(window_center_x, int(bb.xmin * window_width))
        if window_center_x < int(bb.xmin * window_width):
          if x_angle < 0: 
            x_angle = 0
            pan_range = -88
          print("Turning left, Pan range = {}, x angle = {}".format(pan_range, x_angle))
          SetAngle(type = "pan", angle = x_angle)
          pan_range -= 2
      
        # print(window_center_y, int((bb.ymin + bb.height) * window_height))
        if window_center_y > int((bb.ymin + bb.height) * window_height):
          if y_angle < 0: 
            y_angle = 0
            tilt_range = -88
          print("Turning up, Tilt range = {}, y angle = {}".format(tilt_range, y_angle))
          SetAngle(type = "tilt", angle = y_angle)
          tilt_range -= 2
          
        # print(window_center_y, int(bb.ymin * window_height))
        if window_center_y < int(bb.ymin * window_height):
          if y_angle > 180: 
            y_angle = 180
            tilt_range = 88
          print("Turning down, Tilt range = {}, y angle = {}".format(tilt_range, y_angle))
          SetAngle(type = "tilt", angle = y_angle)
          tilt_range += 2
          
    cv2.imshow('Face tracking', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
    
cap.release()
tilt.mid()
pan.mid()