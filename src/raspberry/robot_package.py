# Robot package built for my Thesis
# It includes all the necessary functions to control the robot
import cv2
import numpy as np
import time
from stereovision.calibration import StereoCalibration
import asyncio
import websockets
import base64
import RPi.GPIO as GPIO
import math
from mpu6050 import mpu6050
import requests
import os

# ==========================================
# ==================LOGS====================
# ==========================================

def http_log(msg, url = "http://127.0.0.1:8080/log"):
    """
    This function sends a log message to a specified URL.
    """
    try:
        requests.post(url, json={"msg":msg}, timeout = 0.2)
    except requests.exceptions.RequestException:
        pass	
	
# ==========================================
# =================CAMERA===================
# ==========================================

def initialize_camera():
	device = '/dev/video0'
	width, height = 1280, 360
	cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
	# Set properties
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
	cap.set(cv2.CAP_PROP_GAIN, 0)
	cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
	return cap, width, height

def take_picture(cap, path, width, height):
    max_attempts = 3
    os.makedirs(os.path.dirname(path), exist_ok=True)

    for attempt in range(1, max_attempts + 1):
        for _ in range(5):
            cap.grab()
        ok, frame = cap.read()

        if ok and frame is not None and frame.size:
            if not cv2.imwrite(path, frame):
                raise RuntimeError(f"Fallo al guardar imagen en {path}")
            left  = frame[:, :width // 2]
            right = frame[:,  width // 2:]

            left_gray  = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            return left_gray, right_gray, left
        time.sleep(0.1)

    raise RuntimeError("Unable to grab a valid frame")

# ==========================================
# ==================LLM=====================
# ==========================================

async def send_image(url, path):
	try:
		frame = cv2.imread(path)
		start_time = time.time()
		if frame is not None:
			_, buffer = cv2.imencode('.png', frame)
			img_base64 = base64.b64encode(buffer).decode()
            
			try:
					async with websockets.connect(url, ping_interval=None, ping_timeout=None) as websocket:
						await websocket.send(img_base64)
						response = await websocket.recv()
						end_time = time.time()
                    # tts = gTTS(text = response, lang = 'es', )
                    # tts.save("output.mp3")
						# os.system("mpg321 -r 200 output.mp3")
			except websockets.exceptions.ConnectionClosedError as e:
				http_log(f"Error de conexión: {e}")
			except Exception as e:
				http_log(f"Error al conectar con el servidor: {e}")
		else:
			http_log(f"Error: No se pudo leer la imagen desde {imagen_ruta}")
	except Exception as e:
		http_log(f"Error general: {e}")
    
	return response, end_time - start_time
	

# ==========================================
# ===============CONTROLLER=================
# ==========================================

class RobotController:
    def __init__(self, config=None):
        # Default configuration
        self.config = {
            'ref': 20,              # Speed reference
            'Kp_speed': 0.5,        # Proportional control constant
            'L': 0.15,              # Distance between wheels (m)
            'S': 20,                # Gaps of the crown of the encoder
            'WHEEL_DIAMETER': 64,   # mm
            'sleep_time': 0.1,     # Loop time in seconds
            
            # GPIO pins
            'encoder_pins': [26, 18, 19, 11],  # Encoders pins [E1, E2, E3, E4]
            'motor_pins': {
                'in1': 17, 'in2': 27,  # Left motor control pins
                'in3': 10, 'in4': 9,   # Right motor control pins
                'en_a': 4, 'en_b': 22  # Enable pins for PWM
            },
            'ultrasonic_pins' : {
				'trigger': 21,
				'echo': 20
			},
            
            # MPU6050 settings
            'mpu6050_address': 0x68
        }
        
        # Update with custom configuration if provided
        if config:
            self.config.update(config)
        
        # Initialize state variables
        self.reset_state()
        
        # Setup GPIO and sensors
        self._setup_gpio()
        self._setup_mpu6050()
    
    def reset_state(self):
        """Reset all state variables to their initial values."""
        self.n1 = self.n2 = self.n3 = self.n4 = 0  # Encoder pulse counters
        self.x, self.y, self.theta = 0, 0, 0       # Position and orientation
        self.theta_ref = math.radians(0)           # Reference orientation
        self.total_distance = 0                    # Total distance traveled
        self.target_distance = None                # Target distance to travel
        self.pwm_a_value = 50                      # Initial PWM values
        self.pwm_b_value = 50
        self.gyro_offsets = (0, 0, 0)              # Gyroscope calibration offsets
        
    def _setup_gpio(self):
        """Setup GPIO pins for motors and encoders."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        
        # Setup encoder pins
        encoder_pins = self.config['encoder_pins']
        GPIO.setup(encoder_pins, GPIO.IN)
        
        # Add event detection for encoders
        GPIO.add_event_detect(encoder_pins[0], GPIO.FALLING, 
                             callback=lambda ch: self._counter(ch, 1))
        GPIO.add_event_detect(encoder_pins[1], GPIO.FALLING, 
                             callback=lambda ch: self._counter(ch, 2))
        GPIO.add_event_detect(encoder_pins[2], GPIO.FALLING, 
                             callback=lambda ch: self._counter(ch, 3))
        GPIO.add_event_detect(encoder_pins[3], GPIO.FALLING, 
                             callback=lambda ch: self._counter(ch, 4))
        
        # Setup motor pins
        motor_pins = self.config['motor_pins']
        GPIO.setup([motor_pins['in1'], motor_pins['in2'], 
                   motor_pins['in3'], motor_pins['in4'], 
                   motor_pins['en_a'], motor_pins['en_b']], GPIO.OUT)
        
        # Setup ultrasonic pins
        ultrasonic_pins = self.config['ultrasonic_pins']
        GPIO.setup(ultrasonic_pins['trigger'], GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(ultrasonic_pins['echo'], GPIO.IN)
        
        # Setup PWM for motor control
        self.power_a = GPIO.PWM(motor_pins['en_a'], 100)
        self.power_b = GPIO.PWM(motor_pins['en_b'], 100)
        self.power_a.start(self.pwm_a_value)
        self.power_b.start(self.pwm_b_value)
    
    def _setup_mpu6050(self):
        """Setup MPU6050 sensor."""
        self.sensor = mpu6050(self.config['mpu6050_address'])
    
    def _counter(self, channel, n_encoder):
        """Count pulses detected by encoders."""
        if n_encoder == 1:
            self.n1 += 1
        elif n_encoder == 2:
            self.n2 += 1
        elif n_encoder == 3:
            self.n3 += 1
        else:
            self.n4 += 1
    
    def move_forward(self):
        """Set motors to move forward."""
        motor_pins = self.config['motor_pins']
        GPIO.output(motor_pins['in1'], GPIO.HIGH)
        GPIO.output(motor_pins['in2'], GPIO.LOW)
        GPIO.output(motor_pins['in3'], GPIO.HIGH)
        GPIO.output(motor_pins['in4'], GPIO.LOW)
    
    def stop(self):
        """Stop all motors."""
        motor_pins = self.config['motor_pins']
        GPIO.output(motor_pins['in1'], GPIO.LOW)
        GPIO.output(motor_pins['in2'], GPIO.LOW)
        GPIO.output(motor_pins['in3'], GPIO.LOW)
        GPIO.output(motor_pins['in4'], GPIO.LOW)
    
    def turn_left(self):
        """Set motors to turn left."""
        motor_pins = self.config['motor_pins']
        GPIO.output(motor_pins['in1'], GPIO.HIGH)
        GPIO.output(motor_pins['in2'], GPIO.LOW)
        GPIO.output(motor_pins['in3'], GPIO.LOW)
        GPIO.output(motor_pins['in4'], GPIO.HIGH)
        self.power_a.ChangeDutyCycle(100)
        self.power_b.ChangeDutyCycle(100)
    
    def turn_right(self):
        """Set motors to turn right."""
        motor_pins = self.config['motor_pins']
        GPIO.output(motor_pins['in1'], GPIO.LOW)
        GPIO.output(motor_pins['in2'], GPIO.HIGH)
        GPIO.output(motor_pins['in3'], GPIO.HIGH)
        GPIO.output(motor_pins['in4'], GPIO.LOW)
        self.power_a.ChangeDutyCycle(100)
        self.power_b.ChangeDutyCycle(100)
    
    def read_distance_cm(self):
        """Return the measured distance in centimetres."""
        ultrasonic_pins = self.config['ultrasonic_pins']
        # Send a 10 µs HIGH pulse on TRIG
        GPIO.output(ultrasonic_pins['trigger'], GPIO.HIGH)
        time.sleep(0.00001)          # 10 microseconds
        GPIO.output(ultrasonic_pins['trigger'], GPIO.LOW)

        # Wait for ECHO to go HIGH (start time)
        start = time.time()
        while GPIO.input(ultrasonic_pins['echo']) == 0:
             start = time.time()

        # Wait for ECHO to go LOW again (end time)
        end = time.time()
        while GPIO.input(ultrasonic_pins['echo']) == 1:
              end = time.time()

        # Pulse length × speed of sound (34300 cm/s) ÷ 2 (out and back)
        pulse_len = end - start
        distance = (pulse_len * 34300) / 2
        return distance
    
    def set_target_distance(self, distance_meters):
        """Set target distance to travel."""
        self.target_distance = distance_meters
        self.total_distance = 0
        http_log(f"Target distance set: {self.target_distance} m")
    
    def calibrate_gyro(self, samples=1000, delay=0.01):
        """Calibrate the gyroscope."""
        http_log("Calibrating gyroscope... Please don't move it.")
        offset_x = offset_y = offset_z = 0
        
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            offset_x += data['x']
            offset_y += data['y']
            offset_z += data['z']
            time.sleep(delay)
        
        offset_x /= samples
        offset_y /= samples
        offset_z /= samples
        
        self.gyro_offsets = (offset_x, offset_y, offset_z)
        http_log("Calibration done")
    
    def control_loop(self):
        """Main control loop for robot movement."""
        sleep_time = self.config['sleep_time']
        time.sleep(sleep_time)
        
        # Calculate the pulses of each equivalent wheel
        pulses_left = (self.n2 + self.n4) / (2 * sleep_time)
        pulses_right = (self.n1 + self.n3) / (2 * sleep_time)
        self.n1 = self.n2 = self.n3 = self.n4 = 0
        
        # Calculate the error and the control signal
        ref = self.config['ref']
        Kp_speed = self.config['Kp_speed']
        error_speed_left = ref - pulses_left
        error_speed_right = ref - pulses_right
        error_orientation = self.theta_ref - self.theta
        
        correction_left = (Kp_speed * error_speed_left)
        correction_right = (Kp_speed * error_speed_right)
        
        # Apply the control signal
        self.pwm_a_value = max(0, min(100, self.pwm_a_value + correction_right))
        self.pwm_b_value = max(0, min(100, self.pwm_b_value + correction_left))
        
        self.power_a.ChangeDutyCycle(self.pwm_a_value)
        self.power_b.ChangeDutyCycle(self.pwm_b_value)
        
        # Calculate the position and orientation
        wheel_diameter = self.config['WHEEL_DIAMETER']
        S = self.config['S']
        distance_left = (pulses_left / (2 * S)) * (math.pi * wheel_diameter / 1000)
        distance_right = (pulses_right / (2 * S)) * (math.pi * wheel_diameter / 1000)
        delta_distance = (distance_left + distance_right) / 2
        
        gyro_data = self.sensor.get_gyro_data()
        _, _, offset_z = self.gyro_offsets
        delta_theta = (gyro_data['z'] - offset_z) * sleep_time
        self.total_distance += delta_distance
        
        self.theta += delta_theta
        self.x += delta_distance * math.cos(math.radians(self.theta))
        self.y += delta_distance * math.sin(math.radians(self.theta))
        
        http_log(f"Distance traveled: {self.total_distance:.2f} m / {self.target_distance:.2f} m, angle: {self.theta:.2f}")
    
    def move_distance(self, distance, min_dist_cm=30):
        """Move forward a specified distance."""
        self.set_target_distance(distance)
        self.move_forward()
        while self.total_distance < self.target_distance:
            d = self.read_distance_cm()
            if d < min_dist_cm:
                http_log(f"Obstacle at {d:.1f} cm → STOP")
                break
            self.n1 = self.n2 = self.n3 = self.n4 = 0
            self.control_loop()
        self.stop()
    
    def turn_angle(self, angle):
        """Turn a specified angle in degrees."""
        target_angle = self.theta + angle
        
        if angle > 0:
            self.turn_right()
        else:
            self.turn_left()
        
        sleep_time = self.config['sleep_time']
        _, _, offset_z = self.gyro_offsets
        
        while abs(self.theta) < abs(target_angle):
            time.sleep(sleep_time)
            gyro_data = self.sensor.get_gyro_data()
            delta_theta = (gyro_data['z'] - offset_z) * sleep_time
            self.theta += delta_theta
            http_log(f"Current angle: {self.theta:.2f}° / Target: {target_angle:.2f}°")
        
        self.stop()
        http_log(f"Completed turn ({angle}°)")
    
    def explore(self):
        d = self.read_distance_cm()
        if d > 50:
            self.move_distance(1, 50)
        else:
            self.turn_angle(60)
				
    def cleanup(self):
        """Clean up GPIO resources."""
        self.stop()
        GPIO.cleanup()
        http_log("GPIO clean up")
		
