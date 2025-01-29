import RPi.GPIO as GPIO
import time
import os
import pickle
import cv2
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
from smbus2 import SMBus
import subprocess  # For running AddDatabase.py script

# GPIO Pins for Servo, LED, Buzzer, and Buttons
SERVO_PIN = 12 
BUZZER_PIN = 21
RED_LED_PIN = 17
GREEN_LED_PIN = 9
TRIG_PIN = 18
ECHO_PIN = 24
BUTTON_PIN_23 = 23  # Button to start AddDatabase.py
BUTTON_PIN_7 = 7    # Button to delete database

# LCD I2C Configuration
I2C_ADDR = 0x27  # Adjust based on your LCD's address
LCD_WIDTH = 16   # 16 characters per line
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80  # LCD memory location for the 1st line
LCD_LINE_2 = 0xC0  # LCD memory location for the 2nd line

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN_23, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to start AddDatabase.py
GPIO.setup(BUTTON_PIN_7, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Button to delete database

# Initialize GPIO for ultrasonic sensor (TRIG_PIN as OUTPUT, ECHO_PIN as INPUT)
GPIO.setup(TRIG_PIN, GPIO.OUT)  # Set TRIG_PIN as OUTPUT
GPIO.setup(ECHO_PIN, GPIO.IN)   # Set ECHO_PIN as INPUT

# Initialize I2C bus for LCD
bus = SMBus(1)

# LCD Functions
def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | 0x08
    bits_low = mode | ((bits << 4) & 0xF0) | 0x08
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, bits | 0x04)
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, bits & ~0x04)
    time.sleep(0.0005)

def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.0005)

def lcd_display(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for char in message:
        lcd_byte(ord(char), LCD_CHR)

# Function to delete database
def delete_database():
    try:
        if os.path.exists("dataset"):
            os.system("rm -rf dataset")  # Remove the entire dataset folder
            if os.path.exists("encodings.pickle"):
                os.remove("encodings.pickle")  # Delete encodings file
            lcd_display("Database deleted", LCD_LINE_1)
            lcd_display("Restarting system", LCD_LINE_2)
            time.sleep(3)
            reboot_system()  # Reboot system to reload fresh database
    except Exception as e:
        print(f"Error deleting database: {e}")
        lcd_display("Error deleting DB", LCD_LINE_1)
        lcd_display(str(e), LCD_LINE_2)
        time.sleep(2)

# Function to reboot the system
def reboot_system():
    os.system("sudo reboot")
    
# Function to set the angle of the servo
def set_servo_angle(angle):
    duty_cycle = (angle / 18) + 2  # Formula to convert angle to duty cycle
    GPIO.output(SERVO_PIN, True)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency for PWM
    pwm.start(duty_cycle)
    time.sleep(1)
    pwm.stop()
    

# Function to start the AddDatabase script
def start_add_database():
    try:
        subprocess.run(["python3", "AddDatabase.py"])  # Run AddDatabase.py
    except Exception as e:
        print(f"Error starting AddDatabase.py: {e}")
        lcd_display("Error starting DB", LCD_LINE_1)
        lcd_display("AddDatabase.py", LCD_LINE_2)
        time.sleep(2)

# Function to measure distance
def measure_distance():
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.1)
    
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)
    
    pulse_start = time.time()
    pulse_end = time.time()
    
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Calculate distance in cm
    distance = round(distance, 2)
    
    return distance

# Initialize facial recognition variables
currentname = "Person1"
encodingsP = "encodings.pickle"
vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
fps = FPS().start()

# Load the known faces and embeddings
print("[INFO] loading encodings + face detector...")
data = pickle.loads(open(encodingsP, "rb").read())

# Main loop
try:
    lcd_init()
    lcd_display("System is", LCD_LINE_1)
    lcd_display("Starting...", LCD_LINE_2)
    time.sleep(2)

    door_open = False
    last_face_time = time.time()

    while True:
        # Measure distance before proceeding with face detection
        distance = measure_distance()
        print(f"Distance: {distance} cm")

        if distance < 40:
            frame = vs.read()
            frame = imutils.resize(frame, width=500)

            boxes = face_recognition.face_locations(frame)
            encodings = face_recognition.face_encodings(frame, boxes)
            names = []

            face_detected = False

            for encoding in encodings:
                matches = face_recognition.compare_faces(data["encodings"], encoding)
                name = "Unknown"  

                if True in matches:
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}

                    for i in matchedIdxs:
                        name = data["names"][i]
                        counts[name] = counts.get(name, 0) + 1

                    name = max(counts, key=counts.get)

                    if name == "Person2" and not door_open:
                        print(f"[INFO] {name} detected. Opening the door.")
                        lcd_display(f"{name} detected", LCD_LINE_1)
                        set_servo_angle(90)  
                        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)  
                        time.sleep(0.5)
                        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                        door_open = True
                        
                    elif name == "Person3" and not door_open:
                        print(f"[INFO] {name} detected. Opening the door.")
                        lcd_display(f"{name} detected", LCD_LINE_1)
                        set_servo_angle(90)  
                        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)  
                        time.sleep(0.5)
                        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                        door_open = True    

                if name != "Unknown":
                    face_detected = True

                names.append(name)

            # Update the LCD with the recognized face
            for ((top, right, bottom, left), name) in zip(boxes, names):
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 225), 2)
                y = top - 15 if top - 15 > 15 else top + 15
                cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if not face_detected:
                if door_open and time.time() - last_face_time > 3:
                    print("[INFO] No face detected. Closing the door.")
                    lcd_display("No face detected", LCD_LINE_1)
                    set_servo_angle(0)  
                    GPIO.output(GREEN_LED_PIN, GPIO.LOW)
                    door_open = False
            else:
                last_face_time = time.time()

            if "Unknown" in names:
                print("[WARNING] Unrecognized person detected!")
                lcd_display("Unrecognized", LCD_LINE_1)
                lcd_display("person detected", LCD_LINE_2)
                GPIO.output(RED_LED_PIN, GPIO.HIGH)
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(5)
                GPIO.output(RED_LED_PIN, GPIO.LOW)
                GPIO.output(BUZZER_PIN, GPIO.LOW)

            cv2.imshow("Facial Recognition", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            fps.update()

        else:
            print("[INFO] Person is too far away.")
            lcd_display("Person too far", LCD_LINE_1)
            lcd_display("away", LCD_LINE_2)

        # Check button presses
        if GPIO.input(BUTTON_PIN_23) == GPIO.LOW:  
            lcd_display("Starting AddDB", LCD_LINE_1)
            lcd_display("Process...", LCD_LINE_2)
            start_add_database()

        if GPIO.input(BUTTON_PIN_7) == GPIO.LOW:
            lcd_display("Hold to delete", LCD_LINE_1)
            lcd_display("Database", LCD_LINE_2)
            press_start_time = time.time()
            while GPIO.input(BUTTON_PIN_7) == GPIO.LOW:
                if time.time() - press_start_time > 5:
                    delete_database()
                    break
                time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[INFO] Exiting program.")
finally:
    fps.stop()
    cv2.destroyAllWindows()
    vs.stop()
    GPIO.cleanup()
