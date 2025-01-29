
For the Raspberry Pi 4 GPIO configuration and wiring for the components in your code, here's a detailed breakdown for each component, including the servo motor wiring colors:

GPIO Pin Configuration:
Servo Motor (for controlling the door)

GPIO Pin: 12 (as defined in your code: SERVO_PIN = 12)
Wire Color:
Signal Wire (usually yellow/orange): Connected to GPIO Pin 12.
Power Wire (usually red): Connected to a 5V pin (Pin 2 or Pin 4).
Ground Wire (usually black or brown): Connected to a GND pin (Pin 6, Pin 9, or Pin 14).
Buzzer

GPIO Pin: 21 (as defined in your code: BUZZER_PIN = 21)
Wire Color:
Positive Wire (usually red): Connected to GPIO Pin 21.
Negative Wire (usually black): Connected to GND.
LEDs (Red and Green)

GPIO Pin for Red LED: 17 (as defined in your code: RED_LED_PIN = 17)
Wire Color:
Positive Wire (usually longer leg, red): Connected to GPIO Pin 17.
Negative Wire (shorter leg, black): Connected to GND (through a 220Ω resistor for protection).
GPIO Pin for Green LED: 9 (as defined in your code: GREEN_LED_PIN = 9)
Wire Color:
Positive Wire (longer leg, green): Connected to GPIO Pin 9.
Negative Wire (shorter leg, black): Connected to GND (through a 220Ω resistor for protection).
Buttons

GPIO Pin for Button to Start AddDatabase.py: 23 (as defined in your code: BUTTON_PIN_23 = 23)
GPIO Pin for Button to Delete Database: 7 (as defined in your code: BUTTON_PIN_7 = 7)
Wire Color:
Button Pins: For each button, one wire connects to the GPIO Pin (23 or 7), while the other wire connects to GND.
Ultrasonic Sensor (for distance measurement)

GPIO Pin for TRIG: 18 (as defined in your code: TRIG_PIN = 18)
GPIO Pin for ECHO: 24 (as defined in your code: ECHO_PIN = 24)
Wire Color:
TRIG Pin: Signal wire (usually yellow/orange) connects to GPIO Pin 18.
ECHO Pin: Signal wire (usually green/blue) connects to GPIO Pin 24.
VCC (Power): Usually red, connected to 5V (Pin 2 or Pin 4).
GND (Ground): Usually black, connected to GND (Pin 6 or Pin 9).
LCD (I2C) Display

I2C Address: 0x27 (as defined in your code: I2C_ADDR = 0x27)
Wire Color:
SDA Pin: (usually yellow or green) connects to the SDA (GPIO 2, Pin 3).
SCL Pin: (usually blue) connects to the SCL (GPIO 3, Pin 5).
VCC (Power): (usually red) connects to 5V (Pin 2 or Pin 4).
GND (Ground): (usually black) connects to GND (Pin 6).
