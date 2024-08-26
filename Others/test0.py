import serial

# Initialize Serial Communication with Arduino
arduino_serial = serial.Serial('COM5', 9600, timeout=1)

while True:
    if arduino_serial.in_waiting > 0:
        # Read data from serial port
        data = arduino_serial.readline().strip()
        print("Received:", data)