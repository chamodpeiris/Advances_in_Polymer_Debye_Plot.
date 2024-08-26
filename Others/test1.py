import cv2
import mediapipe as mp
import serial
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Initialize Serial Communication with Arduino
arduino_serial = serial.Serial('COM5', 9600, timeout=1)

# Variables to track hand gestures
last_gesture = None
motor_status = "OFF"  # Initially motor is off

# OpenCV Initialization
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        continue

    # Convert the image to RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image with MediaPipe Hands
    results = hands.process(image_rgb)

    # Get hand landmarks if detected
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw hand landmarks (dots)
            for lm in hand_landmarks.landmark:
                x, y = int(lm.x * image.shape[1]), int(lm.y * image.shape[0])
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)

            # Get landmarks for index and middle fingers
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y * image.shape[0]
            middle_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y * image.shape[0]

            # Check if index finger is lifted (gesture: pointing up)
            if index_tip < middle_tip:
                gesture = 'pointing up'
            else:
                gesture = None

            # Check if gesture has changed
            if gesture != last_gesture:
                if gesture == 'pointing up':
                    arduino_serial.write(b'o')  # Send command to Arduino to turn on stepper motor
                    motor_status = "ON"
                elif gesture is None:
                    arduino_serial.write(b'f')  # Send command to Arduino to turn off stepper motor
                    motor_status = "OFF"
                last_gesture = gesture

    # Show motor status in the frame
    cv2.putText(image, f"Stepper Motor: {motor_status}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # Show the image
    cv2.imshow('Hand Gesture Recognition', image)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()