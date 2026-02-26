import cv2
from ultralytics import YOLO
import requests
import time

# ESP32-CAM stream URL and LED control URL
ESP32_STREAM_URL = "http://172.20.10.5/stream"
ESP32_LED_URL = "http://172.20.10.5/led"

print("Loading YOLO model...")
print("(First run will download the model - about 6MB - please wait)")

# Load YOLOv8 nano model (smallest, fastest version)
# The model will auto-download on first run
model = YOLO('yolov8n.pt')

print("YOLO model loaded!")
print("Connecting to ESP32-CAM stream...")
print(f"Stream URL: {ESP32_STREAM_URL}")

# Open the video stream
cap = cv2.VideoCapture(ESP32_STREAM_URL)

# Check if stream opened successfully
if not cap.isOpened():
    print("ERROR: Could not connect to ESP32-CAM stream")
    print("Make sure the ESP32-CAM is powered on and the URL is correct")
    exit()

print("Stream connected! Press 'q' to quit.")
print("\nDetecting pedestrians (class: person)...")
print("LED will turn ON when pedestrians detected, OFF when none detected")

frame_count = 0
last_boxes = []  # Store last detection results
led_state = False  # Track current LED state
last_led_change = 0  # Debounce LED changes

def control_led(state):
    """Send LED control command to ESP32"""
    global led_state, last_led_change
    
    # Debounce - only change LED state if it's been at least 0.5 seconds
    current_time = time.time()
    if current_time - last_led_change < 0.5:
        return
    
    # Only send command if state is actually changing
    if state != led_state:
        try:
            command = "on" if state else "off"
            response = requests.get(f"{ESP32_LED_URL}?state={command}", timeout=1)
            if response.status_code == 200:
                led_state = state
                last_led_change = current_time
                print(f"LED turned {command.upper()}")
        except Exception as e:
            # Silently ignore LED control errors to not interrupt detection
            pass

frame_count = 0
last_boxes = []  # Store last detection results

while True:
    # Read frame from stream
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame, trying to reconnect...")
        cap.release()
        cap = cv2.VideoCapture(ESP32_STREAM_URL)
        continue
    
    frame_count += 1
    
    # Resize frame for display
    frame_resized = cv2.resize(frame, (640, 480))
    
    # Only run YOLO every other frame to reduce lag (doubles speed)
    if frame_count % 2 == 0:
        # Resize to smaller resolution for faster YOLO processing
        frame_for_detection = cv2.resize(frame, (320, 240))
        
        # Run YOLO detection on smaller frame
        # conf=0.65 means 65% confidence threshold (stricter to reduce false positives)
        # classes=[0] means only detect "person" class (YOLO class 0)
        results = model(frame_for_detection, conf=0.65, classes=[0], verbose=False)
        
        # Get the detections and scale them back up to display size
        last_boxes = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get bounding box coordinates (from 320x240 frame)
                x1, y1, x2, y2 = box.xyxy[0]
                
                # Scale coordinates to display size (640x480)
                x1, y1, x2, y2 = int(x1*2), int(y1*2), int(x2*2), int(y2*2)
                
                # Get confidence score
                confidence = float(box.conf[0])
                
                last_boxes.append((x1, y1, x2, y2, confidence))
    
    # Draw the boxes (either new ones or last frame's boxes)
    pedestrian_count = len(last_boxes)
    
    # Control LED based on pedestrian count
    if pedestrian_count > 0:
        control_led(True)  # Turn LED ON when pedestrians detected
    else:
        control_led(False)  # Turn LED OFF when no pedestrians
    
    for (x1, y1, x2, y2, confidence) in last_boxes:
        # Draw bounding box
        cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Add label with confidence
        label = f'Person {confidence:.2f}'
        cv2.putText(frame_resized, label, (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display pedestrian count
    cv2.putText(frame_resized, f'Pedestrians: {pedestrian_count}', (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Show the frame
    cv2.imshow('ESP32-CAM YOLO Pedestrian Detection', frame_resized)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()

# Turn off LED before exiting
try:
    requests.get(f"{ESP32_LED_URL}?state=off", timeout=1)
    print("LED turned OFF")
except:
    pass

print("Program ended.")
print(f"YOLO successfully detected pedestrians!")
