import cv2
from ultralytics import YOLO
import socket
import time

# ESP32 settings
ESP32_CAMERA_IP = "192.168.1.160"  # ESP32 #1 - Camera
ESP32_LED_IP = "192.168.1.124"      # ESP32 #2 - LED Controller
ESP32_STREAM = f"http://{ESP32_CAMERA_IP}/stream"
TCP_PORT = 4210

print("Loading YOLO...")
model = YOLO('yolov8n.pt')
print("YOLO loaded!")

print(f"Connecting to stream: {ESP32_STREAM}")
cap = cv2.VideoCapture(ESP32_STREAM)

# Increase timeout for slower 15 MHz camera
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer
cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 60000)  # 60 second open timeout
cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 60000)  # 60 second read timeout

if not cap.isOpened():
    print("ERROR: Cannot connect to stream")
    exit()

print("Stream connected!")

# Connect to TCP server on ESP32 #2 (LED Controller)
print(f"Connecting to TCP server {ESP32_LED_IP}:{TCP_PORT}...")
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.settimeout(5)

try:
    tcp_socket.connect((ESP32_LED_IP, TCP_PORT))
    print("TCP connected to LED controller!")
except Exception as e:
    print(f"TCP connection failed: {e}")
    print("Continuing without LED control...")
    tcp_socket = None

print("\nDetecting pedestrians... Press 'q' to quit\n")

frame_count = 0
last_boxes = []
last_detection_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    
    frame_count += 1
    display = cv2.resize(frame, (640, 480))
    
    # Run YOLO every 3 frames (less often = less lag)
    if frame_count % 3 == 0:
        small = cv2.resize(frame, (320, 240))
        results = model(small, conf=0.30, classes=[0], verbose=False)  # Lowered to 30%
        
        last_boxes = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1*2), int(y1*2), int(x2*2), int(y2*2)
                conf = float(box.conf[0])
                last_boxes.append((x1, y1, x2, y2, conf))
        
        # Send TCP command if pedestrian detected
        if len(last_boxes) > 0:
            current_time = time.time()
            # Send every 1 second to keep LED on (LED has 15s timeout)
            if current_time - last_detection_time > 1.0:
                if tcp_socket:
                    try:
                        tcp_socket.send(b"ON\n")
                        response = tcp_socket.recv(1024)
                        print(f"✓ Pedestrian detected - LED ON")
                        last_detection_time = current_time
                    except socket.timeout:
                        print("✗ TCP timeout")
                    except Exception as e:
                        print(f"✗ TCP error: {e}")

    
    # Draw boxes
    for (x1, y1, x2, y2, conf) in last_boxes:
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(display, f'Person {conf:.2f}', (x1, y1-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.putText(display, f'Pedestrians: {len(last_boxes)}', (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    cv2.imshow('Pedestrian Detection', display)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
if tcp_socket:
    tcp_socket.close()
cap.release()
cv2.destroyAllWindows()
print("Done!")
