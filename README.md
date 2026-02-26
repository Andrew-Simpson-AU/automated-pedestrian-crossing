The "CameraWebServer.ino" file needs to be flashed to the ESP32 board. 
The WiFi credentials that are being used should be filled in on lines 13 and 14. 
This WiFi network must be a private, 2.4 GHz network.

The "yolo_pedestrian_detection.py" file should be downloaded somewhere on your computer's C drive. 
You will likely need to change lines 7 and 8 to the IP address of your board that was output from the CameraWebServer.ino file. 
Using command prompts, navigate to the directory it is saved in. 
Then type the command "python yolo_pedestrian_detection.py". 
This should begin the pedestrian detection connected to the camera. 
Make sure your computer is connected to the same network as the board is. 
When finished, type "q" into the command prompt, and the stream will end.
