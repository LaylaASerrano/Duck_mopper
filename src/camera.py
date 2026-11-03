from picamera2 import Picamera2
import time

cam = Picamera2()
cam.configure(picam2.create_still_configuration())
cam.start()
time.sleep(2)  # let auto-exposure settle
cam.capture_file("test.jpg")
cam.stop()
print("Saved test.jpg")
