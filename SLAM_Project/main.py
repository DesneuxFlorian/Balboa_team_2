from Lidar import Lidar
import time
import signal



test=Lidar(13,0x30)
test.start(1,50,0)

def exit_handler(signal, frame):
    global running
    running = False  
    test.stop()
    time.sleep(1)
    

# Attach a signal handler to catch SIGINT (Ctrl+C) and exit gracefully
signal.signal(signal.SIGINT, exit_handler)

running = True

while running:
	d,d2=test.scan_Around()
	print("d1: {}	d2:{}".format(d,d2))
