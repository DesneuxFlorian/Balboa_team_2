from Lidar import Lidar
import signal
import numpy as np
import paho.mqtt.client as mqtt
import json
import serial,time
#sudo pigpiod


arduino=serial.Serial("/dev/ttyACM0", 230400, timeout=1)
test=Lidar(17,27,0x30,arduino)
test.start(1,50,0)

def on_connect(client, userdata, flags, rc):
    client.subscribe("raspberry/setpoint")
    client.subscribe("raspberry/scanningStatus")
    print(f"Connected with result code {rc}")
def on_message(client,userdata,msg):
    #print(f"{msg.topic}{msg.payload}")
    global scanningStatus
    if msg.topic=="raspberry/setpoint":
        data=msg.payload.decode("utf-8")
        try:
            SP="{SP:}\n".format(SP=data)
            arduino.write(SP.encode())
        except:
            print("error")
    elif msg.topic=="raspberry/scanningStatus":
        scanningStatus=msg.payload.decode("utf-8")
        msg="R\n"
        arduino.write(msg.encode())#Reset arduino odometry
        

client = mqtt.Client()
client.on_connect = on_connect
client.on_message=on_message
#client.connect("192.168.0.144", 1883, 60)#papa
client.connect("10.10.10.10", 1883, 60)#hotspot
#client.connect("192.168.1.54", 1883, 60)#Kot
#client.connect("192.168.0.88", 1883, 60)#maman
def exit_handler(signal, frame):
    global running
    running = False  
    test.stop()
    time.sleep(5)



def sendData(distance,angle,odom):
    MQTT_MESSAGE1=json.dumps([distance.tolist(),angle.tolist()])
    MQTT_MESSAGE2=json.dumps(odom)
    # the four parameters are topic, sending content, QoS and whether retaining the message respectively
    client.publish('raspberry/scan', payload=MQTT_MESSAGE1, qos=0, retain=False)
    client.publish('raspberry/odom', payload=MQTT_MESSAGE2, qos=0, retain=False)





# Attach a signal handler to catch SIGINT (Ctrl+C) and exit gracefully
time.sleep(0.1) #wait for serial to open
signal.signal(signal.SIGINT, exit_handler)


msg="R\n"
arduino.write(msg.encode())#Reset arduino odometry
running = True
scanningStatus=False
i=0
print("CTRL+C to exit")
client.loop_start()
while running:
    if scanningStatus=="True":
        #print("NEXT SCAN")
        x,y,yaw=test.getOdom();
        angle,distance=test.scan_Around(120,20)#deg et mm
        sendData(distance,angle,[x,y,yaw])
    else:
        pass
        #print("WAITING")
    

client.loop_stop()
