# subscriber.py
import paho.mqtt.client as mqtt
import numpy as np
import json
import matplotlib.pyplot as plt
import sys,signal,time
import keyboard
import lidar_to_grid_map_original as lidarPlot
from roboviz import MapVisualizer


def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # subscribe, which need to put into on_connect
    # if reconnect after losing the connection with the broker, it will continue to subscribe to the raspberry/topic topic
    client.subscribe("raspberry/scan")
    client.subscribe("raspberry/odom")

# the callback function, it will be triggered when receiving messages
def on_message(client, userdata, msg):
    global dist,angle,pose,scan#,s,ox,oy
    #print(f"{msg.topic} {msg.payload}")
    if msg.topic=="raspberry/scan":
        scanData=np.array(json.loads(msg.payload))
        dist=scanData[0]#m
        angle=scanData[1]#deg
        scan[0].append(dist.tolist())
        scan[1].append(angle.tolist())
        # s=s+1
        # if s>50:#only show last 50 scans
        #     #occupancy_map = np.ones((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS),np.uint8) *127
        #     ox=ox[40:]#delete first 40 elements that correspond to 1 scan
        #     oy=oy[40:]
        #     print(np.size(ox))

    elif msg.topic=="raspberry/odom":
        odom=np.array(json.loads(msg.payload))
        pose=np.array([odom[1]/1000+MAP_SIZE_METERS/2,-odom[0]/1000+MAP_SIZE_METERS/2,odom[2]])
        scan[2].append(pose.tolist())

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

def save():
    n=0
    while n<=100:
        try:
            with open("SCAN_TEST{}.txt".format(n), "x") as fp:
                    json.dump(scan, fp)
                    print("Done writing JSON data into .txt file")
                    return 0
        except FileExistsError:
            n+=1
        except:
             print("Problem writing JSON data into .txt file")
             return 1
    print("No available FileName writing JSON data into .txt file")
    return 1


def exit_handler(signal,frame):
    global running
    running=False
    print("stopping code")
    client.loop_stop()


signal.signal(signal.SIGINT,exit_handler)

# set the will message, when the Raspberry Pi is powered off, or the network is interrupted abnormally, it will send the will message to other clients
client.will_set('raspberry/status', b'{"status": "Off"}')

# create connection, the three parameters are broker address, broker port number, and keep-alive time respectively
#client.connect("192.168.0.144", 1883, 60)#papa
client.connect("10.10.10.10", 1883, 60)#hotspot
#client.connect("192.168.1.54", 1883, 60)#Kot
#client.connect("192.168.0.88", 1883, 60)#maman
# set the network loop blocking, it will not actively end the program before calling disconnect() or the program crash
running=True
scanning=False
client.loop_start()
i=0
j=0
s=0
MAP_SIZE_PIXELS = 250
MAP_SIZE_METERS = 2
map=MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, title='MapVisualizer',show_trajectory=True)
occupancy_map = np.ones((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS),np.uint8) *127
ox=np.empty([0,0])
oy=np.empty([0,0])
scan=[[],[],[]]

while running:
    if keyboard.is_pressed("i"):
        i+=1
        client.publish("raspberry/setpoint",payload="DF",qos=0,retain=False)
        print("send {data}".format(data="DF"))
    if keyboard.is_pressed("k"):
        i-=1
        client.publish("raspberry/setpoint",payload="DB",qos=0,retain=False)
        print("send {data}".format(data="DB"))
    if keyboard.is_pressed("j"):
        j+=1
        client.publish("raspberry/setpoint",payload="RCC",qos=0,retain=False)
        print("send {data}".format(data="RCC"))
    if keyboard.is_pressed("l"):
        j-=1
        client.publish("raspberry/setpoint",payload="RC",qos=0,retain=False)
        print("send {data}".format(data="RC"))
    if keyboard.is_pressed("c"):
        occupancy_map = np.ones((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS),np.uint8) *127
        ox=np.empty([0,0])
        oy=np.empty([0,0])
        angle=0
        dist=0
        scan=[[],[],[]]
        print("DATA AND MAP RESET")
    if keyboard.is_pressed("s"):
        save()
        print("DATA SAVED")
    if keyboard.is_pressed("t"):
        scanning= not(scanning)
        client.publish("raspberry/scanningStatus",payload=scanning,qos=0,retain=False)
        print("send {data}".format(data=scanning))
        time.sleep(.2)
    
    try:
        
        ox = np.sin(np.deg2rad(angle)) * dist/1000*MAP_SIZE_PIXELS/MAP_SIZE_METERS
        oy = np.cos(np.deg2rad(angle)) * dist/1000*MAP_SIZE_PIXELS/MAP_SIZE_METERS
        occupancy_map=\
        lidarPlot.generate_ray_casting_grid_map((pose[0]*MAP_SIZE_PIXELS/MAP_SIZE_METERS,pose[1]*MAP_SIZE_PIXELS/MAP_SIZE_METERS),ox, oy, dist,MAP_SIZE_PIXELS,occupancy_map, True)
        #print(occupancy_map)
        if not map.display(pose[0],pose[1],pose[2]+90,occupancy_map):
            exit(0)


    except Exception:
        #traceback.print_exc()
        pass
    time.sleep(.05)



print("code finished")
sys.exit(0)
