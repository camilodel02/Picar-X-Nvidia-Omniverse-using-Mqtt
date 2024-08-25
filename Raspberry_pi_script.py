import paho.mqtt.client as mqtt
from picarx import Picarx
from vilib import Vilib
import time
import math

# Picar-X Setup
px = Picarx()
#Vilib.camera_start()
#Vilib.display()
# MQTT Broker Details
broker_address = "localhost" 
broker_port = 1883  # Default MQTT port
sensor_data_topic = "picarx/distance"  
steering_topic = "picarx/steering" 
motor_topic="picarx/motor"
pan_topic="picarx/pan"
tilt_topic="picarx/tilt"
gray_scale_data_topic="picarx/grayscale"
steering_angle_topic="picar/field"
back_speed_topic="picarx/speed"
current_steering_angle_topic="usd/steering" #Topic to control steering angle of usd model
current_speed_topic="usd/speed" #Topic to control the back tires speed rotation of usd model
current_pan_angle_topic="usd/pan" #Topic to control the pan angle of usd model
current_tilt_angle_topic="usd/tilt" #Topic to control the tilt angle of usd model


# MQTT Callback Functions
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(steering_topic)  # Subscribe to steering topic
        client.subscribe(motor_topic) #Subscribe to motor topic
        client.subscribe(pan_topic)
        client.subscribe(steering_angle_topic)
        client.subscribe(back_speed_topic)
        client.subscribe(tilt_topic)
    else:
        print(f"Failed to connect, return code {rc}")
#Control commands functions

def on_message(client, userdata, msg):
    if msg.topic==pan_topic:
        pan_angle=max(-30,min(30,int(float(msg.payload.decode()))))
        px.set_cam_pan_angle(pan_angle)
        print(f"Recieved pan angle : {pan_angle}")
        client.publish(current_pan_angle_topic,str(pan_angle)) #publish current pan angle
        
    elif msg.topic==steering_angle_topic:
        field_angle=max(-30, min(30, int(float(msg.payload.decode()))))
        print(f"field angle: {field_angle}")
        px.set_dir_servo_angle(field_angle)
        client.publish(current_steering_angle_topic,str(field_angle)) #publish current steering angle
        
    elif msg.topic==back_speed_topic:
        field_speed=max(-100, min(100, int(float(msg.payload.decode()))))
        if field_speed>0:
            px.forward(field_speed)
            print(f"Forward speed: {field_speed}")
            tire_vel=wheel_vel(field_speed)
            print(f"wheel velocity: {tire_vel}")
        elif field_speed<0:
            px.forward(field_speed)
            print(f"Backward speed: {field_speed}")
            tire_vel=wheel_vel(field_speed)
        elif field_speed==0:
            tire_vel=0
            px.stop()
            print(f"Stopped")
        client.publish(current_speed_topic,str(tire_vel)) #publish current speed of the back tires

            
    elif msg.topic==tilt_topic:
        tilt_angle=max(-12,min(48,int(float(msg.payload.decode()))))
        px.set_cam_tilt_angle(tilt_angle)
        print(f"Received tilt angle command: {tilt_angle}")
        client.publish(current_tilt_angle_topic,str(tilt_angle)) #publish current tilt angle

    else:
        print(f"Received message on topic: {msg.topic}, payload: {msg.payload.decode()}")  # Print other messages for debugging

# Distance Data Function
def sensor_data():
    distance_front = round(px.ultrasonic.read(), 2)
    time.sleep(1)  # Adjust sleep duration as needed
    return distance_front
# Gray Scale data function
# Gray Scale data function (Corrected)
def gray_scale_data():
    try:
        gm_val_list = px.get_grayscale_data()
        #print("list:", gm_val_list)
        # Check if the data is valid and return a comma-separated string
        if gm_val_list is not None and isinstance(gm_val_list, list):
            grayscale_data_str=",".join(map(str, gm_val_list))
            #print("str gray scale:", grayscale_data_str)
            return grayscale_data_str
        else:
            print("Invalid grayscale data received from the sensor.")
            return None  # Return None to indicate an error
    except Exception as e:
        print(f"Error getting grayscale data: {e}")
        return None
def wheel_vel(pwm_duty_cycle):
    pwm=float(pwm_duty_cycle)
    battery_v=5
    av_volt=battery_v*(pwm/100.0)
    print(f"av: {av_volt}")
    rpm=41.2*av_volt*(88.0/103.0) #41.2 turns per volt of the motor
                                  #88/103 rpm of the motor per rpm of the tires
    ang_vel=float(rpm*2*math.pi/60)
    #tire_rad=0.03
    #gear_ratio=1/48
    #calculate linear velocity
    #lin_vel=ang_vel*tire_rad*gear_ratio
    return ang_vel

# MQTT Client Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_address, broker_port)  # Connect to broker

# Main Loop (Publishes sensor data regularly, handles MQTT in the background)
while True:
    str_msg = str(sensor_data())
    grayscale_data_str = str(gray_scale_data())
    #print("loop", grayscale_data_str)
    #client.publish(gray_scale_data_topic, grayscale_data_str)
    #print (str(sensor_data()))
    #client.publish(sensor_data_topic, str_msg)
    client.loop(0.1)   # Check for incoming messages
