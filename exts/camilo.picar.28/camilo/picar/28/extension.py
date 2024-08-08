import omni.ext
import omni.ui as ui
import omni.usd
import paho.mqtt.client as mqtt
import threading
import math
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, Usd
import omni.appwindow
import carb.input
from carb.input import KeyboardEventType
import omni.kit.viewport as viewport
import cv2


#class MJPEGStreamWidget(ui.Widget):
    #def __init__(self, url, **kwargs):
        #super().__init__(**kwargs)
        #self.url = url
        #self._image = None
        #self._fetch_thread = threading.Thread(target=self._fetch_stream)
       # self._fetch_thread.daemon = True
        #self._fetch_thread.start()

    #def _fetch_stream(self):
        #cap = cv2.VideoCapture(self.url)
        #while cap.isOpened():
           # ret, frame = cap.read()
            #if ret:
               # self._image = frame
               # self.rebuild()
            #else:
                #break
        #cap.release()

    #def rebuild(self):
        #if self._image is not None:
           # _, buffer = cv2.imencode('.png', self._image)
            #png_image = np.frombuffer(buffer, dtype=np.uint8)
            #image = ui.Image.from_numpy(png_image)
            #with self.frame:
               # ui.ImageWidget(image=image)




class CamiloPicarExtension(omni.ext.IExt):

    def load_picar_model(self):
        print("Loading model ....")
        #self._usd_context.open_stage("https://www.dropbox.com/scl/fi/dss5lr7g38rzxdcpjv8ul/picar_test_july_15.usd?rlkey=20ecfqdhkzthkc6v4bkdz8y0n&st=wjsm7r6t&dl=1") #cylinders as colliders
        #self._usd_context.open_stage("https://www.dropbox.com/scl/fi/zdzxxsx5imcdb1i8rik5s/picar_test_july_20.usd?rlkey=d5esyvr2k0zd95xs43h0gt24b&st=b76893ze&dl=1") #Better origins alligment
        #self._usd_context.open_stage("https://www.dropbox.com/scl/fi/oyw9rrd8dkz5f5l4lkoyv/picar_test_july_20_scaled.usd?rlkey=1fnjcb6znu95zpiho48anx6zy&st=5k1cmwji&dl=1") #Better scale
        self._usd_context.open_stage("https://www.dropbox.com/scl/fi/s8qe5nbvws8w6t5sr1u0d/picar_test_july_25_2_cars.usd?rlkey=ytnc6d9e0scvmeui7hnb5er7v&st=n9ken91w&dl=1") #2 virtual cars
    def on_startup(self):
        print("[camilo.picar] Camilo Picar startup")
        # MQTT Configuration
        self.broker_address = "10.186.149.198" #"10.186.103.156"  
        self.broker_port = 1883
        self.sensor_data_topic = "picarx/distance"
        self.pan_topic = "picarx/pan"
        self.tilt_topic = "picarx/tilt"
        self.gray_scale_data_topic="picarx/grayscale"
        self.steering_angle_topic="picar/field"
        self.back_speed_topic="picarx/speed"
        self.current_steering_angle_topic="usd/steering" #Topic to control steering angle of usd model
        self.current_speed_topic="usd/speed" #Topic to control the back tires speed rotation of usd model
        self.current_pan_angle_topic="usd/pan" #Topic to control the pan angle of usd model
        self.current_tilt_angle_topic="usd/tilt" #Topic to control the tilt angle of usd model


        self._usd_context = omni.usd.get_context()
        #Variables
        #steering update of usd
        self.left_wheel=None
        self.right_wheel=None
        self.current_steering_angle=0
        self.ang_vel=0.0
        self.current_pan_angle=0
        self.current_tilt_angle=0
        self.current_tire_rotation=0
        self.current_speed=0

        self._sub_stage_event=self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
        self.find_prims()


        self._app_update_sub=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_app_update_event, name="picar_steering._on_app_update_event"
        )
        #sensors
        self.sensor_data_value = ""
        self.gray_scale_data=" "
        self.grayscale_str=" "           
        
        # Create MQTT Client
        self.mqtt_client = mqtt.Client()
        self.connect_to_mqtt() 


        #Keyboard control
        self.app_window = omni.appwindow.get_default_app_window()
        self.keyboard = self.app_window.get_keyboard()
        self.input_interface = carb.input.acquire_input_interface()
        self.keyboard_sub_id = self.input_interface.subscribe_to_keyboard_events(self.keyboard, self.on_keyboard_input)

        #Video stream viewport
        #self.viewport_window=viewport.acquire_viewport_interface().get_viewport_window()
        #Use the HTTP stream directly
        #self.stream_url="http://10.186.103.156:9000/mjpg"
        #self.window2=ui.Window("Picar-X livestream", width=600, height=600)
        #with self.window2.frame:
             #ui.Image(source=self.stream_url)
        #with self.window2:
            #ui.Image(source=self.stream_url)

        #self.stream_url = "http://10.186.103.156:9000/mjpg"
        #self.window2 = ui.Window("Picar-X livestream", width=600, height=600)
       # with self.window2.frame:
            #MJPEGStreamWidget(url=self.stream_url, width=600, height=600)






        self._window = ui.Window("Picar-X Control", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Button("Load Picar-X usd model", clicked_fn=self.load_picar_model)
                
               #ui.Label("Steering angle (set an integer from -30 to 30)")
                #self.steering_angle_field=ui.IntField()
                #self.steering_angle_field.model.add_value_changed_fn(self.on_steering_angle_changed)
                
                #ui.Label("Motors speed % (integer between -100 to 100)")
                #self.speed_field=ui.IntField()
                #self.speed_field.model.add_value_changed_fn(self.on_back_speed_changed)

            
                #ui.Label("Pan angle control")
                #self.pan_field=ui.IntField()
                #self.pan_field.model.add_value_changed_fn(self.on_pan_changed)

                #ui.Label("Tilt Control")    
                #self.tilt_angle_blank=ui.IntField()
                #self.tilt_angle_blank.model.add_value_changed_fn(self.on_tilt_angle_changed)
                #ui.Button("Algorithm", clicked_fn=self.algorithm)

                #self.sensor_data_label = ui.Label(f"Distance Data: {self.sensor_data_value}")
        
    def on_keyboard_input(self, e):
        if e.type == KeyboardEventType.KEY_PRESS or e.type == KeyboardEventType.KEY_REPEAT:
            if e.input == carb.input.KeyboardInput.W:
                self.current_speed = min(100, self.current_speed + 5)
                self.send_speed_command(self.current_speed)
                print(f"Key pressed: {e.input}, Speed: {self.current_speed}")
            elif e.input == carb.input.KeyboardInput.S:
                self.current_speed = max(-100, self.current_speed - 5)
                self.send_speed_command(self.current_speed)
                print(f"Key pressed: {e.input}, Speed: {self.current_speed}")
            elif e.input == carb.input.KeyboardInput.A:
                self.current_steering_angle = max(-30, self.current_steering_angle - 7)
                self.send_servo_command(self.current_steering_angle)
                print(f"Key pressed: {e.input}, Steering Angle: {self.current_steering_angle}")
            elif e.input == carb.input.KeyboardInput.D:
                self.current_steering_angle = min(30, self.current_steering_angle + 7)
                self.send_servo_command(self.current_steering_angle)
                print(f"Key pressed: {e.input}, Steering Angle: {self.current_steering_angle}")
            elif e.input == carb.input.KeyboardInput.I:
                self.current_tilt_angle =min(48,self.current_tilt_angle +5 )
                self.send_tilt_command(self.current_tilt_angle)
                print(f"Key pressed: {e.input}, Tilt angle: {self.current_tilt_angle}")
            elif e.input==carb.input.KeyboardInput.K:
                self.current_tilt_angle=max(-12,self.current_tilt_angle -5)
                self.send_tilt_command(self.current_tilt_angle)
                print(f"Key pressed: {e.input}, Tilt angle: {self.current_tilt_angle}")
            elif e.input==carb.input.KeyboardInput.J:
                self.current_pan_angle=max(-30,self.current_pan_angle - 5)
                self.send_pan_command(self.current_pan_angle)
                print(f"Key pressed: {e.input}, Pan angle: {self.current_pan_angle}")
            elif e.input==carb.input.KeyboardInput.L:
                self.current_pan_angle=min(30,self.current_pan_angle + 5)
                self.send_pan_command(self.current_pan_angle)
                print(f"Key pressed: {e.input}, Pan angle: {self.current_pan_angle}")
            elif e.input == carb.input.KeyboardInput.SPACE:
                self.current_speed = 0
                self.current_steering_angle = 0
                self.current_pan_angle=0
                self.current_tilt_angle=0
                self.send_speed_command(self.current_speed)
                self.send_servo_command(self.current_steering_angle)
                self.send_pan_command(self.current_pan_angle)
                self.send_tilt_command(self.current_tilt_angle)
                print(f"Key pressed: {e.input}, Speed: {self.current_speed}, Steering Angle: {self.current_steering_angle}")


    #Functions that get the steering tires angle and update the value 
    def on_steering_angle_changed(self,model):
        try:
            new_angle=model.get_value_as_int()
            new_angle=max(-30,min(30,new_angle))
            self.send_servo_command(new_angle)
        except ValueError:
            print("Invalid angle, set an integer from -30 to 30")
    def send_servo_command(self,angle):
        str_angle=str(angle)
        self.mqtt_client.publish(self.steering_angle_topic,str_angle)

    #Functions that get the back tires speed and update the value
    def on_back_speed_changed(self,model):
        try:
            new_speed=model.get_value_as_int()
            new_speed=max(-100,min(100,new_speed))
            self.send_speed_command(new_speed)
        except ValueError:
            print("Invalid speed, set an integer from -100 to 100")
    def send_speed_command(self,speed):
        str_speed=str(speed)
        self.mqtt_client.publish(self.back_speed_topic,str_speed)

    #Functions to get the pan angle and update the value
    def on_pan_changed(self,model):
        try:
            new_pan=model.get_value_as_int()
            new_pan=max(-30,min(30,new_pan))
            self.send_pan_command(new_pan)
        except ValueError:
            print("Invalid pan angle, set an integer from -30 to 30")
    def send_pan_command(self,angle):
        str_pan=str(angle)
        self.mqtt_client.publish(self.pan_topic,str_pan)

    #Functions that get the tilt angle and update the value
    def on_tilt_angle_changed(self,model):
        try:
            new_tilt=model.get_value_as_int()
            new_tilt=max(-12,min(48,new_tilt))
            self.send_tilt_command(new_tilt)
        except ValueError:
            print("Invalid angle, set an integer from -12 to 48")
    def send_tilt_command(self,angle):
        str_tilt_angle=str(angle)
        self.mqtt_client.publish(self.tilt_topic,str_tilt_angle)

    #CONTROL OF USD MODEL FUNCTIONS
    def _on_stage_event(self,event):
        if event.type==int(omni.usd.StageEventType.OPENED):
            print("Opened new model")
            self.find_prims()
    def _on_app_update_event(self,evt):
        #ARTICULATION ROOT CONTROL
        #-----------------------------------CAR 1---------------------------------------------------------------------------------
        if self.articulation_root_api:
            ang_vel=0
            ang_vel=(self.ang_vel*360)/(2*math.pi) #Convertion factor between angular velocity from real car and angular velocity from usd stage units
            #lin_vel=self.ang_vel*0.3
            #print(f"Received angular velocity: {self.ang_vel} rad/s")
            #print(f"Linear velocity: {lin_vel} m/s ")
            #print(f"Angular velocity: {ang_vel} degrees/s")                                   
        #BACK TIRES CONTROL            
            if self.lbt_joint:
                lbt_drive=UsdPhysics.DriveAPI(self.lbt_joint,"angular")
                lbt_drive.GetTargetVelocityAttr().Set(10*ang_vel)
                #lbt_drive.GetDampingAttr().Set(100000)
            if self.rbt_joint:
                rbt_drive=UsdPhysics.DriveAPI(self.rbt_joint,"angular")
                rbt_drive.GetTargetVelocityAttr().Set(10*ang_vel)
                #rbt_drive.GetDampingAttr().Set(100000)
            
        #FRONT TIRES CONTROL
            if self.lft_joint:
                #print("found lft joint")
                lft_drive=UsdPhysics.DriveAPI(self.lft_joint,"angular")
                #print("found lft drive")
                lft_drive.GetTargetPositionAttr().Set(-self.current_steering_angle)
                lft_drive.GetDampingAttr().Set(1000)
                lft_drive.GetStiffnessAttr().Set(1000000)
            if self.rft_joint:
                rft_drive=UsdPhysics.DriveAPI(self.rft_joint,"angular")
                rft_drive.GetTargetPositionAttr().Set(-self.current_steering_angle)
                rft_drive.GetDampingAttr().Set(1000)
                rft_drive.GetStiffnessAttr().Set(1000000)
        #TILT CONTROL
            if self.tilt_joint:
                tilt_drive=UsdPhysics.DriveAPI(self.tilt_joint,"angular")
                tilt_drive.GetTargetPositionAttr().Set(-self.current_tilt_angle)
        #PAN CONTROL
            if self.pan_joint:
                pan_drive=UsdPhysics.DriveAPI(self.pan_joint,"angular")
                pan_drive.GetTargetPositionAttr().Set(self.current_pan_angle)


        #-----------------------------------CAR 2---------------------------------------------------------------------------------
        if self.articulation_root_api2:
            ang_vel=0
            ang_vel=(self.ang_vel*360)/(2*math.pi) #Convertion factor between angular velocity from real car and angular velocity from usd stage units
            #lin_vel=self.ang_vel*0.3
            #print(f"Received angular velocity: {self.ang_vel} rad/s")
            #print(f"Linear velocity: {lin_vel} m/s ")
            #print(f"Angular velocity: {ang_vel} degrees/s")                                   
        #BACK TIRES CONTROL            
            if self.lbt_joint2:
                lbt_drive2=UsdPhysics.DriveAPI(self.lbt_joint2,"angular")
                lbt_drive2.GetTargetVelocityAttr().Set(10*ang_vel)
                #lbt_drive.GetDampingAttr().Set(100000)
            if self.rbt_joint2:
                rbt_drive2=UsdPhysics.DriveAPI(self.rbt_joint2,"angular")
                rbt_drive2.GetTargetVelocityAttr().Set(10*ang_vel)
                #rbt_drive.GetDampingAttr().Set(100000)
            
        #FRONT TIRES CONTROL
            if self.lft_joint2:
                #print("found lft joint")
                lft_drive2=UsdPhysics.DriveAPI(self.lft_joint2,"angular")
                #print("found lft drive")
                lft_drive2.GetTargetPositionAttr().Set(self.current_steering_angle)
                lft_drive2.GetDampingAttr().Set(1000)
                lft_drive2.GetStiffnessAttr().Set(1000000)
            if self.rft_joint2:
                rft_drive2=UsdPhysics.DriveAPI(self.rft_joint2,"angular")
                rft_drive2.GetTargetPositionAttr().Set(self.current_steering_angle)
                rft_drive2.GetDampingAttr().Set(1000)
                rft_drive2.GetStiffnessAttr().Set(1000000)
        #TILT CONTROL
            if self.tilt_joint2:
                tilt_drive2=UsdPhysics.DriveAPI(self.tilt_joint2,"angular")
                tilt_drive2.GetTargetPositionAttr().Set(-self.current_tilt_angle)
        #PAN CONTROL
            if self.pan_joint2:
                pan_drive2=UsdPhysics.DriveAPI(self.pan_joint2,"angular")
                pan_drive2.GetTargetPositionAttr().Set(self.current_pan_angle)



    def send_steering_command(self, angle):
         #Convert angle to string if necessary or directly use
        self.mqtt_client.publish(self.steering_angle_topic, str(angle))

    def send_speed_command(self, speed):
         #Convert speed to string if necessary or directly use
        self.mqtt_client.publish(self.back_speed_topic, str(speed))


    def find_prims(self):
        stage=self._usd_context.get_stage()
        #-----------------------------------CAR 1---------------------------------------------------------------------------------
        self.lbt_joint_path="/chassis_isaac_2_5_blender/left_back_tire_link/extender_lbt_rev"
        self.rbt_joint_path="/chassis_isaac_2_5_blender/right_back_tire_link/extender_rbt_rev"
        #self.lbt_joint_path="/chassis_isaac_2_5_blender/left_back_tire_link/lbt_rev"
        #self.rbt_joint_path="/chassis_isaac_2_5_blender/right_back_tire_link/rbt_rev"
        self.lft_revj_path="/chassis_isaac_2_5_blender/left_ackerman_xform/joint_leftt_steering"
        self.rft_revj_path="/chassis_isaac_2_5_blender/right_ackerman_xform/joint_right_steering"
        self.tilt_revj_path="/chassis_isaac_2_5_blender/tilt_link/rev_tilt"
        self.pan_revj_path="/chassis_isaac_2_5_blender/pan_link/rev_pan"

        # Define articulation root
        self.root_prim=stage.GetPrimAtPath("/chassis_isaac_2_5_blender/main_body_frame")
        
        #self.root_prim = stage.GetPrimAtPath("/chassis_isaac_2_5_blender/main_body_frame/frame")
        if self.root_prim:
            self.root_xf=UsdGeom.Xformable(self.root_prim)
            self.articulation_root_api=UsdPhysics.ArticulationRootAPI.Apply(self.root_prim)
            print("Found root prim 1 ")

            if self.articulation_root_api:
                self.root_rigid=UsdPhysics.RigidBodyAPI.Apply(self.root_prim)
                print("Rigid body 1 got")

                self.lbt_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.lbt_joint_path)
                self.rbt_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.rbt_joint_path)
                self.lft_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.lft_revj_path)
                self.rft_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.rft_revj_path)
                self.tilt_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.tilt_revj_path)
                self.pan_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.pan_revj_path)
                #self.rft_trans_rev_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.rft_translation_rev_path)
                #self.lft_trans_rev_joint=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.lft_translation_rev_path)
        else:
            print("not articulation root 1 found")

        #-----------------------------------CAR 2---------------------------------------------------------------------------------
        self.lbt_joint_path2="/chassis_isaac_2_5_blender_01/left_back_tire_link/extender_lbt_rev"
        self.rbt_joint_path2="/chassis_isaac_2_5_blender_01/right_back_tire_link/extender_rbt_rev"
        self.lft_revj_path2="/chassis_isaac_2_5_blender_01/left_ackerman_xform/joint_leftt_steering"
        self.rft_revj_path2="/chassis_isaac_2_5_blender_01/right_ackerman_xform/joint_right_steering"
        self.tilt_revj_path2="/chassis_isaac_2_5_blender_01/tilt_link/rev_tilt"
        self.pan_revj_path2="/chassis_isaac_2_5_blender_01/pan_link/rev_pan"

        # Define articulation root
        self.root_prim2=stage.GetPrimAtPath("/chassis_isaac_2_5_blender_01/main_body_frame")
        
        if self.root_prim2:
            self.root_xf2=UsdGeom.Xformable(self.root_prim2)
            self.articulation_root_api2=UsdPhysics.ArticulationRootAPI.Apply(self.root_prim2)
            print("Found root prim 1 ")

            if self.articulation_root_api2:
                self.root_rigid2=UsdPhysics.RigidBodyAPI.Apply(self.root_prim2)
                print("Rigid body 2 got")

                self.lbt_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.lbt_joint_path2)
                self.rbt_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.rbt_joint_path2)
                self.lft_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.lft_revj_path2)
                self.rft_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.rft_revj_path2)
                self.tilt_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.tilt_revj_path2)
                self.pan_joint2=UsdPhysics.RevoluteJoint.Get(self._usd_context.get_stage(),self.pan_revj_path2)
                
        else:
            print("not articulation root 2 found")



    def connect_to_mqtt(self):
        def on_connect(client, userdata, flags, rc):
            print("Connected to MQTT broker with result code " + str(rc))
            self.mqtt_client.subscribe(self.sensor_data_topic)
            self.mqtt_client.subscribe(self.gray_scale_data_topic)
            self.mqtt_client.subscribe(self.current_steering_angle_topic)
            self.mqtt_client.subscribe(self.current_speed_topic)
            self.mqtt_client.subscribe(self.current_pan_angle_topic)
            self.mqtt_client.subscribe(self.current_tilt_angle_topic)

        def on_message(client, userdata,msg):
            if msg.topic == self.sensor_data_topic:
                self.sensor_data_value = str(msg.payload.decode())
                #print("Received message on topic:  " + msg.topic +"  Distance is:  " + self.sensor_data_value)
                
            elif msg.topic==self.gray_scale_data_topic:
                try:
                    self.gray_scale_data = [int(x) for x in msg.payload.decode().split(",")]
                    #print("gm list ", self.gray_scale_data)
                    self.grayscale_str = ", ".join(map(str, self.gray_scale_data))  
                    #print(f"Received Grayscale Data: {self.grayscale_str}") 
                except ValueError:
                    print("Invalid grayscale data received.")
            elif msg.topic==self.current_steering_angle_topic:
                try:
                    angle=float(msg.payload.decode())
                    print(f"Received current angle for usd: {angle}") 
                    self.current_steering_angle=angle
                except ValueError:
                    print("Invalid steering angle")
            elif msg.topic==self.current_speed_topic:
                try:
                    vel=float(msg.payload.decode())
                    print(f"Received current speed for usd: {vel} rad/s")
                    if vel!=0:
                        self.ang_vel=vel
                    else:
                        self.ang_vel=0
                except ValueError:
                    print("Invalid speed ")
            elif msg.topic==self.current_pan_angle_topic:
                try:
                    pan_ang=float(msg.payload.decode())
                    print(f"Received pan angle for usd: {pan_ang}") 
                    self.current_pan_angle=pan_ang
                except ValueError:
                    print("Invalid pan angle")
            elif msg.topic==self.current_tilt_angle_topic:
                try:
                    tilt_ang=float(msg.payload.decode())
                    print(f"Received tilt angle for usd: {tilt_ang}") 
                    self.current_tilt_angle=tilt_ang
                except ValueError:
                    print("Invalid tilt angle")
            
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_message = on_message

        try:
            self.mqtt_client.connect(self.broker_address, self.broker_port)
            threading.Thread(target=self.mqtt_client.loop_forever).start() 
        except Exception as e:
            print(f"Error connecting to MQTT: {e}")


    def on_shutdown(self):
        print("[camilo.picar] Camilo Picar shutdown")
        self.mqtt_client.disconnect()
        self._on_app_update_sub=None

        # Stop the car
        self.send_speed_command(0)
        self.send_steering_command(0)
        self.send_tilt_command(0)
        self.send_pan_command(0)
        
        # Unsubscribe from keyboard events
        if self.keyboard_sub_id:
            self.input_interface.unsubscribe_to_keyboard_events(self.keyboard_sub_id)
        
        # Disconnect from MQTT
        if self.mqtt_client.is_connected():
            self.mqtt_client.publish(self.back_speed_topic, '0')  # Stop the physical car
            self.mqtt_client.disconnect()


        # Cleanup subscriptions
        self._app_update_sub = None
        self._sub_stage_event = None
        print("Shutdown complete.")
