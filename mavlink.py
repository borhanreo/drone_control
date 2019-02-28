import threading
import time
import sys
import math
import datetime
import cv2
import numpy as np
import os
from AIFACE import face_data_append
from droneapi.lib import Location
from dronekit import connect, VehicleMode, LocationGlobalRelative, Battery, GPSInfo, Command
from pymavlink import mavutil
import sched
#from AIML.autonomous import Autonomous
from AWS.face import Face
from picture.picture import Picture
from Databases import DbInitilize, DbConstant
from Constant import Server,CommandConstant,User,ProjectConstant,AiFaceConstant
from Mission import DroneMission
from Utility import ProjectCommand,SdCard,JsonBuilder
from GoogleLatLng import LatLngCalculation
from flask import Flask, render_template
#from RC import RcController

from flask_socketio import SocketIO
app = Flask(__name__)
app.config['SECRET_KEY'] = Server.ServerClass.get_server_secret_key()
socketio = SocketIO(app)
from socketIO_client import SocketIO as client_socketio, BaseNamespace
import json
from Channels import DroneChannels
import serial
from DateTime import ProjectDateTime
from IndoorFly import IndoorFly,DroneSpeed
from picamera.array import PiRGBArray
from picamera import PiCamera
#ser =serial.Serial('/dev/ttyUSB0', 9600)
from AICAPTURE.mod_capture import ModCapture
#from AUTONOMOUS.autonomous import Autonomous
#################################################### START FACE AI

# def run_face_api():
#     python_path = os.getcwd() + "/AIFACE/pi_face_recognition.py"
#     cascade_path = os.getcwd() + "/AIFACE/haarcascade_frontalface_default.xml"
#     encoding_path = os.getcwd() + "/AIFACE/encodings.pickle"
#     os.system("python "+python_path+" --cascade "+cascade_path+" --encodings "+encoding_path)
#################################################### END FACE AI
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 16
GPIO_ECHO = 20
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
obstacle_enable = False

throttle_value = 307
roll_value = 420
pitch_value =420
yaw_value = 420
sensor_min_reading=15
sensor_max_reading=60

channel_input_min_value= 307
channel_input_max_value= 532
channel_output_min_value= 1089
channel_output_max_value= 1888

def distance():
  GPIO.output(GPIO_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, False)
  StartTime = time.time()
  StopTime = time.time()
  while GPIO.input(GPIO_ECHO) == 0:
    StartTime = time.time()
  while GPIO.input(GPIO_ECHO) == 1:
    StopTime = time.time()
  TimeElapsed = StopTime - StartTime
  distance = (TimeElapsed * 34300) / 2
  return distance
## LASH CHANGE
vehicle = connect('/dev/ttyACM0', wait_ready=False)
#ultrasonicsensor =  serial.Serial('/dev/ttyUSB0',9600, timeout = 5)
ultrasonicsensor = None
battery_voltage = 0.00
battery_current = 0.00
battery_level = 0
gps_fix = 0
gps_num_sat = 0
gps_lat = 0.00
gps_lon = 0.00
gps_alt = 0
mode =""
#### AI Class Start
#autonomous = Autonomous()
picture = Picture()
capture = ModCapture()
face = Face()
my_client_send=None
#autonomous = Autonomous(vehicle)
sonar_alt=-5
#### AI Class End
# scheduler_array= DbInitilize.DbInitializeClass.fetch_all_row()
#scheduler_array= None
# print scheduler_array
#print len(scheduler_array)
is_ultrasonic_enable= False
def init_sender_socket():
    global my_client_send
    my_client_send = client_socketio(ProjectConstant.ProjectConstantClass.get_host(),ProjectConstant.ProjectConstantClass.get_port())
def send_message(send_msg):
    my_client_send.emit('chat message', send_msg)

#BATTERY INFO
def receive_battery_data():
    # Demonstrate getting callback on any attribute change
    def wildcard_callback(self, attr_name, value):
        #print(" CALLBACK_BATTERY: (%s): %s" % (attr_name, value))
        send_message(JsonBuilder.JsonBuilderClass.get_battery_information(value.voltage,value.current,value.level))
    #print("\nAdd attribute callback detecting ANY attribute change L")
    vehicle.add_attribute_listener('battery', wildcard_callback)
previous_milli = 0

#LOCATION INFO
def receive_location_data():
    # Demonstrate getting callback on any attribute change
    def receive_location_callback(self, attr_name, value):
        #print(" CALLBACK_LOCATION: (%s): %s" % (attr_name, value))
        # ALTITUDE GET
        send_message(JsonBuilder.JsonBuilderClass.get_location_information(value.lat,value.lon,value.alt))
        #if value.alt >= 1:
            #vehicle.mode = VehicleMode("ALT_HOLD")
            #RcController.RcControllerClass.send_rc_command_for_pin(2,420)
    vehicle.add_attribute_listener('location.global_relative_frame', receive_location_callback)
def range_finder():
    #@vehicle.on_attribute('rangefinder')
    def rangefinder_callback(self, attr_name,value):
        #print " Rangefinder (metres): %s" % attr_name
        #distance=2.81999993324, voltage=1.38400006294
        #pass
        my_location_alt = vehicle.location.global_relative_frame
        #my_location_alt.alt = value.distance
        print ("Rangefinder ", value.distance, value.voltage)
        global sonar_alt
        sonar_alt = value.distance

    vehicle.add_attribute_listener('rangefinder', rangefinder_callback)
#GPS INFO
def receive_gps_info():
    # Demonstrate getting callback on any attribute change
    def receive_gps_info_callback(self, attr_name, value):
        #print(" CALLBACK_GPS_INFO: (%s): %s" % (attr_name, value))
        send_message(JsonBuilder.JsonBuilderClass.get_gps_inf_information(value.fix_type,value.satellites_visible))
    vehicle.add_attribute_listener('gps_0', receive_gps_info_callback)
#MODE CHANGE
def receive_mode_data():
    @vehicle.on_attribute('mode')
    def receive_mode_data_callback(self, attr_name, value):
        #print(" CALLBACK: Mode changed to (%s), (%s)", value, attr_name)
        send_message(JsonBuilder.JsonBuilderClass.get_mode_information(str(vehicle.mode.name)))
    vehicle.add_attribute_listener('mode', receive_mode_data_callback)

def attitude_data_change():
    def attitude_data_change_callback(self, attr_name, value):
        #pass
        #print ("I am here ",math.degrees(value.pitch), math.degrees(value.roll), math.degrees(value.yaw),value.pitch, value.roll, value.yaw)
        #print ("I am here ",value.pitch, value.roll, value.yaw)
        airspeed = str(DroneSpeed.DroneSpeedClass.get_vehicle_air_speed(vehicle))
        groundspeed = str(DroneSpeed.DroneSpeedClass.get_vehicle_ground_speed(vehicle))
        #stable_position_new(math.degrees(value.pitch),math.degrees(value.roll),math.degrees(value.yaw))
        send_message(JsonBuilder.JsonBuilderClass.get_compass_value(math.degrees(value.pitch),math.degrees(value.roll),math.degrees(value.yaw),airspeed,groundspeed))

    vehicle.add_attribute_listener('attitude', attitude_data_change_callback)
def stable_position_new(p,r,y):
    pitch_per_degree = 2
    pitch_mid_value = 409
    roll_mid_value = 434
    if(p<-0.05):
        value_rc_pitch_backward = int(p *(-1)* pitch_per_degree)
        rc_value_backward = pitch_mid_value + value_rc_pitch_backward
        vehicle.channels.overrides['1'] = remap(rc_value_backward, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("Going forward need to backward",rc_value_backward)
    elif (p > 0.05):
        value_rc_pitch_forward = int(p * pitch_per_degree)
        c_value_forward = pitch_mid_value - value_rc_pitch_forward
        vehicle.channels.overrides['1'] = remap(c_value_forward, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("Going backward need to forward", c_value_forward)
    else:
        vehicle.channels.overrides['1'] = remap(pitch_mid_value, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
    if (r < -0.05):
        value_rc_roll_left = int(r * (-1) * pitch_per_degree)
        rc_value_roll_left = roll_mid_value - value_rc_roll_left
        vehicle.channels.overrides['2'] = remap(rc_value_roll_left, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("Going left need to right", rc_value_roll_left)
    elif (r>0.05):
        value_rc_roll_right = int(r * pitch_per_degree)
        rc_value_roll_right = roll_mid_value + value_rc_roll_right
        vehicle.channels.overrides['2'] = remap(rc_value_roll_right, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("Going right need to left", rc_value_roll_right)
    else:
        vehicle.channels.overrides['2'] = remap(roll_mid_value, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)

def stable_position(p,r,y):
    # pitch_per_degree = 4.822
    # pitch_mid_value = 1481

    pitch_per_degree = 2
    pitch_mid_value = 420
    if(p<0):
        value_rc_pitch_backward = int(p *-1* pitch_per_degree)
        rc_value_backward = pitch_mid_value - value_rc_pitch_backward
        #RcController.RcControllerClass.send_rc_command_for_pin(0, value_rc_pitch_backward)
        vehicle.channels.overrides['1'] = remap(rc_value_backward, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("FORCE TO FORWARD FOR  BACKWARD", value_rc_pitch_backward, rc_value_backward)
    elif (p>0):
        value_rc_pitch_forward = int(p * pitch_per_degree)
        rc_value_forward = pitch_mid_value + value_rc_pitch_forward
        #RcController.RcControllerClass.send_rc_command_for_pin(0, rc_value_forward)
        vehicle.channels.overrides['1'] = remap(rc_value_forward, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("FORCE TO BACKWARD FOR FORWARD ", value_rc_pitch_forward, rc_value_forward)
        # pass
    else:
        pass
    # roll_per_degree = 4.822
    # roll_mid_value = 1494
    roll_per_degree = 2
    roll_mid_value = 420
    if (r < 0):
        value_rc_roll_left = int(r * (-1) * roll_per_degree)
        rc_value_left =  roll_mid_value + value_rc_roll_left
        #RcController.RcControllerClass.send_rc_command_for_pin(1,rc_value_left)
        vehicle.channels.overrides['2'] = remap(rc_value_left, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("FORCE TO LEFT FOR RIGHT ",value_rc_roll_left,rc_value_left)
        ## force to L
    elif (r > 0):
        value_rc_roll_right = int(r * roll_per_degree)
        rc_value_right = roll_mid_value - value_rc_roll_right
        #RcController.RcControllerClass.send_rc_command_for_pin(1,rc_value_right)
        vehicle.channels.overrides['2'] = remap(rc_value_right, channel_input_min_value, channel_input_max_value,
                                                channel_output_min_value, channel_output_max_value)
        print("FORCE TO RIGHT FOR  LEFT",value_rc_roll_right, rc_value_right)
        #pass
    else:
        pass

    if (y < 0):
        pass
    elif (y > 0):
        pass
    else:
        pass

def velocity_data_change():
    def velocity_callback(self, attr_name, value):
        pass
        #print "i am here (%s)", str(value)
    vehicle.add_attribute_listener('velocity', velocity_callback)
def receive_all_data():
    def wildcard_callback(self, attr_name, value):
        #print(" All data CALLBACK_ALL: (%s): %s" % (attr_name, value))
        pass
    vehicle.add_attribute_listener('*', wildcard_callback)
def all_value():
    @vehicle.on_message('*')
    def listener(self, name, message):
        print("Al Message ", message)
# def any_parameter_callback(self, attr_name, value):
#     print " ANY PARAMETER CALLBACK: %s changed to: %s" % (attr_name, value)
# vehicle.parameters.add_attribute_listener('*', any_parameter_callback)

def all_information():
    previous_milli = 0
    while True:
        current_milli = time.mktime(datetime.datetime.now().timetuple()) * 1000
        if current_milli-previous_milli>=1000:
            print (" every 1 second")
            send_message(JsonBuilder.JsonBuilderClass.get_all_information(battery_voltage,battery_current,battery_level,gps_fix,gps_num_sat,gps_lat,gps_lon,gps_alt))
            previous_milli = current_milli

def init_socket():
    my_client = client_socketio(Server.ServerClass.get_server_ip(), Server.ServerClass.get_server_port())
    @socketio.on('chat message')
    def handle_message(message):
        print('received message init: ' + message)

    def socket_receiver(data):
        global channel_param
        channel_param = 1491
        try:
            #print (data)
            #vehicle = mpstate.get_vehicles()[0]
            obj_data = json.loads(data.replace('\r\n', '\\r\\n'),strict=False)
            #obj_data = json.loads(data,strict=False)
            #obj_data = json.loads(data)
            sender_user = obj_data['u']
            if(sender_user==User.UserClass.ground_user()):
                val = obj_data[CommandConstant.action]
                if (val == CommandConstant.CommandConstantClass.is_equal_wp()):
                    #DbInitilize.DbInitializeClass.dbInit()
                    wp_data = obj_data[str(CommandConstant.data)]
                    rout_name = obj_data['name']

                    #rout_name = "abc"
                    SdCard.SdCardClass.file_write(wp_data)
                    time.sleep(1)

                    if(DroneMission.DroneMissionClass.upload_mission(CommandConstant.CommandConstantClass.get_wp_file_name(),vehicle)):
                        DbInitilize.DbInitializeClass.update_wp_status_true()
                        DbInitilize.DbInitializeClass.tbldbWaypointInsert(rout_name,wp_data)
                        send_message(JsonBuilder.JsonBuilderClass.get_waypoint_received_response())
                elif (val == CommandConstant.CommandConstantClass.is_equal_start_drone()): ## Start
                    init_aircraft()
                    send_message(JsonBuilder.JsonBuilderClass.get_start_information(vehicle))
                elif (val == CommandConstant.CommandConstantClass.is_equal_waypoint_delete()): ## Delete all waypoint
                    DbInitilize.DbInitializeClass.delete_all_wp()
                    DbInitilize.DbInitializeClass.delete_all_schedule()
                elif (val == CommandConstant.CommandConstantClass.is_equal_arm()):
                    vehicle.armed=True
                    send_message(JsonBuilder.JsonBuilderClass.get_is_arm())
                elif (val == CommandConstant.CommandConstantClass.is_equal_disarm()):
                    vehicle.armed = False
                elif (val == CommandConstant.CommandConstantClass.is_equal_mode()):
                    mode_str = obj_data[CommandConstant.data]

                    vehicle.mode = VehicleMode(mode_str)
                elif (val == CommandConstant.CommandConstantClass.is_equal_takeoff()):
                    aTargetAltitude = obj_data[CommandConstant.data]
                    send_message(JsonBuilder.JsonBuilderClass.get_is_takeoff())
                    drone_goto(1.00,3)
                elif (val == CommandConstant.CommandConstantClass.is_equal_takeoff_land()):
                    # aTargetAltitude = obj_data[CommandConstant.data]
                    # print ("takeoff land")
                    # DbInitilize.DbInitializeClass.get_data_wp_status()
                    # if(DbInitilize.DbInitializeClass.get_data_wp_status()==0):
                    #     send_message(JsonBuilder.JsonBuilderClass.get_is_no_waypoint())
                    # elif(DbInitilize.DbInitializeClass.get_data_wp_status()==1):
                    #     DbInitilize.DbInitializeClass.update_wp_status_false()
                    #     send_message(JsonBuilder.JsonBuilderClass.get_is_takeoff())
                    #
                    #autonomous.arm_and_takeoff_nogps(1)
                    pass
                elif (val == CommandConstant.CommandConstantClass.is_equal_rc_03()):
                    rc_value = int(obj_data[CommandConstant.data])
                    vehicle.channels.overrides['3'] = rc_value
                elif (val == CommandConstant.CommandConstantClass.is_equal_battery_info()):
                    try:
                        voltage=vehicle.battery.voltage
                        current=vehicle.battery.current
                        level=vehicle.battery.level
                    except:
                        print('An error occurred.')
                        voltage = 0.00
                        current = 0.00
                        level = 0
                    send_message(JsonBuilder.JsonBuilderClass.get_battery_information(voltage,current,level))
                elif (val == CommandConstant.CommandConstantClass.is_equal_reboot()):
                    vehicle.reboot()
                    time.sleep(1)
                elif (val == CommandConstant.CommandConstantClass.is_equal_store_param()):
                    #print "\nPrint all parameters (iterate `vehicle.parameters`):"
                    append_string = ""
                    for key, value in vehicle.parameters.iteritems():
                        #print " Key:%s Value:%s" % (key, value)
                        append_string = append_string + str(key)+":"+str(value)+"\n"
                    SdCard.SdCardClass.param_write(append_string)
                    print (append_string)
                elif (val == CommandConstant.CommandConstantClass.is_equal_home_location()):
                    #home_lat,home_lng = LatLngCalculation.LatLngCalculationClass.get_home_location(vehicle)
                    #print " ",home_lat," ",home_lng
                    init_aircraft()
                elif (val == CommandConstant.CommandConstantClass.is_equal_read_channels()):
                    DroneChannels.DroneChannelsClass.read_channels(vehicle)
                elif (val == CommandConstant.CommandConstantClass.is_equal_manual_fly()):
                    rc_0 = int(obj_data['channel_0'])
                    rc_1 = int(obj_data['channel_1'])
                    rc_2 = int(obj_data['channel_2'])
                    rc_3 = int(obj_data['channel_3'])
                    rc_4 = int(obj_data['channel_4'])
                    rc_5 = int(obj_data['channel_5'])
                    rc_6 = int(obj_data['channel_6'])
                    rc_7 = int(obj_data['channel_7'])
                    global throttle_value
                    throttle_value = rc_2
                    global roll_value
                    roll_value = rc_0
                    global pitch_value
                    pitch_value = rc_1
                    global yaw_value
                    yaw_value = rc_3
                    # RC
                    # RcController.RcControllerClass.send_rc_command(rc_0,rc_1,rc_2,rc_3,rc_4,rc_5,rc_6,rc_7)
                elif (val == CommandConstant.CommandConstantClass.is_equal_send_waypoint_id_name()):
                    print (DbInitilize.DbInitializeClass.get_waypoint_id_name_json_array())
                    #print DbInitilize.DbInitializeClass.select_all_tbldbSchedule()
                    #DbInitilize.DbInitializeClass.select_all_wp()
                    send_message(JsonBuilder.JsonBuilderClass.get_waypoint_id_name_array_to_ground(DbInitilize.DbInitializeClass.get_waypoint_id_name_json_array()))
                elif (val == CommandConstant.CommandConstantClass.is_equal_schedule_create()):


                    print (obj_data['s_id'], obj_data['drone_power_on'], obj_data['flight_start'])
                    #toDayDate = datetime.datetime(2012, 2, 2, 0, 0)

                    # toDayDate = datetime.datetime(int(obj_data['year']), int(obj_data['month']), int(obj_data['day']), int(obj_data['hour']),
                    #                               int(obj_data['minutes']), int(obj_data['second']))
                    # tstamp = time.mktime(toDayDate.timetuple())
                    # print "i am here",tstamp, tstamp+21600
                    # drone_fly = long(tstamp+21600)
                    # drone_poweron = long(tstamp+21300)

                    DbInitilize.DbInitializeClass.tbldbScheduleInsert(int(obj_data['s_id']),int(obj_data['drone_power_on']),int(obj_data['flight_start']))
                    #DbInitilize.DbInitializeClass.tbldbScheduleInsert(int(obj_data['s_id']),drone_poweron,drone_fly)
                    send_message(JsonBuilder.JsonBuilderClass.receive_schedule_response())
                    print (DbInitilize.DbInitializeClass.get_schedule_id_name_json_array())
                    print (DbInitilize.DbInitializeClass.get_schedule_counter())
                    print (" ")
                    serialcmd = str("A,1:B," + str(obj_data['drone_power_on'])+":C,"+str(DbInitilize.DbInitializeClass.get_schedule_counter())+"\n")
                    print (serialcmd)
                    # RTC hide
                    # ser.write(serialcmd)
                elif (val == CommandConstant.CommandConstantClass.is_equal_schedule_show_data()):
                    print (DbInitilize.DbInitializeClass.get_schedule_id_name_json_array())
                    send_message(JsonBuilder.JsonBuilderClass.get_waypoint_time_schedulle(DbInitilize.DbInitializeClass.get_schedule_id_name_json_array()))
                    print (JsonBuilder.JsonBuilderClass.get_waypoint_time_schedulle(DbInitilize.DbInitializeClass.get_schedule_id_name_json_array()))
                elif (val == CommandConstant.CommandConstantClass.is_equal_set_date_time()):
                    year = int(obj_data['year'])
                    month = int(obj_data['month'])
                    day = int(obj_data['day'])
                    hour = int(obj_data['hour'])
                    minutes = int(obj_data['minutes'])
                    second = int(obj_data['second'])

                    time_tuple = (year,  # Year
                                  month,  # Month
                                  day,  # Day
                                  hour,  # Hour
                                  minutes,  # Minute
                                  second,  # Second
                                  0,)  # Millisecond
                    ProjectDateTime.ProjectDateTimeClass._raspberry_pi_set_time(time_tuple)
                    print ("update raspberry pi")
                    print (year,month,day,hour,minutes,second)
                    try:
                        update_arduino = str(
                            "A,1:Z,1:Y," + str(obj_data['year']) + ":M," + str(obj_data['month']) + ":D," + str(obj_data[
                                'day']) + ":H," + str(obj_data['hour']) + ":I," + str(obj_data['minutes']) + ":S," + str(obj_data[
                                'second'])+"\n")
                        print (update_arduino)
                        # RTC hide
                        # ser.write(update_arduino)
                        print ("update arduino ")
                    except Exception as error :
                        pass
                elif (val == CommandConstant.CommandConstantClass.is_equal_indoor_fly()):
                    print ("indoor fly")
                    #IndoorFly.IndoorFlyClass.set_vehicle(vehicle)
                    time.sleep(3)
                    #IndoorFly.IndoorFlyClass.fly_indoor_target_alt(10)
                    #IndoorFly.IndoorFlyClass.set_a()
                    # RcController.RcControllerClass.fly_ai(vehicle, 420, 420, 307, 420, 307, 307, 307, 307)
                elif (val == CommandConstant.CommandConstantClass.is_equal_schedule_all()):
                    send_message(JsonBuilder.JsonBuilderClass.get_all_schedule_response(
                        DbInitilize.DbInitializeClass.fetch_all_row_schedule()))
                    print (JsonBuilder.JsonBuilderClass.get_all_schedule_response(
                        DbInitilize.DbInitializeClass.fetch_all_row_schedule()))

                elif (val == CommandConstant.CommandConstantClass.is_equal_delete_wp_by_id()):
                    id = str(obj_data['id'])
                    DbInitilize.DbInitializeClass.delete_wp_by_id(id)
                    send_message(JsonBuilder.JsonBuilderClass.get_is_wp_delete())
                elif (val == CommandConstant.CommandConstantClass.is_equal_delete_schedule_by_id()):
                    id = str(obj_data['id'])
                    DbInitilize.DbInitializeClass.delete_schedule_by_id(id)
                    send_message(JsonBuilder.JsonBuilderClass.get_is_schedule_delete())
                elif (val == CommandConstant.CommandConstantClass.is_equal_test_takoff()):
                    drone_actions()
                elif (val == CommandConstant.CommandConstantClass.is_equal_obs_start()):
                    #autonomous.start()
                    send_message(JsonBuilder.JsonBuilderClass.get_is_res_obs_start())
                elif (val == CommandConstant.CommandConstantClass.is_equal_obs_stop()):
                    #autonomous.stop()
                    send_message(JsonBuilder.JsonBuilderClass.get_is_res_obs_stop())
                elif (val == CommandConstant.CommandConstantClass.is_equal_face_capture()):
                    dynamic_pic_match()
                elif(val == CommandConstant.CommandConstantClass.is_object_detection_start()):
                    send_message(JsonBuilder.JsonBuilderClass.get_object_detection_response_on())
                elif(val == CommandConstant.CommandConstantClass.is_face_detect()):
                    picture.start()
                elif (val == CommandConstant.CommandConstantClass.is_obstacle_enable()):
                    throttle_value=throttle_value+1
                    print(throttle_value,roll_value,pitch_value,yaw_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(2,throttle_value)

                    # for x in range(1500,1800):
                    #     print ("throttle value",x)
                    #     vehicle.channels.overrides = {'3': x}
                    #     time.sleep(0.1)

                    # global obstacle_enable
                    # obstacle_enable=True
                elif (val == CommandConstant.CommandConstantClass.is_obstacle_enable()):
                    throttle_value=throttle_value+1
                    print(throttle_value,roll_value,pitch_value,yaw_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(2,throttle_value)
                elif (val == CommandConstant.CommandConstantClass.is_obstacle_disable()):
                    throttle_value = throttle_value -1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(2, throttle_value)
                    # channel_param = channel_param -2
                    # print (channel_param)
                    # vehicle.channels.overrides = {'3': channel_param}
                    obstacle_enable=False
                elif (val == CommandConstant.CommandConstantClass.is_roll_inc()):
                    roll_value=roll_value+1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    # RcController.RcControllerClass.send_rc_command_for_pin(0, roll_value)
                elif (val == CommandConstant.CommandConstantClass.is_roll_dec()):
                    roll_value=roll_value-1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    # RcController.RcControllerClass.send_rc_command_for_pin(0, roll_value)
                elif (val == CommandConstant.CommandConstantClass.is_pitch_inc()):
                    pitch_value=pitch_value+1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    # RcController.RcControllerClass.send_rc_command_for_pin(1, pitch_value)
                elif (val == CommandConstant.CommandConstantClass.is_pitch_dec()):
                    pitch_value=pitch_value-1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    # RcController.RcControllerClass.send_rc_command_for_pin(1, pitch_value)

                elif (val == CommandConstant.CommandConstantClass.is_yaw_inc()):
                    yaw_value=yaw_value+1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    # RcController.RcControllerClass.send_rc_command_for_pin(3, yaw_value)
                elif (val == CommandConstant.CommandConstantClass.is_yaw_dec()):
                    yaw_value=yaw_value-1
                    print(throttle_value, roll_value, pitch_value, yaw_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(3, yaw_value)


                elif (val == CommandConstant.CommandConstantClass.is_pitch_single()):
                    rc_0 = int(obj_data['channel_0'])
                    vehicle.channels.overrides['1'] = remap(rc_0, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(0, rc_0)
                elif (val == CommandConstant.CommandConstantClass.is_roll_single()):
                    rc_1 = int(obj_data['channel_1'])
                    vehicle.channels.overrides['2'] = remap(rc_1, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(1, rc_1)
                elif (val == CommandConstant.CommandConstantClass.is_throttle_single()):
                    rc_2 = int(obj_data['channel_2'])
                    #print("MAP ", rc_2, remap(rc_2, 307, 532, 1089, 1888))
                    vehicle.channels.overrides['3'] = remap(rc_2, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(2, rc_2)
                elif (val == CommandConstant.CommandConstantClass.is_yaw_single()):
                    rc_3 = int(obj_data['channel_3'])
                    vehicle.channels.overrides['4'] = remap(rc_3, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                    #RcController.RcControllerClass.send_rc_command_for_pin(3, rc_3)
                elif (val == CommandConstant.CommandConstantClass.is_channel5_single()):
                    rc_4 = int(obj_data['channel_4'])
                    vehicle.channels.overrides['5'] = remap(rc_4, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                elif (val == CommandConstant.CommandConstantClass.is_channel6_single()):
                    rc_5 = int(obj_data['channel_5'])
                    vehicle.channels.overrides['6'] = remap(rc_5, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                elif (val == CommandConstant.CommandConstantClass.is_channel7_single()):
                    rc_6 = int(obj_data['channel_6'])
                    vehicle.channels.overrides['7'] = remap(rc_6, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                elif (val == CommandConstant.CommandConstantClass.is_channel8_single()):
                    rc_7 = int(obj_data['channel_7'])
                    vehicle.channels.overrides['8'] = remap(rc_7, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
                elif (val == CommandConstant.CommandConstantClass.is_ultrasonic_enable_disable1()):
                    global is_ultrasonic_enable
                    is_ultrasonic_enable=True
                elif (val == CommandConstant.CommandConstantClass.is_ultrasonic_enable_disable2()):
                    is_ultrasonic_enable=False
                print "Channel values from RC Tx:", vehicle.channels
            elif sender_user==User.UserClass.self_user():
                pass
                #print ("self message")
        except Exception as error:
            print (error)
        #set_command(data)
    my_client.on('chat message', socket_receiver)
    my_client.wait()

def remap(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def run_capture():
    pass
    #capture.run(False)
    # capture.run_usb_cam()

def dynamic_pic_match():
    image_path = os.getcwd() + "/AWS/target-image/cam.jpg"
    os.system("raspistill -w 1408 -h 792 -o "+image_path)

    time.sleep(2)
    #face.face_s3_match(image_path)
    send_message(JsonBuilder.JsonBuilderClass.get_face_captured())
    person = face.face_s3_match_multi_moduler(image_path)
    if len(person)>0:
        if 'John' in person:
            send_message(JsonBuilder.JsonBuilderClass.get_aiface_response_for_john())
        else:
            send_message(JsonBuilder.JsonBuilderClass.get_aiface_response(person))
    else:
        send_message(JsonBuilder.JsonBuilderClass.get_aiface_response_none())


    print (person)
    # label, confidence = face.face_match(image_path)
    # if label==None or confidence==None:
    #     str = "I do not know you "
    #     send_message(JsonBuilder.JsonBuilderClass.get_aiface_response(str))
    # else:
    #     person_name = label.replace(".jpg", "")
    #     send_message(JsonBuilder.JsonBuilderClass.get_aiface_response(person_name))
    #     print person_name, confidence
    #face.detect_label('http://www.alislah.ma/media/k2/items/cache/758aa02672549a35b7d840da4c5b3387_XL.jpg')
def cmddd():
    # add a takeoff command
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Test command ")
    cmds = vehicle.commands
    altitude = 100  # target altitude
    pitch = 45  # take off pitch. Need to check if degrees or radians, and what is a reasonable valued.
    #cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,pitch, 0, 0, 0, 1, 1, altitude)
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.SET_POSITION_TARGET_LOCAL_NED, 0, 0,pitch, 0, 0, 0, 0, 0, altitude)
    cmds.add(cmd)
    vehicle.commands.upload()
def arm_and_takeoff(aTargetAltitude,test_type):
    # Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print(" Waiting for vehicle to initialise...")
    #     time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    if test_type==True:
        vehicle.mode = VehicleMode("AUTO")
    elif test_type==False:
        vehicle.mode = VehicleMode("LAND")
def ai_receiver():
    print ("AI")
    print ("pixhawk")
def init_aircraft():
    # api = local_connect()
    # vehicle = api.get_vehicles()[0]
    print ("Global Location: %s" % vehicle.location.global_frame)
    print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print ("Local Location: %s" % vehicle.location.local_frame)
    print ("Home  Location: %s" % vehicle.home_location)
    print ("Home  Location: %s" % vehicle.home_location)
    print ("Mode %s" % vehicle)
    print ("Battery %s" % vehicle.battery)
    print ("Mode %s" % vehicle.mode)
    print ("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
def run_nano():
    print("Run Arduino")
    i = 0
    while 1:
        #print ser.readline()
        try:
            obj_data = json.loads('')
            # RTC hide
            # obj_data = json.loads(ser.readline())
            action = int(obj_data['action'])
            if(action==1):
                scheduler_array = DbInitilize.DbInitializeClass.fetch_all_row()
                print (i, scheduler_array)
                if int(obj_data['beat']) in scheduler_array:
                    print ("Ok ")
                    sid =  DbInitilize.DbInitializeClass.fetch_sid_row(str(obj_data['beat']))
                    print (DbInitilize.DbInitializeClass.fetch_wp_by_sid(str(sid[0])))
                    wp_auto_load(DbInitilize.DbInitializeClass.fetch_wp_by_sid(str(sid[0])))
                unixtime = float(obj_data['beat'])
                if i == 0:
                    ProjectDateTime.ProjectDateTimeClass.convert_date_time(unixtime)
                i = i + 1
                # if i == 100:
                #     i = 1
                print (str(obj_data['beat']))

            elif (action==2):
                print ("EEPROM Store res")
            elif (action==3):
                print ("Time updated")
        except Exception as error:
            print("Error "+str(error))
def drone_speed():
    previous_milli = 0
    while 1:
        current_milli = time.mktime(datetime.datetime.now().timetuple()) * 1000
        if current_milli - previous_milli >= 1000:
            airspeed = str(DroneSpeed.DroneSpeedClass.get_vehicle_air_speed(vehicle))
            groundspeed = str(DroneSpeed.DroneSpeedClass.get_vehicle_ground_speed(vehicle))
            print ("i am here ",airspeed,groundspeed)
            send_message(JsonBuilder.JsonBuilderClass.get_speed_json(airspeed,groundspeed))
            previous_milli = current_milli
def wp_auto_load(wp_data):
    print (wp_data)
    SdCard.SdCardClass.file_write(wp_data)
    time.sleep(1)
    if (
    DroneMission.DroneMissionClass.upload_mission(CommandConstant.CommandConstantClass.get_wp_file_name(), vehicle)):
        DbInitilize.DbInitializeClass.update_wp_status_true()
        send_message(JsonBuilder.JsonBuilderClass.get_waypoint_received_response())

def takeoff_drone(time_limit=10):
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    # Tune Throttle
    #RcController.RcControllerClass.fly_up(time_limit)

def drone_actions():
    # TakeOff
    takeoff_drone(time_limit=10)

    vehicle.mode = VehicleMode("ALT_HOLD")

    # MoveLeft
    vehicle.gimbal.rotate(0, 0, 45)
    time.sleep(5)

    # MoveRight
    vehicle.gimbal.rotate(0, 0, -45)
    time.sleep(5)

    # LandDown
    # RcController.RcControllerClass.fly_down(time_limit=10)

def land_drone(time_limit=10):
    pass
    # Tune Throttle
    # RcController.RcControllerClass.fly_down(time_limit)
def thread_ultrasonic_object():
    try:
        while True:
            if obstacle_enable == True:
                dist = int(distance())
                if dist >= 15 and dist <= 80:
                    print ("Measured Distance = %.1f cm" % dist)

                else:
                    print("AA",dist)
            time.sleep(0.2)

            # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")

def thread_read_ultrasor():
    while True:
        if is_ultrasonic_enable==True:
            try:
                line1 = ultrasonicsensor.readline()
                line = line1.replace('\r\n', '')
                data = line.split(",")
                print (len(data), data[0], data[1], data[2], data[3])
                left_sensor = float(str(data[0]))
                right_sensor = float(str(data[1]))
                forward_sensor = float(str(data[2]))
                backward_sensor = float(str(data[3]))
                #print (left_sensor, right_sensor, forward_sensor, backward_sensor)
                if (left_sensor > sensor_min_reading and left_sensor <= sensor_max_reading):
                    #force_pin0(0, 390, 0.5)
                    vehicle.channels.overrides['1'] = remap(390, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)
                elif right_sensor >= sensor_min_reading and right_sensor < sensor_max_reading:
                    #force_pin0(0, 458, 0.5)
                    vehicle.channels.overrides['1'] = remap(458, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)
                else:
                    #RcController.RcControllerClass.send_rc_command_for_pin(0, 434)
                    vehicle.channels.overrides['1'] = remap(434, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)

                if (forward_sensor > sensor_min_reading and forward_sensor < sensor_max_reading):
                    vehicle.channels.overrides['2'] = remap(443, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)
                elif (backward_sensor > sensor_min_reading and backward_sensor < sensor_max_reading):
                    vehicle.channels.overrides['2'] = remap(395, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)
                else:
                    vehicle.channels.overrides['2'] = remap(409, channel_input_min_value, channel_input_max_value,
                                                            channel_output_min_value, channel_output_max_value)
                # if (left_sensor > sensor_min_reading and left_sensor <= sensor_max_reading):
                #     force_pin0(0, 400, 0.5)
                #     #print("Going to right")
                # else:
                #     RcController.RcControllerClass.send_rc_command_for_pin(0, 434)
                # if right_sensor >= sensor_min_reading and right_sensor < sensor_max_reading:
                #     force_pin0(0, 468, 0.5)
                #     #print("Going to left")
                # else:
                #     RcController.RcControllerClass.send_rc_command_for_pin(0, 434)
                # if (forward_sensor > sensor_min_reading and forward_sensor < sensor_max_reading):
                #     force_pin1(1, 443, 0.5)
                # else:
                #     RcController.RcControllerClass.send_rc_command_for_pin(1, 409)
                # if (backward_sensor > sensor_min_reading and backward_sensor < sensor_max_reading):
                #     force_pin1(1, 375, 0.5)
                # else:
                #     RcController.RcControllerClass.send_rc_command_for_pin(1, 409)
            except Exception as error:
                print("Json ", error)
        else:
            time.sleep(0.02)
            pass
def run_rc_value():
    while True:
        print ("Channel values from RC Tx:", vehicle.channels)
        time.sleep(0.05)

def run_thread():
    # thread_apm = threading.Thread(target=receive_battery_data)
    # thread_apm.daemon = True
    # thread_apm.start()
    receive_battery_data()
    receive_all_data()
    range_finder()
    # thread_apm_location = threading.Thread(target=receive_location_data)
    # thread_apm_location.daemon = True
    # thread_apm_location.start()
    receive_location_data()

    thread_receiver = threading.Thread(target=init_socket)
    thread_receiver.daemon = True
    thread_receiver.start()

    thread_sender = threading.Thread(target=init_sender_socket)
    thread_sender.daemon = True
    thread_sender.start()



    # thread_mode_data_response = threading.Thread(target=receive_mode_data)
    # thread_mode_data_response.daemon=True
    # thread_mode_data_response.start()
    receive_mode_data()
    # thread_attitude = threading.Thread(target=attitude_data_change)
    # thread_attitude.daemon = True
    # thread_attitude.start()

    attitude_data_change()

    # velocity_attitude = threading.Thread(target=velocity_data_change)
    # velocity_attitude.daemon = True
    # velocity_attitude.start()
    #velocity_data_change()

    # thread_mode_data_response = threading.Thread(target=receive_gps_info)
    # thread_mode_data_response.daemon=True
    # thread_mode_data_response.start()

    receive_gps_info()

    # thread_all_data = threading.Thread(target=receive_all_data)
    # thread_all_data.daemon = True
    # thread_all_data.start()

    # thread_all_data = threading.Thread(target=receive_all_data)
    # thread_all_data.daemon = True
    # thread_all_data.start()

    thread_ai = threading.Thread(target=ai_receiver)
    thread_ai.daemon = True
    thread_ai.start()

    # thread_capture = threading.Thread(target=run_capture)
    # thread_capture.daemon = True
    # thread_capture.start()

    # thread_ultrasonic = threading.Thread(target=thread_read_ultrasor)
    # thread_ultrasonic.daemon = True
    # thread_ultrasonic.start()

    # thread_face = threading.Thread(target=run_rc_value)
    # thread_face.daemon = True
    # thread_face.start()

    # thread_nano = threading.Thread(target=run_nano)
    # thread_nano.daemon = True
    # thread_nano.start()

    # time.sleep(5) ### it will be complete full socket param initialize
    #
    # thread_drone_speed = threading.Thread(target=drone_speed)
    # thread_drone_speed.daemon = True
    # thread_drone_speed.start()
    #all_value()
    velocity_data_change()
def indoor_altitude_hold(aTargetAltitude):
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.5
    OVER_TAKEOFF_THRUST = 0.4

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        # current_altitude = vehicle.location.global_relative_frame.alt
        # if sonar_alt<4.00:
        current_altitude = sonar_alt
        print(" Altitude: %f  Desired: %f" % (current_altitude, aTargetAltitude))
        if current_altitude == aTargetAltitude * 0.95:
            print("Reached target altitude")
        elif current_altitude>aTargetAltitude:
            thrust = OVER_TAKEOFF_THRUST
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)
def arm_and_takeoff_nogps(aTargetAltitude):
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.5

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        #current_altitude = vehicle.location.global_relative_frame.alt
        #if sonar_alt<4.00:
        current_altitude = sonar_alt
        print(" Altitude: %f  Desired: %f" % (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)

def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000,  # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]
def drone_goto(target_alt,position_hold_duration):
    arm_and_takeoff_nogps(1.000)
    print("Hold position for 3 seconds")
    set_attitude(duration=10)
    print("Move forward")
    # set_attitude(pitch_angle=-5, thrust=0.5, duration=3.21)
    # print("Move backward")
    # set_attitude(pitch_angle=5, thrust=0.5, duration=3)
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)
    if sonar_alt<0.18:
        vehicle.mode = VehicleMode("STABILIZE")


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

##NORTH EASH
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
        0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x,  # X velocity in NED frame in m/s
        velocity_y,  # Y velocity in NED frame in m/s
        velocity_z,  # Z velocity in NED frame in m/s
        0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
if __name__ == '__main__':
    try:
        DbInitilize.DbInitializeClass.dbInit()
        DbInitilize.DbInitializeClass.tbldbWaypoint()
        DbInitilize.DbInitializeClass.tbldbSchedule()

        # insert for first time
        # DbInitilize.DbInitializeClass.dbInitInsert()

        DbInitilize.DbInitializeClass.update_wp_status_false()
        run_thread()
        socketio.run(app)
    except KeyboardInterrupt:
        GPIO.cleanup()
        vehicle.close()
        print("All program stopped by user")






