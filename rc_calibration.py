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
from RC import RcController

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
import RPi.GPIO as GPIO
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

i = -1
def reconnect_io_func():
    try:
        pwm = Adafruit_PCA9685.PCA9685()
        return pwm
    except Exception as error:
        if "Remote I/O error" in (error):
            reconnect_io = True
            while reconnect_io:
                try:
                    # print("while Error: "+str(error))
                    pwm = Adafruit_PCA9685.PCA9685()
                    print(pwm)
                    reconnect_io = False
                    return pwm
                except Exception as error:
                    print((error))
                    reconnect_io = True


servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096
pwm = reconnect_io_func()
from socketIO_client import SocketIO as client_socketio, BaseNamespace

my_client = client_socketio('184.72.95.87', 3000)
str = ""


@socketio.on('chat message')
def handle_message(message):
    print('received message: ' + message)


# my_client.emit('chat message', str)
# print(str)
def doSomething(data):
    try:
        #print (data)
        # vehicle = mpstate.get_vehicles()[0]
        obj_data = json.loads(data.replace('\r\n', '\\r\\n'), strict=False)
        # obj_data = json.loads(data,strict=False)
        # obj_data = json.loads(data)
        sender_user = obj_data['u']
        if (sender_user == User.UserClass.ground_user()):
            val = obj_data[CommandConstant.action]
            print(val)
            if (val == CommandConstant.CommandConstantClass.is_obstacle_enable()):
                global throttle_value
                global roll_value
                global pitch_value
                global yaw_value
                throttle_value = throttle_value + 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(2, throttle_value)
            elif (val == CommandConstant.CommandConstantClass.is_obstacle_disable()):
                throttle_value = throttle_value - 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(2, throttle_value)
                # channel_param = channel_param -2
                # print (channel_param)
                # vehicle.channels.overrides = {'3': channel_param}
                obstacle_enable = False
            elif (val == CommandConstant.CommandConstantClass.is_roll_inc()):
                roll_value = roll_value + 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(0, roll_value)
            elif (val == CommandConstant.CommandConstantClass.is_roll_dec()):
                roll_value = roll_value - 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(0, roll_value)
            elif (val == CommandConstant.CommandConstantClass.is_pitch_inc()):
                pitch_value = pitch_value + 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(1, pitch_value)
            elif (val == CommandConstant.CommandConstantClass.is_pitch_dec()):
                pitch_value = pitch_value - 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(1, pitch_value)

            elif (val == CommandConstant.CommandConstantClass.is_yaw_inc()):
                yaw_value = yaw_value + 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(3, yaw_value)
            elif (val == CommandConstant.CommandConstantClass.is_yaw_dec()):
                yaw_value = yaw_value - 1
                print(throttle_value, roll_value, pitch_value, yaw_value)
                RcController.RcControllerClass.send_rc_command_for_pin(3, yaw_value)
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
                RcController.RcControllerClass.send_rc_command_last_4(rc_4, rc_5, rc_6, rc_7)
            elif (val == CommandConstant.CommandConstantClass.is_pitch_single()):
                print("Channel pitch")
                rc_0 = int(obj_data['channel_0'])
                RcController.RcControllerClass.send_rc_command_for_pin(0, rc_0)
            elif (val == CommandConstant.CommandConstantClass.is_roll_single()):
                rc_1 = int(obj_data['channel_1'])
                RcController.RcControllerClass.send_rc_command_for_pin(1, rc_1)
            elif (val == CommandConstant.CommandConstantClass.is_throttle_single()):
                rc_2 = int(obj_data['channel_2'])
                RcController.RcControllerClass.send_rc_command_for_pin(2, rc_2)
            elif (val == CommandConstant.CommandConstantClass.is_yaw_single()):
                rc_3 = int(obj_data['channel_3'])
                RcController.RcControllerClass.send_rc_command_for_pin(3, rc_3)
            # elif (val == CommandConstant.CommandConstantClass.is_pitch_single()):
            #     rc_0 = int(obj_data['channel_0'])
            #     vehicle.channels.overrides['1'] = remap(rc_0, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            #
            # elif (val == CommandConstant.CommandConstantClass.is_roll_single()):
            #     rc_1 = int(obj_data['channel_1'])
            #     vehicle.channels.overrides['2'] = remap(rc_1, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            #     #RcController.RcControllerClass.send_rc_command_for_pin(1, rc_1)
            # elif (val == CommandConstant.CommandConstantClass.is_throttle_single()):
            #     rc_2 = int(obj_data['channel_2'])
            #     #print("MAP ", rc_2, remap(rc_2, 307, 532, 1089, 1888))
            #     vehicle.channels.overrides['3'] = remap(rc_2, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            #     #RcController.RcControllerClass.send_rc_command_for_pin(2, rc_2)
            # elif (val == CommandConstant.CommandConstantClass.is_yaw_single()):
            #     rc_3 = int(obj_data['channel_3'])
            #     vehicle.channels.overrides['4'] = remap(rc_3, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            #     #RcController.RcControllerClass.send_rc_command_for_pin(3, rc_3)
            # elif (val == CommandConstant.CommandConstantClass.is_channel5_single()):
            #     rc_4 = int(obj_data['channel_4'])
            #     vehicle.channels.overrides['5'] = remap(rc_4, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            # elif (val == CommandConstant.CommandConstantClass.is_channel6_single()):
            #     rc_5 = int(obj_data['channel_5'])
            #     vehicle.channels.overrides['6'] = remap(rc_5, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            # elif (val == CommandConstant.CommandConstantClass.is_channel7_single()):
            #     rc_6 = int(obj_data['channel_6'])
            #     vehicle.channels.overrides['7'] = remap(rc_6, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            # elif (val == CommandConstant.CommandConstantClass.is_channel8_single()):
            #     rc_7 = int(obj_data['channel_7'])
            #     vehicle.channels.overrides['8'] = remap(rc_7, channel_input_min_value, channel_input_max_value, channel_output_min_value, channel_output_max_value)
            # print ("self message")
    except Exception as error:
        print (error)
        # set_command(data)


my_client.on('chat message', doSomething)
my_client.wait()

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000  # 1,000,000 us per second
    pulse_length //= 60  # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096  # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)


# Set frequency to 60hz, good for servos.
# pwm = reconnect_io_func()
pwm.set_pwm_freq(60)

my_client.on('chat message', doSomething)
my_client.wait()

if __name__ == '__main__':
    socketio.run(app)
