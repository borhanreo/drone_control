import os
import io
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from AWS.face import Face
from Utility import JsonBuilder
from Constant import ProjectConstant
from socketIO_client import SocketIO as client_socketio, BaseNamespace
from subprocess import call
class ModCapture(object):
    my_client_send = client_socketio(ProjectConstant.ProjectConstantClass.get_host(),
                                     ProjectConstant.ProjectConstantClass.get_port())
    CAMERA_WIDTH = 1376
    CAMERA_HEIGHT = 768
    FRAME_RATE =20

    # camera = PiCamera()
    # camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
    # camera.framerate = FRAME_RATE
    # rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))
    # time.sleep(0.1)

    face_cascade_path = os.getcwd() + "/AICAPTURE/haarcascade_frontalface_default.xml"
    eye_cascade_path = os.getcwd() + "/AICAPTURE/haarcascade_eye.xml"
    face_cascade = cv2.CascadeClassifier(face_cascade_path)
    eye_cascade = cv2.CascadeClassifier(eye_cascade_path)
    # #image_path = os.getcwd() + "/AWS/target-image/cam.jpg"
    image_path = "/home/pi/development/drone_control/AWS/target-image/cam.jpeg"
    faceapi = Face()
    camera_status = False
    def __init__(self):
        pass

    # def run(self, camera_status=None):
    #     try:
    #         if camera_status==True:
    #             print("Open Again")
    #             self.CAMERA_WIDTH = 1376
    #             self.CAMERA_HEIGHT = 768
    #             self.camera = PiCamera()
    #             self.camera.resolution = (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)
    #             self.camera.framerate = self.FRAME_RATE
    #             self.rawCapture = PiRGBArray(self.camera, size=(self.CAMERA_WIDTH, self.CAMERA_HEIGHT))
    #             time.sleep(0.1)
    #             pass
    #         time.sleep(2)
    #         for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
    #             try:
    #                 img = frame.array
    #                 gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #                 faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
    #                 for (x, y, w, h) in faces:
    #                     self.my_client_send.emit('chat message',JsonBuilder.JsonBuilderClass.get_face_detected())
    #                     print ("detect face")
    #                     cv2.imwrite(self.image_path, img)
    #                     self.my_client_send.emit('chat message', JsonBuilder.JsonBuilderClass.get_face_captured())
    #                     person = self.faceapi.face_s3_match_multi(self.image_path)
    #                     print (person)
    #                     self.speech("hi i am detecting")
    #                     self.my_client_send.emit('chat message', JsonBuilder.JsonBuilderClass.get_aiface_response(person))
    #                 cv2.waitKey(1) & 0xFF
    #                 self.rawCapture.truncate(0)
    #             except Exception as error:
    #                 print("A1 ")
    #                 self.camera.close()
    #                 self.run(True)
    #                 #print (error)
    #     except Exception as e:
    #         print("A2 ")
    #         print (e)

    def speech(self,str):
        cmd_beg = 'espeak -g5 -ven+f4 '
        cmd_end = ' 2>/dev/null'  # To dump the std errors to /dev/null
        str = str.replace(' ', '_')
        str = str.replace(' ', '.')
        str = str.replace(' ', ',')
        call([cmd_beg + str + cmd_end], shell=True)
    def run_usb_cam(self):

        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        time.sleep(2)
        previous_milli = 0
        interval=1000
        while (True):
            ret, img = cam.read()
            #cv2.imshow('frame', img)
            current_milli = int(round(time.time() * 1000))
            if (current_milli - previous_milli) > interval:
                print ("interval ")
                print (current_milli)
                print (previous_milli)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    self.my_client_send.emit('chat message', JsonBuilder.JsonBuilderClass.get_face_detected())
                    print ("detect face")
                    cv2.imwrite(self.image_path, img)
                    self.my_client_send.emit('chat message', JsonBuilder.JsonBuilderClass.get_face_captured())
                    person = self.faceapi.face_s3_match_multi(self.image_path)
                    print (person)
                    if 'John' in person:
                        self.my_client_send.emit('chat message', JsonBuilder.JsonBuilderClass.get_aiface_response_for_john())
                    else:
                        self.my_client_send.emit('chat message',JsonBuilder.JsonBuilderClass.get_aiface_response(person))
                previous_milli = current_milli
                cv2.waitKey(1) & 0xFF
        cam.release()
        cv2.destroyAllWindows()

