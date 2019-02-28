# Drone Control Python File

Blindfold a friend and try to get him to walk around obstacles using only drone terminology.  Make sure you clear the room of anything dangerous first:

- **Plus Pitch** – Move Forward
- **Minus Pitch** – Move Backward
- **Plus Yaw** – Turn Right
- **Minus Yaw** – Turn Left
- **Plus Roll** – Move Right
- **Minus Roll** – Move Left
- **Plus Throttle** – Stand Taller
- **Minus Throttle** – Crouch

Don’t let them run into anything!

For more details: 
- https://www.robolink.com/lesson-b04-flight-part-ii
- http://ardupilot.org/copter/docs/common-rcmap.html
- https://github.com/dronekit/dronekit-python

## Dependency

1. RPi.GPIO
2. Adafruit_PCA9685
3. flask
4. flask_socketio
5. json 
    
#### Install

**Install Adafruit PCA9685 I2C device with raspberry pi via I2C Bus**
    
    sudo apt-get install python-smbus 
    sudo apt-get install i2c-tools
    # Test is connected device
    sudo i2cdetect -y 0
    # OR
    sudo i2cdetect -y 1
    sudo apt-get install git build-essential python-dev 
    cd ~ git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git 
    cd Adafruit_Python_PCA9685 
    sudo python setup.py install 
    # if you have python3 installed: 
    sudo python3 setup.py install 
    
**Flask install**

    pip uninstall gevent
    sudo pip install socketIO_client
    pip install flask
    pip install flask-socketio 
           
### Download from git

    sudo git clone https://code.leftofthedot.com/borhanreo/drone_control.git
    cd drone_control
    sudo python main.py
    
    
    
### Install socket.io issue

**If you get any problem then need to uninstall gevent**    
    
    pip2 freeze | grep socket
    sudo pip2 uninstall gevent-socketio
    sudo pip2 uninstall gevent-python
    sudo pip2 install python-socketio
    sudo pip2 install socketIO-client
    sudo pip2 install websocket-client
    
# Install mavlink

### Install
    pip install dronekit
    sudo apt-get update
    sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt-dev python-lxml
    sudo pip install future
    sudo pip install pymavlink
    sudo pip install mavproxy
    
### Show list similar usb port identity 
 
    python -m serial.tools.list_ports 
    
/dev/ttyAMA0
/dev/ttyUSB0
/dev/ttyUSB1
3 ports found
##### FACE dependency install 
    export WORKON_HOME=$HOME/.virtualenvs
    export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python2.7    
    source /usr/local/bin/virtualenvwrapper.sh
    source ~/.profile
    mkvirtualenv cv -p python2
    
    
    
    pip install face_recognition
    pip install imutils
#### dlib install
    pip install dlib or install error
    mkdir -p dlib
    git clone -b 'v19.6' --single-branch https://github.com/davisking/dlib.git dlib/
    cd ./dlib
    sudo python setup.py install --compiler-flags "-mfpu=neon"    
    https://gist.github.com/ageitgey/1ac8dbe8572f3f533df6269dab35df65
##### Run for create pickle file
    python encode_faces.py --dataset dataset --encodings encodings.pickle --detection-method hog
##### program run 
    python pi_face_recognition.py --cascade haarcascade_frontalface_default.xml --encodings encodings.pickle
             
### Need to remember for pip is python2 or python3 here we need pip (python 2)

    pip --version    
    
### Connect Ardupilot

    bash run.sh
    
##### Or   
 
    python mavlink_lib.py --master=/dev/ttyACM0 --baudrate 115200 --aircraft MyCopter  

##### OR   
 
    bash run.sh  
       
### Useful mavproxy command

##### show available mod

     mode
     
##### Guided Mode 

    mode guided
    
##### Arm   
 
    arm throttle
    
##### takeoff  
  
    takeoff 40
    
##### Parameter load  
  
    param load ..\Tools\autotest\default_params\copter.parm
    
##### Circle mode    

    mode circle
    param set circle_radius 2000      
       
##### Target altitude 

**Write guided then desire altitude guided ALTITUDE**
    
    guided 100    

**Write guided then desire LAT LNG ALT guided ALTITUDE**    

    guided 22.376666 -121.54464 120

##### JSON 
    
    python mavlink_lib.py --master=/dev/ttyACM0 --baudrate 115200 --aircraft MyCopter    
    {"u":"ground","action":"wp","data":"dfjghdfu"}
    {"u":"ground","action":"read_channels"}
    {"u":"ground","action":"mode","data":"STABILIZE"}
    vehicle = connect('/dev/ttyACM0', wait_ready=True)
    {"u":"ground","action":"takeoff","data",100}
    {"u":"ground","action":"wp_delete"}
    {"u":"ground","action":"wp_schedule"}
    {"u":"ground","action":"wp_schedule_create","s_id":"1","drone_power_on":"1533570618","flight_start":"1533309500"}
    {"u":"ground","action":"wp_schedule_show"}
    {"u":"ground","action":"indoor_fly"}
    {"u":"ground","action":"set_date_time","year":2018,"month":8,"day":8,"hour":15,"minutes",40,"second":40}
    {"u":"ground","action":"wp","data":"QGC WPL 110*0,1,0,16,0,0,0,0,37.619000,-122.381000,10.000000,1*1,0,0,22,0.00000000,0.00000000,0.00000000,0.00000000,37.61905850,-122.38286730,10.000000,1*2,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.61931350,-122.38396170,50.000000,1*3,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.61989130,-122.38484140,50.000000,1*4,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62084310,-122.38501310,50.000000,1*5,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62181190,-122.38486290,50.000000,1*6,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62237270,-122.38351110,50.000000,1*7,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62210080,-122.38248110,50.000000,1*8,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62182890,-122.38108630,50.000000,1*9,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62133600,-122.38031390,50.000000,1*10,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62050320,-122.37962720,50.000000,1*11,0,0,21,0.00000000,0.00000000,0.00000000,0.00000000,37.61953440,-122.38044260,50.000000,1"}
    
    {"u":"ground","action":"rc_03","data":1120}
    {"u":"ground","action":"battery"}
    {"u":"ground","action":"reboot"}
    {"u":"eagle","action":"location","lat":37.61905850,"lon":"122.38286730","alt":10}
    
##### save parameter

    param save ./myparams.parm
    
- http://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- https://ardupilot.github.io/MAVProxy/html/modules/cmdlong.html

#### GPS Data

    master=mpstate.master()
    lat = master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
    lng = master.field('GLOBAL_POSITION_INT', 'lon', 0) * 1.0e-7 
           
#### Auto open a terminal 

**To auto-start the terminal on boot, open this file with nano:**

    nano ~/.config/lxsession/LXDE-pi/autostart
**Add this line to the end of the file:**

    @lxterminal
**Close, save and reboot**
        
    sudo reboot
    
    
##### Motor can not sync 

-https://youtu.be/Y8G3tua0ezI
 
Power on rpi and run.. **Drone power will be off**   

    All power shuld be off
    cd /home/pi/development/drone_control
    python  python obsAI.py
    RC 3 HIGH / THROTTLE 100% MAXIMUM**
    POWER ON/ CONNECT BATTERY  DRONE** 
    AFTER BEEF COMPLETED then again drone battery power OFF**    
    power on or Plug battery again
    After beef completer then throttle going to minimum**      
    unplug again
    
    
####Reference    

##### Dronekit
http://ardupilot.org/copter/docs/common-lightware-sf40c-objectavoidance.html 
  
##### OpencV target set 
https://www.pyimagesearch.com/2015/05/04/target-acquired-finding-targets-in-drone-and-quadcopter-video-streams-using-python-and-opencv/

##### Face
https://www.pyimagesearch.com/2018/06/18/face-recognition-with-opencv-python-and-deep-learning/


###FACE API 
https://www.pyimagesearch.com/2018/06/25/raspberry-pi-face-recognition/

