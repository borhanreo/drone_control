#### Control

     python mavlink.py --master=/dev/ttyACM0 --baudrate 115200 --aircraft MyCopter

#### start  json

    @lxpanel --profile LXDE-pi
    @pcmanfm --desktop --profile LXDE-pi
    @xscreensaver -no-splash
    @point-rpi
    {"u":"ground","action":"arm"}
    cd /home/pi/development/drone_control
    python mavlink_lib.py --master=/dev/ttyACM0 --baudrate 115200 --aircraft MyCopter
    
    {"u":"ground","action":"wp","data":"dfjghdfu"}
    {"u":"ground","action":"read_channels"}
    {"u":"ground","action":"mode","data":"STABILIZE"}
    vehicle = connect('/dev/ttyACM0', wait_ready=True)
    {"u":"ground","action":"takeoff","data",100}
    {"u":"ground","action":"wp_delete"}
    {"u":"ground","action":"indoor_fly"}
    {"u":"ground","action":"wp_schedule"}
    {"u":"ground","action":"wp_schedule_create","s_id":"1","drone_power_on":"1533570618","flight_start":"1533309500"}
    {"u":"ground","action":"wp_schedule_show"}
    {"u":"ground","action":"all_schedule"}
    {"u":"ground","action":"dl_wp_by_id","id":1}
    {"u":"ground","action":"dl_schedule_by_id","id":1}
    {"u":"ground","action":"test_takeoff"}
    
    {"u":"ground","action":"obs_start"}
    {"u":"ground","action":"obs_stop"}
    {"u":"ground","action":"res_obs_start"}
    {"u":"ground","action":"res_obs_stop"}
    
    
    {"u":"ground","action":"set_date_time","year":2020,"month":8,"day":14,"hour":11,"minutes",33,"second":40}
    {"u":"ground","name":"t1","action":"wp","data":"QGC WPL 110*0,1,0,16,0,0,0,0,37.619000,-122.381000,10.000000,1*1,0,0,22,0.00000000,0.00000000,0.00000000,0.00000000,37.61905850,-122.38286730,10.000000,1*2,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.61931350,-122.38396170,50.000000,1*3,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.61989130,-122.38484140,50.000000,1*4,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62084310,-122.38501310,50.000000,1*5,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62181190,-122.38486290,50.000000,1*6,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62237270,-122.38351110,50.000000,1*7,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62210080,-122.38248110,50.000000,1*8,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62182890,-122.38108630,50.000000,1*9,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62133600,-122.38031390,50.000000,1*10,0,0,16,0.00000000,0.00000000,0.00000000,0.00000000,37.62050320,-122.37962720,50.000000,1*11,0,0,21,0.00000000,0.00000000,0.00000000,0.00000000,37.61953440,-122.38044260,50.000000,1"}
    
    {"u":"ground","action":"rc_03","data":1120}
    {"u":"ground","action":"battery"}
    {"u":"ground","action":"reboot"}
    Eagle
    {"u":"eagle","action":"location","lat":37.61905850,"lon":"122.38286730","alt":10}
    {"u":"eagle","action":"all_schedule_res"}
    {"u":"ground","action":"dl_wp_by_id_res"}
    {"u":"ground","action":"dl_schedule_by_id"}
    # drone map open
    {"u":"ground","action":"give_drone_location"}
    # Face AI match
    {"u":"ground","action":"face_capture"}
    {"u":"ground","action":"face_learn","name":"borhan"}
    
    
    
    