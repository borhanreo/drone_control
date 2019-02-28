start = '100'
home_location = '101'
read_channels = '102'
wp = '103'
arm = '104'
disarm = '105'
mode = '106'
takeoff = '107'
takeoff_land = '108'
store_param = '109'
reboot = '110'
battery = '111'
rc_03 = '112'
manual_fly = '113'
wp_delete = '114'
wp_delete_one = '115'
wp_schedule = '116'
wp_schedule_create = '117'
wp_schedule_show = '118'
set_date_time = '119'
indoor_fly = '120'
all_schedule = '121'
all_schedule_res = '122'
dl_wp_by_id = '123'
dl_schedule_by_id = '124'
test_takeoff = '125'
obs_start = '126'
obs_stop = '127'
res_obs_start = '128'
res_obs_stop = '129'
face_capture = '130'
face_capture_res = '131'
stop_Object_Detect = '132'
face_learn = '133'
gps = '134'
action = '135'
lat = '136'
lon = '137'
alt = '138'
voltage = '139'
current = '140'
level = '141'
data = '142'
fix = '143'
sat_num = '144'
value = '145'
array = '146'
location = '147'
roll  = '148'
gps_info = '149'
wp_res = '150'
wp_schedule_show_res = '151'
res_schedule = '152'
dl_wp_by_id_res = '153'
speed = '154'
gyroscope = '155'
obj_started = '156'
obj_stop = '157'
ground = '159'
air = '160'
yaw = '161'
pitch = '162'
no_waypoint = '163'
start_Object_Detect = '164'
start_face_detect = '165'
start_face_captured = '166'
face_capture_res_john = '167'
face_capture_res_none = '168'
obstacle_enable = '172'
obstacle_disable = '173'


roll_inc = '180'
roll_dec = '181'
pitch_inc = '182'
pitch_dec = '183'
yaw_inc = '184'
yaw_dec = '185'


pitch_single = '190'
roll_single='191'
throttle_single='192'
yaw_single='193'
channel5='194'
channel6='195'
channel7='196'
channel8='197'
is_ultrasonic_enable_disable_code1 = '200'
is_ultrasonic_enable_disable_code2 = '201'
automatic_left = '204'
automatic_right = '205'
automatic_refresh = '206'
class CommandConstantClass:
    str_user = ""
    str_eagleeye = ""

    def __init__(self):
        self.str_user = "borhan"
        self.str_eagleeye = "eagleeye"
        print ("ProjectConstantClass")

    @staticmethod
    def is_equal_start_drone():
        # return 'start'
        return start

    @staticmethod
    def is_equal_home_location():
        # return 'home_location'
        return home_location

    @staticmethod
    def is_equal_read_channels():
        # return 'read_channels'
        return read_channels

    @staticmethod
    def is_equal_wp():
        # return 'wp'
        return wp

    @staticmethod
    def is_equal_arm():
        # return 'arm'
        return arm

    @staticmethod
    def is_equal_disarm():
        # return 'disarm'
        return disarm

    @staticmethod
    def is_equal_mode():
        # return 'mode'
        return mode

    @staticmethod
    def is_equal_takeoff():
        # return 'takeoff'
        return takeoff

    @staticmethod
    def is_equal_takeoff_land():
        # return 'takeoff_land'
        return takeoff_land

    @staticmethod
    def is_equal_store_param():
        # return 'store_param'
        return store_param

    @staticmethod
    def is_equal_reboot():
        # return 'reboot'
        return reboot

    @staticmethod
    def is_equal_battery_info():
        # return 'battery'
        return battery

    @staticmethod
    def is_equal_rc_03():
        # return 'rc_03'
        return rc_03

    @staticmethod
    def is_equal_manual_fly():
        # return 'manual_fly'
        return manual_fly

    @staticmethod
    def get_wp_file_name():
        return 'wp.txt'

    @staticmethod
    def is_equal_waypoint_delete():
        # return 'wp_delete'
        return wp_delete

    @staticmethod
    def is_equal_waypoint_delete_one():
        # return 'wp_delete_one'
        return wp_delete_one

    @staticmethod
    def is_equal_send_waypoint_id_name():
        # return 'wp_schedule'
        return wp_schedule

    @staticmethod
    def is_equal_schedule_create():
        # return 'wp_schedule_create'
        return wp_schedule_create

    @staticmethod
    def is_equal_schedule_show_data():
        # return 'wp_schedule_show'
        return wp_schedule_show

    @staticmethod
    def is_equal_set_date_time():
        # return 'set_date_time'
        return set_date_time

    @staticmethod
    def is_equal_indoor_fly():
        # return 'indoor_fly'
        return indoor_fly

    @staticmethod
    def is_equal_schedule_all():
        # return 'all_schedule'
        return all_schedule

    @staticmethod
    def is_equal_schedule_all_res():
        # return 'all_schedule_res'
        return all_schedule_res

    @staticmethod
    def is_equal_delete_wp_by_id():
        # return 'dl_wp_by_id'
        return dl_wp_by_id

    @staticmethod
    def is_equal_delete_schedule_by_id():
        # return 'dl_schedule_by_id'
        return dl_schedule_by_id

    @staticmethod
    def is_equal_test_takoff():
        # return 'test_takeoff'
        return test_takeoff

    @staticmethod
    def is_equal_obs_start():
        # return 'obs_start'
        return obs_start

    @staticmethod
    def is_equal_obs_stop():
        # return 'obs_stop'
        return obs_stop

    @staticmethod
    def is_equal_obs_start_res():
        # return 'res_obs_start'
        return res_obs_start

    @staticmethod
    def is_equal_obs_stop_res():
        # return 'res_obs_stop'
        return res_obs_stop

    @staticmethod
    def is_equal_face_capture():
        # return 'face_capture'
        return face_capture

    @staticmethod
    def is_equal_face_capture_res():
        # return 'face_capture_res'
        return face_capture_res

    @staticmethod
    def is_object_detection_start():
        # return 'start_Object_Detect'
        return start_Object_Detect

    @staticmethod
    def is_object_detection_stop():
        # return 'stop_Object_Detect'
        return stop_Object_Detect

    @staticmethod
    def is_face_learn():
        # return 'face_learn'
        return face_learn

    @staticmethod
    def is_face_detect():
        # return 'face_learn'
        return start_face_detect

    @staticmethod
    def is_face_captured():
        # return 'face_learn'
        return start_face_captured

    @staticmethod
    def is_obstacle_enable():
        return obstacle_enable
    @staticmethod
    def is_obstacle_disable():
        return obstacle_disable

    @staticmethod
    def is_roll_inc():
        return roll_inc

    @staticmethod
    def is_roll_dec():
        return roll_dec

    @staticmethod
    def is_pitch_inc():
        return pitch_inc

    @staticmethod
    def is_pitch_dec():
        return pitch_dec

    @staticmethod
    def is_yaw_inc():
        return yaw_inc
    @staticmethod
    def is_yaw_dec():
        return yaw_dec

    @staticmethod
    def is_throttle():
        return throttle_single

    @staticmethod
    def is_pitch_single():
        return pitch_single

    @staticmethod
    def is_roll_single():
        return roll_single

    @staticmethod
    def is_throttle_single():
        return throttle_single

    @staticmethod
    def is_yaw_single():
        return yaw_single

    @staticmethod
    def is_channel5_single():
        return channel5

    @staticmethod
    def is_channel6_single():
        return channel6

    @staticmethod
    def is_channel7_single():
        return channel7

    @staticmethod
    def is_channel8_single():
        return channel8
    @staticmethod
    def is_ultrasonic_enable_disable1():
        return is_ultrasonic_enable_disable_code1

    @staticmethod
    def is_ultrasonic_enable_disable2():
        return is_ultrasonic_enable_disable_code2

    @staticmethod
    def is_automatic_left():
        return automatic_left

    @staticmethod
    def is_automatic_right():
        return automatic_right

    @staticmethod
    def is_automatic_refresh():
        return automatic_refresh
    ###Borhan start_face_detect


# pitch_single = '190'
# roll_single = '191'
# throttle_single = '192'
# yaw_single = '193'