import json
from Constant import CommandConstant, User
class JsonBuilderClass:
    @staticmethod
    def get_start_information(vehicle):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.gps # 'gps'
        start_data[CommandConstant.lat] = vehicle.location.global_frame.lat #lat
        start_data[CommandConstant.lon] = vehicle.location.global_frame.lon #lon
        start_data[CommandConstant.mode] =  vehicle.mode.name #mode
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_arm():
        start_data = {}
        start_data['u'] = User.UserClass.self_user()  # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.arm # 'arm'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_takeoff():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.takeoff # 'takeoff'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_no_waypoint():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.no_waypoint # 'no_waypoint'
        json_data = json.dumps(start_data)
        return json_data
    @staticmethod
    def get_location_information(lat, lon, alt):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.location # 'location'
        start_data[CommandConstant.lat] = lat
        start_data[CommandConstant.lon] = lon
        start_data[CommandConstant.alt] = alt
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_battery_information(voltage, current, level):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.battery # 'battery'
        start_data[CommandConstant.voltage] = voltage
        start_data[CommandConstant.current] = current
        start_data[CommandConstant.level] = level
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_all_information(b_voltage, b_current, b_level, gps_fix, gps_num_sat, gps_lat, gps_lon, gps_alt):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data['action'] = 'all_info'
        start_data['b_voltage'] = b_voltage
        start_data['b_current'] = b_current
        start_data['b_level'] = b_level
        start_data['gps_fix'] = gps_fix
        start_data['gps_num_sat'] = gps_num_sat
        start_data['gps_lat'] = gps_lat
        start_data['gps_lon'] = gps_lon
        start_data['gps_alt'] = gps_alt
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_mode_information(mode):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.mode # 'mode'
        start_data[CommandConstant.data] = mode
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_gps_inf_information(fix, sat):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.gps_info # 'gps_info'
        start_data[CommandConstant.fix] = fix
        start_data[CommandConstant.sat_num] = sat
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_waypoint_received_response():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.wp_res # 'wp_res'
        start_data[CommandConstant.value] = True
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_waypoint_id_name_array_to_ground(array):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.wp_schedule # 'wp_schedule'
        start_data[CommandConstant.array] = array
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_waypoint_time_schedulle(array):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.wp_schedule_show_res # 'wp_schedule_show_res'
        start_data[CommandConstant.array] = array
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def receive_schedule_response():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.array] = CommandConstant.res_schedule # 'res_schedule'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_all_schedule_response(array):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.all_schedule_res # 'all_schedule_res'
        start_data[CommandConstant.array] = array
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_wp_delete():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.dl_wp_by_id_res # 'dl_wp_by_id_res'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_schedule_delete():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.dl_schedule_by_id # 'dl_schedule_by_id'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_res_obs_start():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.res_obs_start # 'res_obs_start'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_is_res_obs_stop():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.res_obs_stop # 'res_obs_stop'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_speed_json(airsped=0.0,groundspeed=0.0):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.speed # 'speed'
        start_data[CommandConstant.air] = str(airsped)
        start_data[CommandConstant.ground] = str(groundspeed)
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_compass_value(pitch=0.0, roll=0.0,yaw=0.0,air=0.0,ground=0.0):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.gyroscope # 'gyroscope'
        start_data[CommandConstant.pitch] = str(pitch)
        start_data[CommandConstant.roll] = str(roll)
        start_data[CommandConstant.yaw] = str(yaw)
        start_data[CommandConstant.air] = str(air)
        start_data[CommandConstant.ground] = str(ground)
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_aiface_response(array):
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.face_capture_res # 'face_capture_res'
        start_data['u'] = User.UserClass.self_user()
        start_data['str']  = json.dumps(array)
        json_data = json.dumps(start_data)
        return json_data

    ########## ferdib - al - islam start
    @staticmethod
    def get_object_detection_response_on():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.obj_started # 'obj_started'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_object_detection_response_off():
        start_data = {}
        start_data['u'] = User.UserClass.self_user() # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.obj_stop # 'obj_stop'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_face_detected():
        start_data = {}
        start_data['u'] = User.UserClass.self_user()  # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.start_face_detect  # 'obj_stop'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_face_captured():
        start_data = {}
        start_data['u'] = User.UserClass.self_user()  # 'eagle'
        start_data[CommandConstant.action] = CommandConstant.start_face_captured  # 'obj_stop'
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_aiface_response_for_john():
        start_data = {}
        start_data[CommandConstant.action] = CommandConstant.face_capture_res_john  # 'face_capture_res'
        start_data['u'] = User.UserClass.self_user()
        json_data = json.dumps(start_data)
        return json_data

    @staticmethod
    def get_aiface_response_none():
        start_data = {}
        start_data[CommandConstant.action] = CommandConstant.face_capture_res_none  # 'face_capture_res'
        start_data['u'] = User.UserClass.self_user()
        json_data = json.dumps(start_data)
        return json_data



