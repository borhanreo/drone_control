import os
from Constant import CommandConstant

class SdCardClass:
    @staticmethod
    def file_write(wp_data):
        # if(os.path.isfile(CommandConstant.CommandConstantClass.get_wp_file_name())):
        #     print ("File exit")
        #     os.remove(CommandConstant.CommandConstantClass.get_wp_file_name())
        #     open(CommandConstant.CommandConstantClass.get_wp_file_name(), "w+")
        #     with open(CommandConstant.CommandConstantClass.get_wp_file_name(), 'a') as the_file:
        #         the_file.write(wp_data)
        # else:
        #     print("file not exit")
        #     open(CommandConstant.CommandConstantClass.get_wp_file_name(), "w+")
        #     with open(CommandConstant.CommandConstantClass.get_wp_file_name(), 'a') as the_file:
        #         the_file.write(wp_data)
        try:
            #print(data)
            temp = wp_data
            file = open('wp.txt', 'w')
            for letter in temp:
                if (letter == '*'):
                    file.write(letter.replace('*', '\n'))
                elif (letter == ','):
                    file.write(letter.replace(',', '\t'))
                else:
                    file.write(letter)
            file.close()
        except Exception as ex:
            print(ex)
    @staticmethod
    def param_write(data):
        try:
            #print(data)
            temp = data
            file = open('param.txt', 'w')
            file.write(temp)
            file.close()
        except Exception as ex:
            print(ex)

    @staticmethod
    def write_face_data(data):
        try:
            # print(data)
            temp = str(data)
            file = open('face_rom_data.txt', 'w')
            file.write(temp)
            file.close()
        except Exception as ex:
            print(ex)

    @staticmethod
    def write_face_label(data):
        try:
            # print(data)
            temp = str(data)
            file = open('face_rom_label.txt', 'w')
            file.write(temp)
            file.close()
        except Exception as ex:
            print(ex)

    @staticmethod
    def get_face_name(path):
        subjects = []
        file = open(path, "r")
        for line in file:
            val = line.replace("\n", "")
            subjects.append(val)
        return subjects

    @staticmethod
    def set_face_name(path,new_name):
        subjects = []
        file = open(path, "r")
        for line in file:
            val = line.replace("\n", "")
            subjects.append(val)
        subjects.append(new_name)

        return subjects