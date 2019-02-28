AI_TRANING_DATA_PATH = "AIFACE/training-data"
AI_FACE_DATA_PATH = "AIFACE/face-file/haarcascade_frontalface.xml"
AI_FACE_NAME_PATH = "/face-name"
class AIFaceConstant:
    def get_training_data_path(self):
        return AI_TRANING_DATA_PATH
    def get_face_data_path(self):
        return AI_FACE_DATA_PATH

    def get_face_name_path(self):
        return AI_FACE_NAME_PATH