import os
import cv2
import numpy

from Constant import AiFaceConstant
face_recognizer = None
subjects = ["", "Borhan","Tawhid", "Arif_iOS","Mamun"]
class FaceDataAppend:
    @classmethod
    def set_face_recognizer(self,s_face_recognizer):
        global face_recognizer
        face_recognizer=s_face_recognizer
    @classmethod
    def prepare_training_data(self,data_folder_path):
        dirs = os.listdir(data_folder_path)
        faces = []
        labels = []
        counter = 0
        for dir_name in dirs:
            if not dir_name.startswith("s"):
                continue
            label = int(dir_name.replace("s", ""))
            subject_dir_path = data_folder_path + "/" + dir_name
            subject_images_names = os.listdir(subject_dir_path)
            for image_name in subject_images_names:
                if image_name.startswith("."):
                    continue
                image_path = subject_dir_path + "/" + image_name
                image = cv2.imread(image_path)
                print(".....image....Processing....", counter)
                counter = counter + 1
                cv2.waitKey(100)
                face, rect = self.detect_face(image)
                if face is not None:

                    faces.append(face)
                    labels.append(label)

        cv2.destroyAllWindows()
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        return faces, labels
    @classmethod
    def detect_face(self,img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        face_cascade = cv2.CascadeClassifier(AiFaceConstant.AI_FACE_DATA_PATH)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
        if (len(faces) == 0):
            return None, None
        (x, y, w, h) = faces[0]
        # numpy.set_printoptions(threshold='nan')
        # print gray[y:y + w, x:x + h], faces[0]
        return gray[y:y + w, x:x + h], faces[0]
    @classmethod
    def draw_rectangle(img, rect):
        (x, y, w, h) = rect
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    @classmethod
    def draw_text(img, text, x, y):
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)

    @classmethod
    def predict(self,test_img, catch=None,fn=None):
        img = test_img.copy()
        print (img)
        face, rect = self.detect_face(img)
        try:
            label, confidence = fn.predict(face)
            label_text = subjects[label]
            print (label_text, " ", confidence)
            self.draw_rectangle(img, rect)
            self.draw_text(img, label_text, rect[0], rect[1] - 5)
            return img
        except:
            print (ValueError)
            return None
    @classmethod
    def get_subject(self):
        return subjects

