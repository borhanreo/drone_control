import cv2
import os

import numpy as np

from AIFACE import face_data_append
from Constant import Server, CommandConstant, User, ProjectConstant, AiFaceConstant

#################################################### START FACE AI
subjects = ["", "Borhan", "Tawhid", "Arif_iOS", "Mamun"]


def detect_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier('AIFACE/face-file/lbpcascade_frontalface.xml')
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
    if (len(faces) == 0):
        return None, None
    (x, y, w, h) = faces[0]
    return gray[y:y + w, x:x + h], faces[0]


def prepare_training_data(data_folder_path):
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
            print("....Processing....", counter)
            counter = counter + 1
            cv2.waitKey(100)
            face, rect = detect_face(image)
            if face is not None:
                faces.append(face)
                labels.append(label)
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    cv2.destroyAllWindows()

    return faces, labels


def predict(test_img, catch=None):
    # make a copy of the image as we don't want to chang original image
    img = test_img.copy()
    # detect face from the image
    print (img)
    face, rect = detect_face(img)
    print ("BB")
    # predict the image using our face recognizer
    try:
        label, confidence = face_recognizer.predict(face)
        # get name of respective label returned by face recognizer
        label_text = subjects[label]
        # speech("hi " + label_text)
        print (label_text, " ", confidence)
        # draw a rectangle around face detected
        # draw_rectangle(img, rect)
        # draw name of predicted person
        # draw_text(img, label_text, rect[0], rect[1] - 5)
        return img
    except:
        print (ValueError)
        return None


print("Preparing data...")
faces, labels = prepare_training_data("AIFACE/training-data")
print("Data prepared")
print("Total faces: ", len(faces))
print("Total labels: ", len(labels))
face_recognizer = cv2.face.LBPHFaceRecognizer_create()
face_recognizer.train(faces, np.array(labels))
val = str(os.getcwd() + "/AIFACE/training-data/s2/" + "1.jpg")
test_img = cv2.imread(val)
# test_img =cv2.imread("cam.jpg")
predicted_img = predict(test_img)
if predicted_img is None:
    print ("I do not know you ")
else:
    cv2.imshow(subjects[1], cv2.resize(predicted_img, (480, 640)))
    cv2.waitKey(0)
