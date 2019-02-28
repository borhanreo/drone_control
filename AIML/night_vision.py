import math

import cv2
import imutils
import numpy as np

KNOWN_ANGLE = 5
PIXEL_TOM_CM_RATIO = 1


def DistanceBetweenTwoPoint(p1, p2):
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    return dist


def GetAngleOfLineBetweenTwoPoints(p1, p2):
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    return math.degrees(math.atan2(yDiff, xDiff))


def Midpoint(p1, p2):
    return (int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2))


# Captura de video a traves de la webcam
cap = cv2.VideoCapture(0)

while (True):
    CM = 0
    centers = []

    _, frame = cap.read()
    img = imutils.resize(frame, width=800)

    width, height = img.shape[:2]
    img_center = (int(height / 2), int(width / 2))

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # A histogram is obtained based on the saturations of colors.

    # lower mask (0-10)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0 + mask1

    kernal = np.ones((5, 5), "uint8")  # Create a 5x5 matrix which will travel the video,

    mask = cv2.erode(mask, kernal, iterations=1)  # It is eroded using the kernel on the mask.
    res1 = cv2.bitwise_and(img, img, mask=mask)  # The new image will replace blue.

    cv2.circle(img, img_center, 7, (255, 255, 255), -1)

    (_, contours, hierarchy) = cv2.findContours(mask, cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)  # Find the contours of the objects seen in the filter

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)  # opencv function that gets the contours
        if area > 0 and area < 50:
            M = cv2.moments(contour)  # The center of mass of the conflicting markers is obtained.
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            D = DistanceBetweenTwoPoint(img_center, (cx, cy))
            A = GetAngleOfLineBetweenTwoPoints(img_center, (cx, cy))
            if D > 50 and (abs(A) < 5 or abs(A) > 175):
                centers.append((cx, cy))
                cv2.circle(img, (cx, cy), 7, (255, 255, 255), -1)

    centers_len = len(centers)
    if centers_len > 1:
        for center in centers:
            p1 = centers[centers_len - 2]
            p2 = centers[centers_len - 1]
            if (p1[0] < img_center[0] and p2[0] > img_center[0]) or (p1[0] > img_center[0] and p2[0] < img_center[0]):
                M = Midpoint(p1, p2)
                D = DistanceBetweenTwoPoint(p1, p2)
                A = GetAngleOfLineBetweenTwoPoints(p1, p2)
                if D > 100 and (abs(A) < 5 or abs(A) > 175):
                    cv2.line(img, p1, p2, (255, 0, 0), 2)
                    cv2.putText(img, "%.2fpx" % D, M, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255))
                    CM = int(PIXEL_TOM_CM_RATIO * D / (2 * math.atan(KNOWN_ANGLE)))
                    print(D, CM)

                cv2.putText(img, "%.2fcm" % CM,
                            (img.shape[1] - 350, img.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            2.0, (255, 0, 0), 3)

    # show the output frame
    cv2.imshow("Frame", img)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
