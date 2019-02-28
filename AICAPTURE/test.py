import cv2

cam = cv2.VideoCapture(0)
print cam.isOpened()

while(True):
        ret, frame = cam.read()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cam.release()
cv2.destroyAllWindows()