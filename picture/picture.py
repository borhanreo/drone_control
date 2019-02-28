import os
import time
class Picture:
    def __init__(self):
        pass
    def start(self):
        for x in range(100):
            path = os.getcwd()+ "/AIFACE/training-data/s6/"+str(x+1)+".jpg"
            os.system("raspistill -w 480 -h 640 -o "+path)
            time.sleep(0.1)