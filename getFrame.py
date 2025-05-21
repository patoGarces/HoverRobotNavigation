import cv2
from enum import Enum
from cv2_enumerate_cameras import enumerate_cameras
import threading

import queue

WINDOWS_PLATFORM_NAME = 'Windows'
LINUX_PLATFORM_NAME = 'Linux'

class GetFrame:

    # out_full = cv2.VideoWriter('full.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(3448,808))
    # out_left = cv2.VideoWriter('left.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))
    # out_right = cv2.VideoWriter('right.avi',cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 40,(1264,800))

    def __init__(self):
        self.flagRunning = threading.Event()
        self.flagRunning.clear()
        self.hilo_emision = None
        self.queueFrames = queue.Queue(maxsize=1)
        
    def getQueueGetFrame(self):
        return self.queueFrames

    def decode(self,frame):

        height, width, _ = frame.shape
        mid = width // 2
        left_frame = frame[:, :mid, :]
        right_frame = frame[:, mid:, :]

        left_frame = cv2.resize(left_frame,(1264,800))
        right_frame = cv2.resize(right_frame,(1264,800))
        
        return (left_frame, right_frame)

    def getNewFrame(self):

        while self.flagRunning.isSet():
            ret, frame = self.cap.read()
            if not ret:
                break
            left, right = self.decode(frame)

            # self.out_full.write(frame)
            # self.out_left.write(left)
            # self.out_right.write(right)

            self.queueFrames.put((left,right))

    def startStream(self, platform, vid_camera, resolution, fps):
        cameraIndex = -1
        contIndex = 0

        cv2Cap = cv2.CAP_MSMF
        if(platform == LINUX_PLATFORM_NAME):
            cv2Cap = cv2.CAP_V4L2 

        for camera_info in enumerate_cameras(cv2Cap): # or cv2.CAP_DSHOW

            if( vid_camera.lower() in camera_info.path.lower()):
                cameraIndex = contIndex
                break
            contIndex+=1
        
        if( cameraIndex == -1 ):
            print('GetFrames error camera no encontrada')
            return False
        
        self.cap = cv2.VideoCapture(cameraIndex)

        if not self.cap.isOpened:
            print("Camera is not found")
            return False
        else:
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
            # self.cap.set(cv2.CAP_PROP_FPS, 8)
            try:
                if (resolution == Resolutions.RES_640x480):
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1080)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)        
                elif (resolution == Resolutions.RES_2560x800):   
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
                elif (resolution == Resolutions.RES_3448x808):
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
                else:
                    print("Error al seleccionar la resolucion,resValue: ",resolution)

                self.cap.set(cv2.CAP_PROP_FPS, fps)

                self.flagRunning.set()
                self.hilo_emision = threading.Thread(target=self.getNewFrame)
                self.hilo_emision.start()
                return True
            except Exception as error:
                print("error: ",error)
                return False

    def stopStream(self):
        self.flagRunning.clear()
        self.hilo_emision = None
        # self.cap.release()

    def showPreviewRgb(self,frameLeft,frameRight):
        left = cv2.resize(frameLeft,(int(256*3),int(108*4.5)))
        right = cv2.resize(frameRight,(int(256*3),int(108*4.5)))
        cv2.imshow('RGB left', left)
        cv2.imshow('RGB right', right)


class Resolutions(Enum):          # TODO: implementar
    RES_3448x808 = 0
    RES_2560x800 = 1
    RES_640x480 = 2
