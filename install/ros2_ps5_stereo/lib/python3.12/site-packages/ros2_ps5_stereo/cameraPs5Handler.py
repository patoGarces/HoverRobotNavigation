import time
import numpy as np
import cv2

import logging
import queue

from ros2_ps5_stereo.Utils.utils import CameraStatusUi
from ros2_ps5_stereo.controlCamera import ControlCamera 
from ros2_ps5_stereo.controlCamera import StatusCamera
from ros2_ps5_stereo.getFrame import GetFrame
from ros2_ps5_stereo.getFrame import Resolutions

class CameraPs5Handler():

    statusCameraUi = 0
    cameraResolution = Resolutions.RES_640x480
    cameraFps = 8

    def __init__(self):

        self.controlCamera = ControlCamera()
        self.getFrame =  GetFrame()

        self.zFilterHeight = 0
        self.zFilterThickness = 0.5
        self.depthFilter = 2.5
        self.forwardPose = 0.0

        self.framesQueue = self.getFrame.getQueueGetFrame()

        print('Init Headless server')

        self.statusCameraHardware = StatusCamera.CAMERA_NOT_CONNECTED

        while (self.statusCameraHardware != StatusCamera.CAMERA_CONNECTED_OK):

            self.statusCameraHardware = self.controlCamera.getCameraStatus()
            if( self.statusCameraHardware == StatusCamera.CAMERA_NOT_CONNECTED):
                self.__setStatusCameraUi(CameraStatusUi.NOT_FOUND)
            if( self.statusCameraHardware == StatusCamera.CAMERA_CONNECTED_PENDING_FW):
                self.__setStatusCameraUi(CameraStatusUi.WAITING_FW)

            time.sleep(1)

        self.__setStatusCameraUi(CameraStatusUi.CONNECTED)        
        self.startVideoStream()

    def closeEvent(self, event):
        self.video_capture.release()  # Liberar la captura de la cámara al cerrar la aplicación

    def startVideoStream(self):
        try:
            cameraIndex = self.controlCamera.getCameraIndex()
            if (cameraIndex is not None):
                self.cameraResolution = Resolutions.RES_2560x800
                self.cameraFps = 30
                self.getFrame.startStream(cameraIndex, self.cameraResolution, self.cameraFps)
            else:
                print('Camera index no encontrado')
        except Exception as error :
            print('error', error)
            return False
        else:
            return True

    def stopVideoStream(self):
        try:
            self.getFrame.stopStream()
        except Exception as error :
            print('error', error)
        
        # subscription.dispose()

    def getQueueFrames(self):
        return self.framesQueue

    def __setStatusCameraUi(self,status: CameraStatusUi):
        self.statusCameraUi = status
        print('Status camera: ' + status.value)

        if (status == CameraStatusUi.WAITING_FW):
            print("Cargando FW")
            self.__loadFirmware()
        elif (status == CameraStatusUi.CONNECTED):
            print("Iniciar video")
        elif (status == CameraStatusUi.NOT_FOUND):
            print("Buscando camara")
        elif (status == CameraStatusUi.STREAM_RUNNING):
            print("Stream corriendo")

    def __loadFirmware(self):
        result = self.controlCamera.loadFirmwareCamera()
        if (result != None):
            if(result == True):
                self.__setStatusCameraUi(CameraStatusUi.CONNECTED)
            else:
                print('result: ' + str(result))
                self.__setStatusCameraUi(CameraStatusUi.ERROR)

if __name__ == "__main__":

    # npRet = np.load('DepthParams/param_ret.npy')
    # npK = np.load('DepthParams/param_K.npy')
    # npDist = np.load('DepthParams/param_dist.npy')
    # depthMapProcessor = DepthMapProcessor(npRet,npK,npDist)

    logging.basicConfig(level=logging.NOTSET)

    # DepthmapServerHeadless(depthMapProcessor)
    cameraPs5Handler = CameraPs5Handler()
    queueframeProcessed = cameraPs5Handler.getQueueFrames()

    while True:
        try:
            # Obtiene los frames de la cola de visualización
            frames = queueframeProcessed.get(timeout=1)

            frameR,frameL = frames

            cv2.imshow('frameL', frameR)
            # queueframeProcessed.updateFrames(frames)

            # Salir si se presiona 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except queue.Empty:
            print("No frames received, retrying...")
            continue

    cv2.destroyAllWindows()

