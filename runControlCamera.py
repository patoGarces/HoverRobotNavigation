import sys
import time
import numpy as np
import cv2

import logging
import queue

import platform

from Utils.utils import CameraStatusUi
from controlCamera import ControlCamera 
from controlCamera import StatusCamera
from getFrame import GetFrame
from getFrame import Resolutions

VID_CAMERA = 'VID_05A9'

WINDOWS_PLATFORM_NAME = 'Windows'
LINUX_PLATFORM_NAME = 'Linux'

class DepthmapServerHeadless():

    statusCameraUi = 0
    cameraResolution = Resolutions.RES_640x480
    cameraFps = 8

    # def __init__(self,_depthMapProcessor):
    def __init__(self):

        # self.communicator = Communicator()  # Crear la instancia de Communicator
        self.controlCamera = ControlCamera()
        self.getFrame =  GetFrame()

        self.zFilterHeight = 0
        self.zFilterThickness = 0.5
        self.depthFilter = 2.5
        self.forwardPose = 0.0

        self.framesQueue = self.getFrame.getQueueGetFrame()

        if not hasattr(self, 'nube_puntos'):
            self.output2D = []
        # self.output2D = []

        print('Init Headless server')
        # self.updateCameraStatus()

        self.statusCameraHardware = StatusCamera.CAMERA_NOT_CONNECTED

        while (self.statusCameraHardware != StatusCamera.CAMERA_CONNECTED_OK):

            self.statusCameraHardware = self.controlCamera.getCameraStatus()
            if( self.statusCameraHardware == StatusCamera.CAMERA_NOT_CONNECTED):
                self.setStatusCameraUi(CameraStatusUi.NOT_FOUND)
            if( self.statusCameraHardware == StatusCamera.CAMERA_CONNECTED_PENDING_FW):
                self.setStatusCameraUi(CameraStatusUi.WAITING_FW)

            time.sleep(1)


        self.setStatusCameraUi(CameraStatusUi.CONNECTED)        

        self.startVideoStream()
        self.showFrameProcessed()

    def processFrames(self,frames):

        frameR,frameL = frames
        # frameL, frameR = self.depthMapProcessor.rectifiedFrames(frameL,frameR)
        undisFrameL, undisFrameR = self.depthMapProcessor.downsampledFrames(frameL,frameR)
        # undisFrameL, undisFrameR = self.depthMapProcessor.getHorizontalStripe(frameL,frameR)
        
        depthMapColor,points,disp,disp_color,depthMap = self.depthMapProcessor.getStereoDepthmap(undisFrameL,undisFrameR)

        frame = cv2.cvtColor(undisFrameL, cv2.COLOR_BGR2RGB)                       # Convertir el fotograma a RGB
        # self.video_labelL.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel

        frame = cv2.cvtColor(undisFrameR, cv2.COLOR_BGR2RGB)                       # Convertir el fotograma a RGB
        # self.video_labelR.setPixmap(self.convertFrameToQt(frame))                  # Mostrar el fotograma en el QLabel
        frame = cv2.cvtColor(depthMapColor, cv2.COLOR_BGR2RGB)                     # Convertir el fotograma a RGB

        frameWithLine = cv2.line(frame, (0, int(frame.shape[0]/2)), (frame.shape[1], int(frame.shape[0]/2)), (255, 255, 255), 1)
        # self.video_labelDepthMap.setPixmap(self.convertFrameToQt(frameWithLine,self.video_labelDepthMap.width(),self.video_labelDepthMap.height()))           # Mostrar el fotograma en el QLabel

        cv2.imshow("depth frame",frameWithLine)

        # z_max = -2.3
        z_max = -self.depthFilter               # TODO: remover el '-' cuando este bien ubicada la nube de puntos espacialmente
        mask = (points[:, :, 2] >= z_max)
        points = points[mask]

        num_points = points.shape[0]  # Solo usamos el primer eje

        # Concatenamos los puntos homogéneos
        points_homogeneous = np.concatenate((points, np.ones((num_points, 1))), axis=1)

        # Rotación de la nube de puntos
        yaw = np.radians(self.cameraPose[2])
        Qyaw = np.float32([[np.cos(yaw)     , 0 , np.sin(yaw)   , 0],
                            [0              , 1 , 0             , self.forwardPose],
                            [-np.sin(yaw)   , 0 , np.cos(yaw)   , 0],
                            [0              , 0 , 0             , 1]])
        
        pitch = np.radians(self.cameraPose[0])
        Qpitch = np.float32([[  1     , 0               , 0                 , 0],
                            [   0     , np.cos(pitch)   , -np.sin(pitch)    , 0],
                            [   0     , np.sin(pitch)   , np.cos(pitch)     , 0],
                            [   0     , 0               , 0                 , 1]])

        roll = np.radians(self.cameraPose[1])

        print('Pitch: '+ str(round(pitch,2)) + '   Roll: '+ str(round(roll,2)) + '    Yaw: ' + str(round(yaw,2)) + '    Forward: ' + str(self.forwardPose))
        # Multiplico por la matriz de rotación
        rotated_points = np.dot(points_homogeneous, Qyaw.T)
        rotated_points = np.dot(rotated_points, Qpitch.T)

        rotated_points = rotated_points[:, :3]          # Elimino la cuarta columna

        # print('size points: ' + str(np.size(points)) + 'disp max: ' +str(disp.max()))
        # mask = (disp > 0) & (disp < 255)     # Descarto puntos invalidos
        # points = points[mask]
        # print('size points mask: ' + str(np.size(points)))

        out_colors = cv2.cvtColor(disp_color, cv2.COLOR_BGR2RGB)[mask]

        rotated_points = rotated_points.reshape(-1, 3)         # Aplano a (800*1264, 3)

        # # Agrega los puntos rotados actuales a la lista acumulativa
        # self.output2D.append(rotated_points)
        # # Convierte la lista acumulada en un array de NumPy para su procesamiento
        # nube_puntos_acumulada = np.vstack(self.output2D)
        # self.drawer2DWidget.updatePlot(nube_puntos_acumulada,out_colors)

        # self.drawer2DWidget.updatePlot(rotated_points,out_colors)
        self.plotter2D.updatePlot(rotated_points,out_colors)

        y_min = self.zFilterHeight - self.zFilterThickness
        y_max = self.zFilterHeight + self.zFilterThickness    

        index_filter = (rotated_points[:, 1] >= y_min) & (rotated_points[:, 1] <= y_max)
        # Filtro los puntos que cumplen la condicion
        rotated_points = rotated_points[index_filter]
        out_colors = out_colors[index_filter]

        self.communicator.update_cloud.emit(rotated_points, out_colors)  # Emitir la señal

    def closeEvent(self, event):
        self.video_capture.release()  # Liberar la captura de la cámara al cerrar la aplicación

    def startVideoStream(self):
        try:
            self.getFrame.startStream(platform.system(), VID_CAMERA, self.cameraResolution, self.cameraFps)
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

    def changeResolution(self,index):                   # optimizar
        if (index == 0):
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 8
        elif (index == 1):
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 30
        elif (index == 2):
            self.cameraResolution = Resolutions.RES_640x480
            self.cameraFps = 60
        if (index == 3):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 8
        elif (index == 4):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 30
        elif (index == 5):
            self.cameraResolution = Resolutions.RES_2560x800
            self.cameraFps = 60
        elif (index == 6):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 8
        elif (index == 7):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 30
        elif (index == 8):
            self.cameraResolution = Resolutions.RES_3448x808
            self.cameraFps = 60

        self.stopVideoStream()
        self.startVideoStream()

    def setStatusCameraUi(self,status: CameraStatusUi):
        self.statusCameraUi = status
        print('Status camera: ' + status.value)

        if (status == CameraStatusUi.WAITING_FW):
            print("Cargando FW")
            self.loadFirmware()
        elif (status == CameraStatusUi.CONNECTED):
            print("Iniciar video")
        elif (status == CameraStatusUi.NOT_FOUND):
            print("Buscando camara")
        elif (status == CameraStatusUi.STREAM_RUNNING):
            print("Stream corriendo")
    
    def updateCameraStatus(self):
        statusCameraHardware = self.controlCamera.getCameraStatus()
        if( statusCameraHardware == StatusCamera.CAMERA_NOT_CONNECTED ):
            self.setStatusCameraUi(CameraStatusUi.NOT_FOUND)
        elif( statusCameraHardware == StatusCamera.CAMERA_CONNECTED_PENDING_FW ):
            self.setStatusCameraUi(CameraStatusUi.WAITING_FW)
        elif( statusCameraHardware == StatusCamera.CAMERA_CONNECTED_OK ):
            self.setStatusCameraUi( CameraStatusUi.CONNECTED)

    def loadFirmware(self):
        result = self.controlCamera.loadFirmwareCamera()
        if (result != None):
            if(result == True):
                self.setStatusCameraUi(CameraStatusUi.CONNECTED)
            else:
                print('result: ' + str(result))
                self.setStatusCameraUi(CameraStatusUi.ERROR)

    def showFrameProcessed(self): 
        queueframeProcessed = self.getQueueFrames()

        while True:
            try:
                # Obtiene los frames de la cola de visualización
                frames = queueframeProcessed.get(timeout=1)

                self.processFrames(frames)

                # Salir si se presiona 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except queue.Empty:
                print("No frames received, retrying...")
                continue

        cv2.destroyAllWindows()


if __name__ == "__main__":

    # npRet = np.load('DepthParams/param_ret.npy')
    # npK = np.load('DepthParams/param_K.npy')
    # npDist = np.load('DepthParams/param_dist.npy')
    # depthMapProcessor = DepthMapProcessor(npRet,npK,npDist)

    logging.basicConfig(level=logging.NOTSET)

    # DepthmapServerHeadless(depthMapProcessor)
    DepthmapServerHeadless()
    # queueframeProcessed = depthMapServerHeadless.getQueueFrames()

    # while True:
    #     try:
    #         # Obtiene los frames de la cola de visualización
    #         frames = queueframeProcessed.get(timeout=1)

    #         depthMapServerHeadless.updateFrames(frames)

    #         # Salir si se presiona 'q'
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break

    #     except queue.Empty:
    #         print("No frames received, retrying...")
    #         continue

    # cv2.destroyAllWindows()

