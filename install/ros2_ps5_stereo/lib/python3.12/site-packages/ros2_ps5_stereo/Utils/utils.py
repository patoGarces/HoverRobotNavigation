from enum import Enum

class CameraStatusUi(Enum):
    CONNECTING = "Conectando camara"
    CONNECTED = "Camara Conectada"
    WAITING_FW = "Camara esperando fw"
    NOT_FOUND = "Camara no encontrada"
    STREAM_INIT = "Iniciando Stream"
    STREAM_RUNNING = "Stream corriendo"
    ERROR = "Error camara"
