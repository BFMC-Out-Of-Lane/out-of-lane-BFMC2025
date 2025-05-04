import base64
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from src.hardware.camera.lane_detection.lane_detector import LaneDetector
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (Brightness, Contrast, LaneKeeping,
                                            Record, Recording, mainCamera,
                                            serialCamera, trafficSigns)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber

class threadCamera(ThreadWithStop):
    """Thread que maneja las funcionalidades de la cámara.
    
    Args:
        queuesList (dict): Diccionario de colas de mensajes.
        logger (logging object): Logger para depuración.
        debugger (bool): Bandera de depuración.
    """
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 5
        self.recording = False

        self.video_writer = None
        self.lane_detector = LaneDetector(min_offset=180, distance_to_aim=200, is_simu=False)

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.lane_keeping_sender = messageHandlerSender(self.queuesList, LaneKeeping)

        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()    

    def subscribe(self):
        """Se subscriben los mensajes requeridos."""
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.trafficSubscriber = messageHandlerSubscriber(self.queuesList, trafficSigns, "lastonly", True)

    def Queue_Sending(self):
        """Envía periódicamente el estado de grabación."""
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    def stop(self):
        if self.recording and self.video_writer is not None:
            self.video_writer.release()

        super(threadCamera, self).stop()

    def Configs(self):
        """Callback para recibir configuraciones desde la pipe."""
        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))

            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Brightness": max(0.0, min(1.0, float(message))),
                }
            )

        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Contrast": max(0.0, min(32.0, float(message))),
                }
            )

        threading.Timer(1, self.Configs).start()

    def run(self):
        """Captura imágenes, procesa y envía datos al gateway de procesos."""
        send = True
        while self._running:
            try:
                recordRecv = self.recordSubscriber.receive()
                if recordRecv is not None:
                    self.recording = bool(recordRecv)
                    if not self.recording and self.video_writer is not None:
                        self.video_writer.release()
                    elif self.recording:
                        fourcc = cv2.VideoWriter_fourcc(*"XVID")
                        # You can choose different codecs, e.g., 'MJPG', 'XVID', 'H264', etc.
                        # filename = "output_video_" + str(time.time()) + ".avi"
                        # self.video_writer = cv2.VideoWriter(filename, fourcc, self.frame_rate, (1920, 1080))
                        
                        self.video_writer = cv2.VideoWriter(
                            "output_video" + str(time.time()) + ".avi",
                            fourcc,
                            self.frame_rate,
                            (1280, 720),
                        )

            except Exception as e:
                print(e)

            if send:
                mainRequest = self.camera_capture("main")  # Main camera capture
                serialRequest = self.camera_capture("lores")  # Low-resolution camera capture
                
                if self.recording == True:
                    self.video_writer.write(mainRequest)

                if len(serialRequest.shape) == 2 or serialRequest.shape[2] == 1:
                    serialRequest = cv2.cvtColor(serialRequest, cv2.COLOR_YUV2BGR_I420) # Converts YUV format to BGR
                    
                # Crear una versión de baja resolución (lores) redimensionando la imagen
                serialRequest = serialRequest[:, :320]  # Crops the image to the lores resolution

                
                # Codificar la imagen principal
                _, mainEncodedImg = cv2.imencode(".jpg", mainRequest)
                mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
                self.mainCameraSender.send(mainEncodedImageData)

                # mainEncodedImageData = encode_image(mainRequest)
                # self.mainCameraSender.send(mainEncodedImageData)

                traffic_sign = self.trafficSubscriber.receive()
                processed_image, cross_tracking_error, deviation = self.lane_detector.process_frame(serialRequest, traffic_sign)
                self.lane_keeping_sender.send({"cross_tracking_error": cross_tracking_error, "deviation": deviation})
                
                # Codificar la imagen procesada para el envío
                _, serial_encoded_processed_image = cv2.imencode(".jpg", processed_image)
                serial_encoded_processed_image_data = base64.b64encode(serial_encoded_processed_image).decode("utf-8")
                self.serialCameraSender.send(serial_encoded_processed_image_data)

                # serialEncodedImageData = encode_image(serialRequest)
                # self.serialCameraSender.send(serialEncodedImageData)

            send = not send

    def camera_capture(self, stream_name):
        """Captures images for 'main' and 'lores'."""

        frames = self.camera.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())

        # Convertir el espacio de color de RGB a BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if stream_name == "lores":
            frame = cv2.resize(frame, (304, 171), interpolation=cv2.INTER_LINEAR)

        return frame
    
    def start(self):
        super(threadCamera, self).start()

    def _init_camera(self):
        """Inicializa el pipeline de la cámara Intel RealSense D435."""
        ctx = rs.context()
        if len(ctx.query_devices()) == 0:
            raise RuntimeError("No se detectó ninguna cámara RealSense conectada.")

        self.camera = rs.pipeline()
        config = rs.config()
        # Solo un stream de color a 1920×1080 @30 fps
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 15)
        # config.enable_stream(rs.stream.depth,  640,  480, rs.format.z16,  30)

        # try:
        #     # Resuelvo la configuración contra el pipeline
        #     profile = config.resolve(self.camera)
        # except Exception as e:
        #     self.logger.error(f"Error al resolver streams: {e}")
        #     raise

        # self.camera.start(profile)
        self.camera.start(config)