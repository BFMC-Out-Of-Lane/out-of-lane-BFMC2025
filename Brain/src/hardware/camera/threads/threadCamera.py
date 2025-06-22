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
                                            serialCamera, TrafficSignPosition, secondaryCamera, TrafficSignDetection)
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
        self.depth_scale = 0.001  # Default depth scale

        self.video_writer = None
        self.lane_detector = LaneDetector(min_offset=60, perc_of_sight_len=0.3)

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.secondaryCameraSender = messageHandlerSender(self.queuesList, secondaryCamera)
        self.lane_keeping_sender = messageHandlerSender(self.queuesList, LaneKeeping)
        self.signsSender = messageHandlerSender(self.queuesList, TrafficSignDetection)
        
        self.distanceToObjectMessage = {}

        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()    

    def subscribe(self):
        """Se subscriben los mensajes requeridos."""
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)
        self.trafficSignPositionSubscriber = messageHandlerSubscriber(self.queuesList, TrafficSignPosition, "lastonly", True)

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

    def distance_depth(self, point_x, point_y, depth_frame):
        
        # If no depth frame is available, continue to the next iteration.
        if not depth_frame:        
            return

            # Convert the depth frame to a NumPy array for easier processing.
            # The data is typically in raw units, so we multiply by the depth_scale to get meters.
        depth_image_data = np.asanyarray(depth_frame.get_data()) * self.depth_scale

            # Initialize variables for comprehensive depth statistics
        valid_depth_pixels = 0
        total_distance = 0.0

        # Define the points around the center point for which we want to measure distance
        # This is a list of points around the center point (point_x, point_y) to measure distance.
        points = [(point_x, point_y),
                (point_x+1, point_y),
                (point_x, point_y+1),
                (point_x+1, point_y+1),
                (point_x-1, point_y),
                (point_x, point_y-1),
                (point_x-1, point_y-1),
                (point_x+1, point_y-1),
                (point_x-1, point_y+1),
                ]  # List of 9 points around the center point
        for (x, y) in points:
        # Verificar si el punto está dentro de los límites de la imagen
            if 0 <= x < 1280 and 0 <= y < 720:  # Assuming the depth frame is 1280x720        
                dist = depth_image_data[y, x]   # Obtener la distancia del píxel actual en metros
                # print(f"Distancia medida en píxel ({x}, {y}): {dist:.2f} metros")

                # Verificar si la distancia es válida (mayor que 0)
                if dist > 0.001:  # Usar un umbral pequeño para filtrar ruido
                    valid_depth_pixels += 1
                    total_distance += dist

            # Calculate average distance if there are valid pixels.
        average_distance = total_distance / valid_depth_pixels if valid_depth_pixels > 0 else 0.0

        return average_distance  # Return the average distance in meters for the center point

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
                depthFrame = self.camera_capture_depth()  # Depth frame capture
                
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

                traffic_sign_pos = self.trafficSignPositionSubscriber.receive()
                # print(f"traffic_sign_pos recibido: {traffic_sign_pos}") 
                traffic_sign_dist = {}
                
                if traffic_sign_pos is not None and "objectDict" in traffic_sign_pos:
                    for class_name, center in traffic_sign_pos["objectDict"].items():
                        center_x = int(center[0])                        
                        center_y = int(center[1])
                        if 0 <= center_x < 1280 and 0 <= center_y < 720:
                            distance_meters = self.distance_depth(center_x, center_y, depthFrame)
                            distance_centimeters = distance_meters * 100  # Convert to centimeters
                            traffic_sign_dist[class_name] = int(distance_centimeters)
                        else:
                            traffic_sign_dist[class_name] = None
                else:
                    traffic_sign_dist = None

                if traffic_sign_dist is not None:    
                    self.distanceToObjectMessage = {
                        "objectDict": traffic_sign_dist,
                    }
                    self.signsSender.send(self.distanceToObjectMessage)
                
                processed_image, cross_tracking_error, deviation, binary_warped = self.lane_detector.process_frame(serialRequest, traffic_sign_dist)
                self.lane_keeping_sender.send({"cross_tracking_error": cross_tracking_error, "deviation": deviation})
                


                # Codificar la imagen procesada para el envío
                _, serial_encoded_processed_image = cv2.imencode(".jpg", processed_image)
                serial_encoded_processed_image_data = base64.b64encode(serial_encoded_processed_image).decode("utf-8")
                self.serialCameraSender.send(serial_encoded_processed_image_data)
                
                # Encode binary warped image before sending
                _, binary_warped_encoded = cv2.imencode(".jpg", binary_warped)
                binary_warped_encoded_data = base64.b64encode(binary_warped_encoded).decode("utf-8")
                self.secondaryCameraSender.send(binary_warped_encoded_data)

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

    def camera_capture_depth(self):
        frames = self.camera.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            print("No depth frame captured!!!!!!!")
            return None
        return depth_frame
    
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
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        self.camera.start(config)
