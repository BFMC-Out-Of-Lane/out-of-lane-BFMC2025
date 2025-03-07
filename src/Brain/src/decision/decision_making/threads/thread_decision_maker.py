import math
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, LaneKeeping,
                                            SpeedMotor, SteerMotor,
                                            UltrasonicSensors, mainCamera,
                                            serialCamera, trafficSigns)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber
from src.decision.decision_making.odometer import odometer
import time

MIN_DISTANCE_TO_STOP = 200
MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN = 250

# Parking constants
PARKING_STEP1_DISTANCE = 150
PARKING_STEP2_DISTANCE = 150
PARKING_STEP3_DISTANCE = 150
PARKING_MODE_SPEED = 100
MANEUVERING_SPEED = 50
PARKING_SPOT_NUMBER = 1
LENGTH_SPOT = 756
DISTANCE_ENTER_PARKING_ZONE = 100 
RIGHT_LANE_SIDE = 1
ANGLE_PARKING = 230
FORWARD = 1
REVERSE = -1

class threadDecisionMaker(ThreadWithStop):
    """This thread handles decision_making.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging

        self.current_speed = "0"
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.subscribers = {}
        self.odometer = odometer()

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
            self._handle_ultrasonic_message()
            self._handle_lane_keeping_message()
            self._handle_traffic_signs_message()
            self.odometer.update(int(self.current_speed))

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)
        self.subscribers["trafficSigns"] = messageHandlerSubscriber(self.queuesList, trafficSigns, "lastonly", True)

    def _handle_ultrasonic_message(self):
        """Handles the ultrasonic message, i.e.: decides if the vehicle should slow down, stop or continue"""

        ultrasonic_message = self.subscribers["UltrasonicSensors"].receive()
        if ultrasonic_message:
            current_speed = int(self.current_speed)
            front = int(ultrasonic_message["front"])
            rear = int(ultrasonic_message["rear"])
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "CurrentSpeed": {current_speed}, "UltrasonicSensors": {ultrasonic_message}}} }}'
                )

            if current_speed > 0:
                if front < MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send("0")  # stop the vehicle if front distance is less than 10 cm
                elif current_speed > front - MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send(str(front - MIN_DISTANCE_TO_STOP))
            elif current_speed < 0:
                if rear < MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send("0")  # stop the vehicle if front distance is less than 10 cm
                elif abs(current_speed) > rear - MIN_DISTANCE_TO_STOP:
                    self.speed_sender.send(str(-(rear - MIN_DISTANCE_TO_STOP)))

    def _handle_lane_keeping_message(self):
        """Handles the lane keeping message, i.e.: decides the steering angle"""

        lane_keeping = self.subscribers["LaneKeeping"].receive()
        if lane_keeping and int(self.current_speed) > 0:
            self.cross_tracking_error = lane_keeping["cross_tracking_error"]
            self.deviation = lane_keeping["deviation"]
            if self.cross_tracking_error and self.deviation:
                steering = int(self.deviation + math.atan( 1 * self.cross_tracking_error / self.current_speed) * 180 / math.pi) * 10 
                if steering < -230:
                    steering = -230
                elif steering > 230:
                    steering = 230

                self.steer_sender.send(str(steering))

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""
        try:
            traffic_signs_message = self.subscribers["trafficSigns"].receive()
            if traffic_signs_message:
                detected_clss = str(traffic_signs_message["clss"])
                pixels_distance = int(traffic_signs_message["pixels_distance"])
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected": {detected_clss}, "Distance object-car": {pixels_distance}}} }}')

                if detected_clss == "stop":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": {detected_clss}}} }}')
                    return
                    # self.previous_speed = self.current_speed  # Guardar la velocidad antes de detenerse
                    # self.speed_sender.send("0")  # Detener el vehículo
                    # self.logging.info(
                    #     f'{{"threadDecisionMaker": {{ "Object detected - CAR STOPPING": {detected_clss}}} }}')
                    # time.sleep(3)  # Esperar 3 segundos
                    # self.speed_sender.send(self.previous_speed)  # Restaurar la velocidad anterior
                    # self.logging.info(
                    #     f'{{"threadDecisionMaker": {{ "Car resuming speed": {self.previous_speed}}} }}')
                    # return

                if detected_clss == "crosswalk":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": {detected_clss}}} }}')
                    return

                if detected_clss == "highway-end":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "highway-entry":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "one-way":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "parking":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "no-entry":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "roundabout":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return

                if detected_clss == "priority":
                    self.speed_sender.send("0")
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: {detected_clss}}} }}')
                    return
                
                # if detected_clss == "priority" and pixels_distance < MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                #     pass

        except Exception as e:
            print(e)


    def _activate_parking_mode(self):
        flag_parking_mode = True
        parking_status = 0

        """Handles the parking-mode """
        "Status of parking mode:"
        "0-> enter parking zone (when the car see the P signal)=> witch means locate the car next to de place choosen to parking"
        "1-> start parking maneuver (once its located)"
        "2-> finish parking maneuver (stay quiet)"
        "3-> exit parking"
        lane_side = self.enter_parking_zone(self,parking_status)
        self.start_parking_maneuver(self,parking_status,lane_side)
        self.logging.info(f'{{"threadDecisionMaker: im parked, let me rest :| "}}')
        time.sleep(5)
        self.exit_parking_spot(self,parking_status,lane_side)
        flag_parking_mode = False
        self.logging.info(f'{{"threadDecisionMaker: on track again :) " }}')
        "ACTIVAR LANE DETECTOR PORQUE SIGUE SU CAMINO"

        # if parking_status == 0:
        #     lane_side = self.enter_parking_zone(self,parking_status)
        #     parking_status = 1
        # elif parking_status == 1:
        #     self.start_parking_maneuver(self,parking_status,lane_side)
        #     # parking_maneuver(self, lane_side)
        #     parking_status = 2
        # elif parking_status == 2:
        #     self.logging.info(f'{{"threadDecisionMaker: im parked, let me rest :| "}}')
        #     time.sleep(5)
        #     parking_status = 3
        # elif parking_status == 3:
        #     self.exit_parking_spot(self,parking_status,lane_side)
        #     # parking_maneuver(self, lane_side, False)
        #     parking_status = -1
        #     flag_parking_mode = False
        #     self.logging.info(f'{{"threadDecisionMaker: on track again :) " }}')
        #     "ACTIVAR LANE DETECTOR PORQUE SIGUE SU CAMINO"
    

    def move_x_distance(self, distance,speed,direction):
        self.speed_sender.send(str(speed*direction))
        time.sleep(distance/speed)
        self.speed_sender.send("0")

    def enter_parking_zone(self):
        self.logging.info(f'{{"carActions: enter to parking zone"  }}')

        # ubicarse: step1. avanzar desde senda peatonal X + LongParking*EspacioElegido ---> CONSTANTE_STEP_LOCATION"
        distance_to_locate = LENGTH_SPOT*PARKING_SPOT_NUMBER + DISTANCE_ENTER_PARKING_ZONE
        self.move_x_distance(self, distance_to_locate, PARKING_MODE_SPEED, FORWARD)
        
        self.speed_sender.send("0")

        return RIGHT_LANE_SIDE
        
    def start_parking_maneuver(self,lane_side):
        self.logging.info(f'{{"carActions: parking maneuver" }}')

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse en reversa - PARKING_STEP1_DISTANCE
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        self.move_x_distance(self, PARKING_STEP1_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self.speed_sender.send("0")

        # 2. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse revesra --> - PARKING_STEP2_DISTANCE
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        self.move_x_distance(self, PARKING_STEP2_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self.speed_sender.send("0")

        # 3. desplazarse delante ---> PARKING_STEP3_DISTANCE
        self.steer_sender.send("0")
        self.move_x_distance(self, PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, FORWARD)

        self.speed_sender.send("0")
        
    def exit_parking_spot(self,lane_side):
        self.logging.info(f'{{"carActions: im trying to leave the spot" }}')

        # 3b. desplazarse para atras ---> - PARKING_STEP3_DISTANCE"
        self.steer_sender.send("0")
        self.move_x_distance(self, PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self.speed_sender.send("0")

        # 2b. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse adelante --> PARKING_STEP2_DISTANCE"
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        self.move_x_distance(self, PARKING_STEP2_DISTANCE, MANEUVERING_SPEED, FORWARD)

        self.speed_sender.send("0")

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse adelante PARKING_STEP1_DISTANCE"
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        self.move_x_distance(self, PARKING_STEP1_DISTANCE, MANEUVERING_SPEED, FORWARD)
        self.steer_sender.send("0")
        self.speed_sender.send("0")


    def parking_maneuver(self, lane_side, is_starting=True):
        action = "parking maneuver" if is_starting else "exiting the parking spot"
        self.logging.info(f'{{"carActions: {action}" }}')

        if is_starting:
            steps = [
                (ANGLE_PARKING * lane_side, PARKING_STEP1_DISTANCE, REVERSE),
                (ANGLE_PARKING * lane_side * -1, PARKING_STEP2_DISTANCE, REVERSE),
                (0, PARKING_STEP3_DISTANCE, FORWARD)
            ]
        else:
            steps = [
                (0, PARKING_STEP3_DISTANCE, REVERSE),
                (ANGLE_PARKING * lane_side * -1, PARKING_STEP2_DISTANCE, FORWARD),
                (ANGLE_PARKING * lane_side, PARKING_STEP1_DISTANCE, FORWARD)
            ]

        for angle, distance, direction in steps:
            self.steer_sender.send(str(angle))
            self.move_x_distance(self, distance, MANEUVERING_SPEED, direction)
            self.speed_sender.send("0")

        self.steer_sender.send("0")
        self.speed_sender.send("0")
    
    