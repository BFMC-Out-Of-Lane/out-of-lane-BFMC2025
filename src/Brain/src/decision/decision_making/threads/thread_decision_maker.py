import math
import time

from src.decision.decision_making.odometer.odometer import Odometer
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, LaneKeeping,
                                            SpeedMotor, SteerMotor,
                                            UltrasonicSensors, WarningSignal,
                                            mainCamera, serialCamera, DrivingMode,
                                            trafficSigns)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_TO_STOP = 200
MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN = 30
TIME_AT_STOP_SIGN = 3

# Parking constants
PARKING_STEP1_DISTANCE = 600
PARKING_STEP2_DISTANCE = 200
PARKING_STEP3_DISTANCE = 100
PARKING_MODE_SPEED = 100
MANEUVERING_SPEED = 50
PARKING_SPOT_NUMBER = 1
LENGTH_SPOT = 756
DISTANCE_ENTER_PARKING_ZONE = 830
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

        self.onHighwayLane = False

        self.current_speed = "0"
        self.driving_mode = "stop"
        
        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.signal_sender = messageHandlerSender(self.queuesList, WarningSignal)

        self.subscribers = {}
        self.odometer = Odometer()
        self.stop_signal_timer = time.time()

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.driving_mode = self.subscribers["DrivingMode"].receive() or self.driving_mode
            if self.driving_mode == "auto":
                self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
                # self._handle_ultrasonic_message()
                try:
                    self._handle_lane_keeping_message()
                    self._handle_traffic_signs_message()
                except Exception as e:
                    print(e)

            self.odometer.update(int(self.current_speed))

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(self.queuesList, UltrasonicSensors, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)
        self.subscribers["trafficSigns"] = messageHandlerSubscriber(self.queuesList, trafficSigns, "lastonly", True)
        self.subscribers["DrivingMode"] = messageHandlerSubscriber(self.queuesList, DrivingMode, "lastonly", True)

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
                if front > MIN_DISTANCE_TO_STOP:
                    self._send_speed(0)  # stop the vehicle if front distance is less than 10 cm
                elif current_speed > front - MIN_DISTANCE_TO_STOP:
                    self._send_speed(front - MIN_DISTANCE_TO_STOP)
            elif current_speed < 0:
                if rear > MIN_DISTANCE_TO_STOP:
                    self._send_speed(0)  # stop the vehicle if front distance is less than 10 cm
                elif abs(current_speed) > rear - MIN_DISTANCE_TO_STOP:
                    self.speed_send(-(rear - MIN_DISTANCE_TO_STOP))

    def _handle_lane_keeping_message(self):
        """Handles the lane keeping message, i.e.: decides the steering angle"""

        lane_keeping = self.subscribers["LaneKeeping"].receive()
        if lane_keeping and int(self.current_speed) > 0:
            self.cross_tracking_error = lane_keeping["cross_tracking_error"]
            self.deviation = lane_keeping["deviation"]
            if self.cross_tracking_error and self.deviation:
                steering = int(self.deviation + math.atan(1 * self.cross_tracking_error /
                               self.current_speed) * 180 / math.pi) * 10
                if steering < -230:
                    steering = -230
                elif steering > 230:
                    steering = 230

                self.steer_sender.send(str(steering))

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""

        def timer_stop_signal(t):
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": }} }}')
            self._send_speed(0)
            time.sleep(t)

        traffic_signs_message = self.subscribers["trafficSigns"].receive()
        
        if traffic_signs_message is not None:
            detected_object_dict = traffic_signs_message["objectDict"]

            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "stop-sign" in detected_object_dict:
                # self.signal_sender.send(str({"WarningName": "stop", "WarningID": 20}))
                if detected_object_dict["stop-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN and time.time() - self.stop_signal_timer > 3:
                    timer_stop_signal(TIME_AT_STOP_SIGN)
                    self.stop_signal_timer = time.time()
                self.parking_sign_first_detection = True

            if "crosswalk-sign" in detected_object_dict or "crosswalk-road" in detected_object_dict:
                if detected_object_dict["crosswalk-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self._send_speed(100)
                    # self.signal_sender.send({"WarningName": "crosswalk", "WarningID": 4})

            if "highway-end-sign" in detected_object_dict:
                if detected_object_dict["highway-end-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self._send_speed(200)
                    print("HIGHWAY EXIT")
                    # self.signal_sender.send({"WarningName": "highway_exit", "WarningID": 7})

            if "highway-entry-sign" in detected_object_dict:
                if detected_object_dict["highway-entry-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self._send_speed(400)
                    print("HIGHWAY ENTRANCE")
                    # self.signal_sender.send({"WarningName": "highway_entrance", "WarningID": 6})

            if "one-way-sign" in detected_object_dict:
                # self.signal_sender.send({"WarningName": "oneway", "WarningID": 9})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: one-way-sign" }}}}')

            if "parking-sign" in detected_object_dict:
                if detected_object_dict["parking-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN and time.time() - self.stop_signal_timer > 3:
                    self.stop_signal_timer = time.time()
                    self._send_speed(0)
                    self._activate_parking_mode()
                    # self.signal_sender.send({"WarningName": "parking", "WarningID": 10})
                    self.parking_sign_first_detection = False

            if "no-entry-sign" in detected_object_dict:
                if detected_object_dict["no-entry-sign"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self._send_speed(0)
                    # TODO: Add path plannign logic
                    # self.signal_sender.send({"WarningName": "forbidden", "WarningID": 5})

            if "roundabout-sign" in detected_object_dict:
                # self.signal_sender.send({"WarningName": "roundabaout", "WarningID": 15})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": "Object detected: roundabout-sign" }}')

            if "priority-sign" in detected_object_dict:
                # self.signal_sender.send({"WarningName": "priority", "WarningID": 13})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": "Object detected: priority-signal }}')

            if "pedestrian" in detected_object_dict:
                if detected_object_dict["pedestrian"] > MIN_DISTANCE_TO_STOP_TRAFFIC_SIGN:
                    self._send_speed(0)
                    # self.signal_sender.send({"WarningName": "ped_on_crosswalk", "WarningID": 12})
                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": "Object detected: pedestrian }}')

            if "red-light" in detected_object_dict and time.time() - self.stop_signal_timer > 3:
                self.stop_signal_timer = time.time()
                self._send_speed(0)
                # self.signal_sender.send({"WarningName": "red-light", "WarningID": 22})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: red-light" }}}}')

            if "yellow-light" in detected_object_dict:
                # self.signal_sender.send({"WarningName": "yellow-light", "WarningID": 23})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected:yellow-light" }}}}')

            if "green-light" in detected_object_dict:
                # self.signal_sender.send({"WarningName": "green-light", "WarningID": 24})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: green-light" }}}}')
                    
            return
     
        if self.current_speed == "0":
            self.speed_sender.send(200)
            
    def _activate_parking_mode(self):
        parking_status = 0

        """Handles the parking-mode """
        "Status of parking mode:"
        "0-> enter parking zone (when the car see the P signal)=> witch means locate the car next to de place choosen to parking"
        "1-> start parking maneuver (once its located)"
        "2-> finish parking maneuver (stay quiet)"
        "3-> exit parking"

        self.move_x_distance(1000, 100, FORWARD)
        self._send_speed(0)

        # lane_side = self.enter_parking_zone()
        # lane_side = 1
        # self.start_parking_maneuver(lane_side)
        # self.logging.info(f'{{"threadDecisionMaker: im parked, let me rest :| "}}')
        # time.sleep(3)
        # self.exit_parking_spot(lane_side)
        # self.logging.info(f'{{"threadDecisionMaker: on track again :) " }}')
        # "ACTIVAR LANE DETECTOR PORQUE SIGUE SU CAMINO"

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
    

    def move_x_distance(self, distance, speed, direction):
        self.speed_send(speed*direction)
        print(distance/speed)
        time.sleep(distance/speed)
        self._send_speed(0)

    def enter_parking_zone(self):
        self.logging.info(f'{{"carActions: enter to parking zone"  }}')

        # ubicarse: step1. avanzar desde senda peatonal X + LongParking*EspacioElegido ---> CONSTANTE_STEP_LOCATION"
        distance_to_locate = LENGTH_SPOT*PARKING_SPOT_NUMBER + DISTANCE_ENTER_PARKING_ZONE
        self.move_x_distance(distance_to_locate, PARKING_MODE_SPEED, FORWARD)
        
        self._send_speed(0)

        return RIGHT_LANE_SIDE
        
    def start_parking_maneuver(self,lane_side):
        self.logging.info(f'{{"carActions: parking maneuver" }}')

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse en reversa - PARKING_STEP1_DISTANCE
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        self.move_x_distance(PARKING_STEP1_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self._send_speed(0)

        # 2. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse revesra --> - PARKING_STEP2_DISTANCE
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        self.move_x_distance(PARKING_STEP2_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self._send_speed(0)

        # 3. desplazarse delante ---> PARKING_STEP3_DISTANCE
        self.steer_sender.send("0")
        self.move_x_distance(PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, FORWARD)

        self._send_speed(0)
        
    def exit_parking_spot(self,lane_side):
        self.logging.info(f'{{"carActions: im trying to leave the spot" }}')

        # 3b. desplazarse para atras ---> - PARKING_STEP3_DISTANCE"
        self.steer_sender.send("0")
        self.move_x_distance(PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, REVERSE)

        self._send_speed(0)

        # 2b. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse adelante --> PARKING_STEP2_DISTANCE"
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        self.move_x_distance(PARKING_STEP2_DISTANCE, MANEUVERING_SPEED, FORWARD)

        self._send_speed(0)

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse adelante PARKING_STEP1_DISTANCE"
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        self.move_x_distance(PARKING_STEP1_DISTANCE, MANEUVERING_SPEED, FORWARD)
        self.steer_sender.send("0")
        self._send_speed(0)


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
            self.move_x_distance(distance, MANEUVERING_SPEED, direction)
            self._send_speed(0)

        self.steer_sender.send("0")
        self._send_speed(0)
    
    def _send_speed(self, new_speed):
        self.speed_sender.send(str(new_speed))
        self.current_speed = str(new_speed)