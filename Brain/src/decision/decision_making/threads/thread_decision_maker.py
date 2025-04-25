import math
import time

from src.decision.decision_making.gps.gps import GPS
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, DrivingMode, ImuData,
                                            LaneKeeping, Position, SpeedMotor,
                                            SteerMotor, UltrasonicSensors,
                                            WarningSignal, trafficSigns)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

MIN_DISTANCE_TO_STOP = 200
MIN_DISTANCE_TO_SIGN = 20
MIN_DISTANCE_TO_CROSSWALK = 70
TIME_AT_STOP_SIGN = 2

# Parking constants
PARKING_STEP1_DISTANCE = 390
PARKING_STEP2_DISTANCE = 250
PARKING_STEP3_DISTANCE = 120
PARKING_MODE_SPEED = 100
MANEUVERING_SPEED = 100
PARKING_SPOT_NUMBER = 1
LENGTH_SPOT = 756
LENGHT_CAR = 400
DISTANCE_ENTER_PARKING_ZONE = 1050
DISTANCE_PARKING_SIGNAL_FROM_PARKING = 100
DISTANCE_ENTER_PARKING_ZONE2 = DISTANCE_PARKING_SIGNAL_FROM_PARKING + MIN_DISTANCE_TO_SIGN
RIGHT_LANE_SIDE = 1
ANGLE_PARKING = 230
FORWARD = 1
REVERSE = -1
TIME_BEING_PARKED = 4


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
        self.driving_mode = "stop"
        self.lane_keeping = None
        self.position = None
        self.imu_data = None

        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.signal_sender = messageHandlerSender(self.queuesList, WarningSignal)

        self.subscribers = {}
        self.stop_timer = time.time() - 60
        self.parking_timer = time.time() - 60

        self.gps = GPS(
            path2graphml="/home/ool/out-of-lane-BFMC2025/racetrack/test_track_w_roundabout.graphml", start="1", end="2")

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        while self._running:
            self.driving_mode = self.subscribers["DrivingMode"].receive() or self.driving_mode
            if self.driving_mode == "auto":
                self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
                self.lane_keeping = self.subscribers["LaneKeeping"].receive() or self.lane_keeping
                self.position = self.subscribers["Position"].receive() or self.position
                self.imu_data = self.subscribers["ImuData"].receive() or self.imu_data

                try:
                    # self._handle_lane_keeping_message()
                    # self._handle_ultrasonic_message()
                    self._handle_traffic_signs_message()
                    self._handle_position_message()
                except Exception as e:
                    self.logging.error(f'{{"threadDecisionMaker": {str(e)}}}')

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["UltrasonicSensors"] = messageHandlerSubscriber(
            self.queuesList, UltrasonicSensors, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)
        self.subscribers["trafficSigns"] = messageHandlerSubscriber(self.queuesList, trafficSigns, "lastonly", True)
        self.subscribers["DrivingMode"] = messageHandlerSubscriber(self.queuesList, DrivingMode, "lastonly", True)
        self.subscribers["Position"] = messageHandlerSubscriber(self.queuesList, Position, "lastonly", True)
        self.subscribers["ImuData"] = messageHandlerSubscriber(self.queuesList, ImuData, "lastonly", True)

    # def _handle_ultrasonic_message(self):
    #     """Handles the ultrasonic message, i.e.: decides if the vehicle should slow down, stop or continue"""

    #     ultrasonic_message = self.subscribers["UltrasonicSensors"].receive()
    #     if ultrasonic_message:
    #         current_speed = int(self.current_speed)
    #         front = int(ultrasonic_message["front"])
    #         rear = int(ultrasonic_message["rear"])
    #         if self.debugging:
    #             self.logging.info(
    #                 f'{{"threadDecisionMaker": {{ "CurrentSpeed": {current_speed}, "UltrasonicSensors": {ultrasonic_message}}} }}'
    #             )

    #         if current_speed > 0:
    #             if front > MIN_DISTANCE_TO_STOP:
    #                 self._send_speed(0)  # stop the vehicle if front distance is less than 10 cm
    #             elif current_speed > front - MIN_DISTANCE_TO_STOP:
    #                 self._send_speed(front - MIN_DISTANCE_TO_STOP)
    #         elif current_speed < 0:
    #             if rear > MIN_DISTANCE_TO_STOP:
    #                 self._send_speed(0)  # stop the vehicle if front distance is less than 10 cm
    #             elif abs(current_speed) > rear - MIN_DISTANCE_TO_STOP:
    #                 self._send_speed(-(rear - MIN_DISTANCE_TO_STOP))

    def _handle_lane_keeping_message(self):
        """Handles the lane keeping message, i.e.: decides the steering angle"""

        if self.lane_keeping and int(self.current_speed) > 0:
            self.cross_tracking_error = self.lane_keeping["cross_tracking_error"]
            self.deviation = self.lane_keeping["deviation"]
            if self.cross_tracking_error and self.deviation:
                self.steer_sender.send(str(self._calculate_steering_angle(
                    self.deviation, self.cross_tracking_error, self.current_speed)))

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""

        def timer_stop_signal(t):
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": }} }}')

            self._send_speed(0)
            time.sleep(0.2)
            self._send_speed(0)
            time.sleep(0.2)
            self._send_speed(0)

            time.sleep(t)

        traffic_signs_message = self.subscribers["trafficSigns"].receive()

        if traffic_signs_message is not None:
            detected_object_dict = traffic_signs_message["objectDict"]

            if "stop-sign" in detected_object_dict:
                # self.signal_sender.send(str({"WarningName": "stop", "WarningID": 20}))
                if detected_object_dict["stop-sign"] > MIN_DISTANCE_TO_SIGN + 5 and time.time() - self.stop_timer > 12:
                    self.stop_timer = time.time()
                    self.steer_sender.send(str(0))  # ?
                    timer_stop_signal(TIME_AT_STOP_SIGN)
                    self.steer_sender.send(str(0))  # ?
                    self._send_speed(200)

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "no-entry-sign" in detected_object_dict:
                if detected_object_dict["no-entry-sign"] > MIN_DISTANCE_TO_SIGN + 5 and time.time() - self.stop_timer > 12:
                    # self.signal_sender.send({"WarningName": "forbidden", "WarningID": 5})
                    self._send_speed(0)

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object-Distance": "stop-sign"}} }}')

            if ("crosswalk-sign" in detected_object_dict and detected_object_dict["crosswalk-sign"] > MIN_DISTANCE_TO_SIGN) or ("crosswalk-road" in detected_object_dict and detected_object_dict["crosswalk-road"] > MIN_DISTANCE_TO_CROSSWALK):
                # self.signal_sender.send(str({"WarningName": "crosswalk", "WarningID": 4}))
                self.stop_timer = time.time()
                self._send_speed(100)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "pedestrian" in detected_object_dict and detected_object_dict["pedestrian"] > MIN_DISTANCE_TO_SIGN:
                # self.signal_sender.send(str({"WarningName": "ped_on_crosswalk", "WarningID": 12}))
                self.stop_timer = time.time()
                self._send_speed(0)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "highway-entry-sign" in detected_object_dict and detected_object_dict["highway-entry-sign"] > MIN_DISTANCE_TO_SIGN:
                # self.signal_sender.send(str({"WarningName": "highway_entrance", "WarningID": 6}))
                self._send_speed(400)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "highway-end-sign" in detected_object_dict and detected_object_dict["highway-end-sign"] > MIN_DISTANCE_TO_SIGN:
                # self.signal_sender.send(str({"WarningName": "highway_exit", "WarningID": 7}))
                self._send_speed(200)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "parking-sign" in detected_object_dict and detected_object_dict["parking-sign"] > MIN_DISTANCE_TO_SIGN and time.time() - self.parking_timer > 60:
                # self.signal_sender.send(str({"WarningName": "parking", "WarningID": 10}))
                self.parking_timer = time.time()
                self._send_speed(0)
                self._activate_parking_mode()

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "priority-sign" in detected_object_dict and detected_object_dict["priority-sign"] > MIN_DISTANCE_TO_SIGN:
                # self.signal_sender.send({"WarningName": "priority", "WarningID": 13})
                self._send_speed(200)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

        if int(self.current_speed) < 200 and time.time() - self.stop_timer > 3:
            self._send_speed(200)

    def _handle_position_message(self):
        if self.position is not None and self.imu_data is not None:
            x = float(self.position["x"])
            y = float(self.position["y"])
            current_node = self.gps.current_node(x, y, 150)
            next_node = self.gps.get_next_node()
            if current_node is not None and self.gps.last():
                self.logging.info("LLEGAMOS")
                self._send_speed(0)
                time.sleep(10)
            else:
                if current_node is not None:
                    self.logging.info(
                        f"Current node: {current_node.id}, x: {x}, y: {y}, Node coords: X: {current_node.x}, Y: {current_node.y}, diff in mm, x:{int(current_node.x - x)}, y:{int(current_node.y - y)}")
                    self.logging.info(f"Next node: {next_node}, last: {self.gps.last()}")

                corrected_yaw = float(self.imu_data['yaw']) if float(
                    self.imu_data['yaw']) <= 180 else float(self.imu_data['yaw']) - 360
                deviation = corrected_yaw - math.atan2(next_node.y - y, next_node.x - x) * 180 / math.pi
                self.steer_sender.send(str(self._calculate_steering_angle(deviation, 0, 200)))
                time.sleep(0.2)

    def _calculate_steering_angle(self, deviation, cross_tracking_error, current_speed):
        """
        Uses stanley's method to calculate the steering angle.
        """

        if deviation > 180:
            deviation = deviation - 360
        elif deviation < -180:
            deviation = deviation + 360

        steering = int(deviation + math.atan(0.5 * cross_tracking_error /
                                             int(current_speed)) * 180 / math.pi) * 10
        if steering < -230:
            steering = -230
        elif steering > 230:
            steering = 230

        return steering

    def _activate_parking_mode(self):
        self._send_speed(0)
        lane_side = RIGHT_LANE_SIDE
        self._start_parking_maneuver(lane_side)
        self.logging.info(f'{{"threadDecisionMaker: im parked, let me rest :| "}}')
        time.sleep(TIME_BEING_PARKED)
        self._exit_parking_spot(lane_side)
        self.logging.info(f'{{"threadDecisionMaker: on track again :) " }}')

    def _move_x_distance(self, distance, speed, direction):
        self._send_speed(speed*direction)
        time.sleep(distance/speed)
        self._send_speed(0)
        time.sleep(1)

    def _start_parking_maneuver(self, lane_side):
        self.logging.info(f'{{"carActions: parking maneuver" }}')

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse en reversa - PARKING_STEP1_DISTANCE
        self.logging.info(f'{{"carActions: STEP1" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP1_DISTANCE, MANEUVERING_SPEED, REVERSE)
        self._send_speed(0)

        # 2. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse revesra --> - PARKING_STEP2_DISTANCE
        self.logging.info(f'{{"carActions: STEP2" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP2_DISTANCE, MANEUVERING_SPEED, REVERSE)
        self._send_speed(0)

        # 3. desplazarse delante ---> PARKING_STEP3_DISTANCE
        self.logging.info(f'{{"carActions: STEP3" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, FORWARD)
        self._send_speed(0)

    def _exit_parking_spot(self, lane_side):
        self.logging.info(f'{{"carActions: im trying to leave the spot" }}')

        # 3b. desplazarse para atras ---> - PARKING_STEP3_DISTANCE"
        self.logging.info(f'{{"carActions: STEP3-e" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP3_DISTANCE, MANEUVERING_SPEED, REVERSE)
        self._send_speed(0)

        # 2b. servos al lado opuesto ( - DIRECTION_PARKING y ANGLE_PARKING) y desplazarse adelante --> PARKING_STEP2_DISTANCE"
        self.logging.info(f'{{"carActions: STEP2-e" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side*(-1)))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP2_DISTANCE - 30, MANEUVERING_SPEED, FORWARD)
        self._send_speed(0)

        # 1. servos a DIRECTION_PARKING y ANGLE_PARKING y desplazarse adelante PARKING_STEP1_DISTANCE"
        self.logging.info(f'{{"carActions: STEP1-e" }}')

        time.sleep(1)
        self.steer_sender.send(str(ANGLE_PARKING*lane_side))
        time.sleep(1)

        self._move_x_distance(PARKING_STEP1_DISTANCE - 40, MANEUVERING_SPEED, FORWARD)
        self._send_speed(0)

        time.sleep(1)
        self.steer_sender.send("0")
        time.sleep(1)

    def _turn_until_lane(self, lane, speed):
        self._send_speed(speed)

        while abs(self.lane_keeping["deviation"]) > 50:
            if lane == RIGHT_LANE_SIDE:
                self.steer_sender.send(str(230))
            else:
                self.steer_sender.send(str(-230))

    def _send_speed(self, new_speed):
        if new_speed == 200:
            new_speed = 100

        if self.current_speed != str(new_speed):
            self.speed_sender.send(str(new_speed))
            self.current_speed = str(new_speed)
