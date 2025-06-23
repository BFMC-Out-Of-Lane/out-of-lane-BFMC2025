import math
import time

from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.decision.decision_making.gps.gps import GPS
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (CurrentSpeed, DrivingMode, ImuData,
                                            Klem, LaneKeeping, NodeIndex,
                                            PointToFollowMode, Position,
                                            RawConfig, SetNodeList, SpeedMotor,
                                            SteerMotor, TrafficSignDetection,
                                            WarningSignal, Location)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import \
    messageHandlerSubscriber

PATH_TO_GRAPHML = "/home/ool/out-of-lane-BFMC2025/racetrack/competition_track_graph.graphml"
IS_LANE_KEEPING_ON = False  # If it is OFF only node following will work

# Initial position for competition
# INIT_X = 11830  # TODO: Change this to the correct value
# INIT_Y = 2050
# INIT_ANGLE = 0
# MANDATORY_NODES = ["257", "261", "317", "368", "386", "343", "363", "398", "426", "436", "183", "126", "75", "185", "198", "6",
                #    "82", "419", "404", "318", "330", "288", "42", "25", "110", "118", "121", "29", "71", "31", "89", "177", "225", "228", "240"]

# Initial position for testing (COMMENT OUT!)
INIT_X = 15660  # TODO: Change this to the correct value
INIT_Y = 4600
INIT_ANGLE = 90
MANDATORY_NODES = ["55", "243"]


# Traffic sign constants
MIN_DISTANCE_TO_SIGN = 30
MIN_DISTANCE_TO_CAR = 50
MIN_DISTANCE_TO_CROSSWALK = 70
TIME_AT_STOP_SIGN = 2

# Parking constants
PTF_MODE_0 = 0
PTF_MODE_3 = 3
PARKING_MODE_IN = 1
PARKING_MODE_OUT = -1
PARKING_SIDE = 1
PARKING_SPEED = 100
PARKING_INIT_X = 1660  # TODO: Change this to the correct value
PARKING_INIT_Y = 740

# Navigation modes
LANE_DETECTOR = "lane_detector"
NODE_NAVIGATION = "node_navigation"
PARKING = "parking"


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
        self.imu_data = None
        self.klem = "0"
        self.lane_keeping = None
        self.nav_mode = LANE_DETECTOR
        self.point_to_follow_mode = None
        self.position = None

        self.speed_sender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.steer_sender = messageHandlerSender(self.queuesList, SteerMotor)
        self.signal_sender = messageHandlerSender(self.queuesList, WarningSignal)
        self.node_list_sender = messageHandlerSender(self.queuesList, SetNodeList)
        self.raw_config_sender = messageHandlerSender(self.queuesList, RawConfig)

        self.subscribers = {}
        self.stop_timer = time.time() - 60
        self.parking_timer = time.time() - 60
        self.set_position_timer = time.time()
        self.shared_memory = sharedMem()

        self.gps = GPS(
            path2graphml=PATH_TO_GRAPHML,
            mandatory_nodes=MANDATORY_NODES,
            raw_config_sender=self.raw_config_sender,
        )

        self.subscribe()
        super(threadDecisionMaker, self).__init__()

    def run(self):
        send_gps_data = False
        while self._running:
            self.klem = self.subscribers["Klem"].receive() or self.klem
            self.driving_mode = self.subscribers["DrivingMode"].receive() or self.driving_mode

            loc = self.subscribers["Location"].receive()
            if loc:
                print(f'{{"threadDecisionMaker": {{ "Location": {loc} }}}}')

            if self.klem == "0":
                self.current_speed = "0"
                send_gps_data = True

            if send_gps_data and self.klem == "15":
                self.raw_config_sender.send(f"#setAngle:{INIT_ANGLE};;\r\n")
                self.raw_config_sender.send(f"#setPosition:{INIT_X};{INIT_Y};;\r\n")
                self.gps.send_to_nucleo()
                if self.debugging:
                    self.logging.info(f'threadDecisionMaker: GPS sent')

                send_gps_data = False

            if self.klem == "30" and self.driving_mode == "auto":
                self.current_speed = self.subscribers["CurrentSpeed"].receive() or self.current_speed
                self.imu_data = self.subscribers["ImuData"].receive() or self.imu_data
                self.lane_keeping = self.subscribers["LaneKeeping"].receive() or self.lane_keeping
                self.position = self.subscribers["Position"].receive() or self.position

                try:
                    if self.nav_mode == NODE_NAVIGATION or self.nav_mode == LANE_DETECTOR:
                        self._handle_node_index_message()
                        if self.nav_mode == LANE_DETECTOR:
                            self._handle_lane_keeping_message()
                    elif self.nav_mode == PARKING:
                        self._finish_parking_maneuver()

                    self._handle_traffic_signs_message()
                except Exception as e:
                    self.logging.error(f'{{"threadDecisionMaker": {str(e)}}}')

    def subscribe(self):
        """Subscribes to the messages you are interested in"""

        self.subscribers["CurrentSpeed"] = messageHandlerSubscriber(self.queuesList, CurrentSpeed, "lastonly", True)
        self.subscribers["LaneKeeping"] = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastonly", True)
        self.subscribers["trafficSigns"] = messageHandlerSubscriber(
            self.queuesList, TrafficSignDetection, "lastonly", True)
        self.subscribers["DrivingMode"] = messageHandlerSubscriber(self.queuesList, DrivingMode, "lastonly", True)
        self.subscribers["Position"] = messageHandlerSubscriber(self.queuesList, Position, "lastonly", True)
        self.subscribers["ImuData"] = messageHandlerSubscriber(self.queuesList, ImuData, "lastonly", True)
        self.subscribers["Klem"] = messageHandlerSubscriber(self.queuesList, Klem, "lastonly", True)
        self.subscribers["NodeIndex"] = messageHandlerSubscriber(self.queuesList, NodeIndex, "lastonly", True)
        self.subscribers["PointToFollowMode"] = messageHandlerSubscriber(
            self.queuesList, PointToFollowMode, "lastonly", True)
        self.subscribers["Location"] = messageHandlerSubscriber(self.queuesList, Location, "lastonly", True)

    def _handle_lane_keeping_message(self):
        """Handles the lane keeping message, i.e.: decides the steering angle"""

        if self.lane_keeping and int(self.current_speed) > 0:
            self.cross_tracking_error = self.lane_keeping["cross_tracking_error"]
            self.deviation = self.lane_keeping["deviation"]
            if self.cross_tracking_error and self.deviation:
                self.steer_sender.send(str(self._calculate_steering_angle(
                    self.deviation, self.cross_tracking_error, self.current_speed)))
                time.sleep(0.01)  # Allow some time for the steering to adjust

    def _handle_traffic_signs_message(self):
        """Handles the traffic signs, i.e.: decides if the vehicle should slow down, stop or continue"""

        def timer_stop_signal(t):
            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object detected -------------------- CAR STOPPING": }} }}')

            self._send_speed(0)
            time.sleep(0.2)
            self._send_speed(0)

            time.sleep(t)

        traffic_signs_message = self.subscribers["trafficSigns"].receive()

        if traffic_signs_message is not None:
            detected_object_dict = traffic_signs_message["objectDict"]

            if self.debugging:
                self.logging.info(
                    f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict} }}}}')

            if "stop-sign" in detected_object_dict:
                self.signal_sender.send(str({"WarningName": "stop", "WarningID": 20}))
                if detected_object_dict["stop-sign"] < MIN_DISTANCE_TO_SIGN and time.time() - self.stop_timer > 12:
                    self.stop_timer = time.time()
                    timer_stop_signal(TIME_AT_STOP_SIGN)
                    self._send_speed(300)

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "no-entry-sign" in detected_object_dict:
                if detected_object_dict["no-entry-sign"] < MIN_DISTANCE_TO_SIGN and time.time() - self.stop_timer > 12:
                    self.signal_sender.send({"WarningName": "forbidden", "WarningID": 5})
                    self._send_speed(0)

                    if self.debugging:
                        self.logging.info(
                            f'{{"threadDecisionMaker": {{ "Object-Distance": "stop-sign"}} }}')

            if ("crosswalk-sign" in detected_object_dict and detected_object_dict["crosswalk-sign"] < MIN_DISTANCE_TO_SIGN) or ("crosswalk-road" in detected_object_dict and detected_object_dict["crosswalk-road"] < MIN_DISTANCE_TO_CROSSWALK):
                self.signal_sender.send(str({"WarningName": "crosswalk", "WarningID": 4}))
                self.stop_timer = time.time()
                self._send_speed(100)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if (
                ("pedestrian" in detected_object_dict and detected_object_dict["pedestrian"] < MIN_DISTANCE_TO_SIGN) or
                ("roadblock" in detected_object_dict and detected_object_dict["roadblock"] < MIN_DISTANCE_TO_SIGN)
            ):
                if "pedestrian" in detected_object_dict and detected_object_dict["pedestrian"] < MIN_DISTANCE_TO_SIGN:
                    self.signal_sender.send(str({"WarningName": "ped_on_crosswalk", "WarningID": 12}))
                self.stop_timer = time.time()
                self._send_speed(0)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if (("car-white" in detected_object_dict and detected_object_dict["car-white"] < MIN_DISTANCE_TO_CAR) or 
               ("car-blue" in detected_object_dict and detected_object_dict["car-blue"] < MIN_DISTANCE_TO_CAR)):
                self.signal_sender.send(str({"WarningName": "car_ahead", "WarningID": 2}))
                current_index_in_path = self.gps.get_current_node(self.position)
                targets_with_dashed_line = self.gps.get_target(self.gps.G,self.gps.get_node(current_index_in_path).id)
                if targets_with_dashed_line:
                    self._start_skipping_maneuver()
                    if self.debugging:
                        self.logging.info("Skipping car ahead")
                else:
                    self.stop_timer = time.time()
                    self._send_speed(0) 
            
            if "highway-entry-sign" in detected_object_dict and detected_object_dict["highway-entry-sign"] < MIN_DISTANCE_TO_SIGN:
                self.signal_sender.send(str({"WarningName": "highway_entrance", "WarningID": 6}))
                self._send_speed(500)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "highway-end-sign" in detected_object_dict and detected_object_dict["highway-end-sign"] < MIN_DISTANCE_TO_SIGN:
                self.signal_sender.send(str({"WarningName": "highway_exit", "WarningID": 7}))
                self._send_speed(300)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "parking-sign" in detected_object_dict and detected_object_dict["parking-sign"] < MIN_DISTANCE_TO_SIGN and time.time() - self.parking_timer > 60:
                self.signal_sender.send(str({"WarningName": "parking", "WarningID": 10}))

                self._start_parking_maneuver()

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "priority-sign" in detected_object_dict and detected_object_dict["priority-sign"] < MIN_DISTANCE_TO_SIGN:
                self.signal_sender.send({"WarningName": "priority", "WarningID": 13})
                self._send_speed(300)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object-Distance": {detected_object_dict}}} }}')

            if "red-light" in detected_object_dict and time.time() - self.stop_timer > 3:
                self.signal_sender.send({"WarningName": "red-light", "WarningID": 22})
                self.stop_timer = time.time()
                self._send_speed(0)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: red-light" }}}}')

            if "yellow-light" in detected_object_dict and detected_object_dict["yellow-light"] < MIN_DISTANCE_TO_SIGN:
                self.signal_sender.send({"WarningName": "yellow-light", "WarningID": 23})
                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected:yellow-light" }}}}')

            if "green-light" in detected_object_dict and detected_object_dict["green-light"] < MIN_DISTANCE_TO_SIGN:
                self.signal_sender.send({"WarningName": "green-light", "WarningID": 24})
                self._send_speed(300)

                if self.debugging:
                    self.logging.info(
                        f'{{"threadDecisionMaker": {{ "Object detected: green-light" }}}}')

        if int(self.current_speed) < 300 and time.time() - self.stop_timer > 5:
            self._send_speed(300)

    def _handle_node_index_message(self):
        if self.position is not None and self.lane_keeping is not None:
            current_node_index = self.gps.get_current_node(self.position)
            cross_tracking_error = self.lane_keeping["cross_tracking_error"]

            if IS_LANE_KEEPING_ON and time.time() - self.set_position_timer > 0.1:
                self.set_position_timer = time.time()

                if not self.gps.is_last(current_node_index, 2) and cross_tracking_error:
                    angle = self.gps.calc_angle_between_previous_n_next_nodes(current_node_index)
                    if angle == 0 or angle == 180:
                        self.nav_mode = LANE_DETECTOR
                        if abs(cross_tracking_error) < 10:
                            self.raw_config_sender.send(
                            f"#setPosition:0;{int(self.gps.get_node(current_node_index).y)};;\r\n")
                    elif angle == 90 or angle == -90:
                        self.nav_mode = LANE_DETECTOR
                        if abs(cross_tracking_error) < 10:
                            self.raw_config_sender.send(
                            int(f"#setPosition:{int(self.gps.get_node(current_node_index).x)};0;;\r\n"))
                    elif self.nav_mode == LANE_DETECTOR:
                        self._node_navigation(current_node_index)

            elif self.nav_mode == LANE_DETECTOR:
                self._node_navigation(current_node_index)

    def _node_navigation(self, current_node_index):
        self.nav_mode = NODE_NAVIGATION
        self.gps.cut_path_from(current_node_index + 2)
        self.gps.send_to_nucleo()
        self.raw_config_sender.send("#pointToFollow:1;0;0;;\r\n")
        self._send_speed(300)

    def _finish_parking_maneuver(self):
        self.point_to_follow_mode = self.subscribers["PointToFollowMode"].receive()  # Wait for the point to be set
        if self.point_to_follow_mode is not None and self.point_to_follow_mode == PTF_MODE_0:
            self.raw_config_sender.send(
                f"#setParkingMode:{PARKING_MODE_IN};{PARKING_SIDE};{PARKING_SPEED};;\r\n"
            )
            self.driving_mode = "stop"
            self.klem = "0"

    def _calculate_steering_angle(self, deviation, cross_tracking_error, current_speed):
        """
        Uses stanley's method to calculate the steering angle.
        """

        if deviation > 180:
            deviation = deviation - 360
        elif deviation < -180:
            deviation = deviation + 360

        steering = int(deviation + math.atan(2 * cross_tracking_error /
                                             int(current_speed)) * 180 / math.pi) * 10
        if steering < -230:
            steering = -230
        elif steering > 230:
            steering = 230

        return steering

    def _start_parking_maneuver(self):
        if self.debugging:
            self.logging.info("nav_mode: PARKING")

        self.nav_mode = PARKING
        self.raw_config_sender.send(f"#pointToFollow:{PTF_MODE_3};{PARKING_INIT_X};{PARKING_INIT_Y};;\r\n")
        self._send_speed(300)

    def _send_speed(self, new_speed):
        self.speed_sender.send(str(new_speed))
        self.current_speed = str(new_speed)

    def _start_skipping_maneuver(self):
        if self.debugging:
            self.logging.info("maneuver: SKIPPING")
            
        self.raw_config_sender.send(f"#skipCarMode:{1};;\r\n")