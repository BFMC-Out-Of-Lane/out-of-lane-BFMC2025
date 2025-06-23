#!/usr/bin/env python3
"""
Test script for the periodic sender functionality
"""

import sys
import time
from multiprocessing import Queue, Pipe

# Add the parent directory to the path
sys.path.insert(0, "../../..")

from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.data.TrafficCommunication.threads.threadTrafficCommunicaiton import threadTrafficCommunication
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import Position, ImuData
from src.templates.threadwithstop import ThreadWithStop

class PeriodicSenderThread(ThreadWithStop):
    """Thread that sends messages periodically every 20 seconds"""
    
    def __init__(self, queueList, interval=20, enable_position=True, enable_imu=True):
        super(PeriodicSenderThread, self).__init__()
        self.queueList = queueList
        self.interval = interval
        self.enable_position = enable_position
        self.enable_imu = enable_imu
        
        # Initialize senders only if enabled
        if self.enable_position:
            self.positionSender = messageHandlerSender(self.queueList, Position)
        if self.enable_imu:
            self.imuDataSender = messageHandlerSender(self.queueList, ImuData)
        
    def run(self):
        """Run the periodic sender thread"""
        print(f"Periodic sender started - sending messages every {self.interval} seconds")
        while self._running:
            try:
                # Send periodic messages
                self._send_periodic_messages()
                time.sleep(self.interval)
            except Exception as e:
                print(f"Error in periodic sender: {e}")
                time.sleep(1)
    
    def _send_periodic_messages(self):
        """Send periodic messages every 20 seconds"""
        current_time = time.time()
        timestamp = time.strftime('%H:%M:%S')
        
        if self.enable_position:
            # Example position data - you can customize these values
            position_data = {
                "x": 69.69, # I'm very smart I know
                "y": 420,
                "timestamp": current_time
            }
            self.positionSender.send(position_data)
        
        if self.enable_imu:
            # Example IMU data - you can customize these values
            imu_data = {
                "angle": 180,
            }
            self.imuDataSender.send(imu_data)
            print(f"[{timestamp}] Sent messages")
    
    def set_interval(self, new_interval):
        """Change the sending interval"""
        self.interval = new_interval
        print(f"Periodic sender interval changed to {new_interval} seconds")
    
    def enable_message_type(self, message_type, enabled=True):
        """Enable or disable specific message types"""
        if message_type == "position":
            self.enable_position = enabled
        elif message_type == "imu":
            self.enable_imu = enabled
        print(f"Periodic sender {message_type} messages {'enabled' if enabled else 'disabled'}")


def test_periodic_sender():
    """Test the periodic sender functionality"""
    
    # Create shared memory and queues
    shared_memory = sharedMem()
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    
    # Configuration
    filename = "src/data/TrafficCommunication/useful/publickey_server_test.pem"
    deviceID = 3
    frequency = 0.4
    
    print("Starting Traffic Communication with Periodic Sender...")
    print("Periodic messages will be sent every 20 seconds")
    print("Press Ctrl+C to stop")
    
    # Create and start the traffic communication thread
    traffic_communication = threadTrafficCommunication(
        shared_memory, queueList, deviceID, frequency, filename
    )
    
    try:
        traffic_communication.start()
        
        # Monitor the General queue for messages
        start_time = time.time()
        duration = 120  # Run for 2 minutes to see multiple periodic messages
        
        while time.time() - start_time < duration:
            try:
                message = queueList["General"].get(timeout=1)
                print(f"Received message: {message}")
            except:
                pass
                
    except KeyboardInterrupt:
        print("\nStopping Traffic Communication...")
    finally:
        traffic_communication.stop()
        print("Traffic Communication stopped")

if __name__ == "__main__":
    test_periodic_sender() 