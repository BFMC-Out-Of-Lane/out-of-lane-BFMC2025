# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import time
import queue
from threading import Thread

from src.templates.threadwithstop import ThreadWithStop

class SenderThread(Thread):
    def __init__(self, pipe, logger, debugging):
        super(SenderThread, self).__init__()
        self.pipe = pipe
        self.logger = logger
        self.debugging = debugging
        self.message_queue = queue.Queue()
        self.daemon = True
        self._running = True

    def run(self):
        while self._running:
            try:
                message = self.message_queue.get(timeout=0.1)
                self.pipe.send(message)
            except queue.Empty:
                continue
            except Exception as e:
                if True:
                    self.logger.error(f"Failed to send message: {str(e)}")

    def stop(self):
        self._running = False

class threadGateway(ThreadWithStop):
    """Thread which will handle processGateway functionalities.\n
    Args:
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ===================================== INIT =========================================

    def __init__(self, queueList, logger, debugging):
        super(threadGateway, self).__init__()
        self.logger = logger
        self.debugging = debugging
        self.sendingList = {}
        self.queuesList = queueList
        self.messageApproved = []
        self.sender_threads = {}

    # =================================== SUBSCRIBE ======================================

    def subscribe(self, message):
        """This functin will add the pipe into the approved messages list and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        # Declaration of variables:
        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]
        Pipe = message["To"]["pipe"]
        print(Owner, Id, To, Pipe)
        if not Owner in self.sendingList.keys():
            self.sendingList[Owner] = {}
        if not Id in self.sendingList[Owner].keys():
            self.sendingList[Owner][Id] = {}
        if not To in self.sendingList[Owner][Id].keys():
            self.sendingList[Owner][Id][To] = Pipe
            # Create a sender thread for this pipe
            sender = SenderThread(Pipe, self.logger, self.debugging)
            sender.start()
            self.sender_threads[(Owner, Id, To)] = sender
        self.messageApproved.append((Owner, Id))
        # Debugging( you can comment this):
        if self.debugging:
            self.printList()

    # ================================== UNSUBSCRIBE =====================================

    def unsubscribe(self, message):
        """This functin will remove the pipe into the approved messages list and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]

        # Stop and remove the sender thread
        if (Owner, Id, To) in self.sender_threads:
            self.sender_threads[(Owner, Id, To)].stop()
            del self.sender_threads[(Owner, Id, To)]

        # We delete the value from Dictionary
        del self.sendingList[Owner][Id][To]
        self.messageApproved.remove((Owner, Id))
        if self.debugging:
            self.printList()

    # =================================== SENDING ========================================

    def send(self, message):
        """This functin will send the message on all the pipes that are in the sending list of the message ID.
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        Type = message["msgType"]
        Value = message["msgValue"]
        if (Owner, Id) in self.messageApproved:
            for element in self.sendingList[Owner][Id]:
                if self.debugging:
                    print(self.sendingList[Owner][Id][element])
                try:
                    # Queue the message for sending
                    self.sender_threads[(Owner, Id, element)].message_queue.put(
                        {"Type": Type, "value": Value, "id": Id, "Owner": Owner}
                    )
                except Exception as e:
                    if True:
                        self.logger.error(f"Failed to queue message for {element}: {str(e)}")

    # ====================================================================================

    # Function for debugging:
    def printList(self):
        """Made for debugging"""

        self.logger.warning(self.sendingList)

    # ==================================== RUN ===========================================

    def run(self):
        """This function will take the messages in priority order form the queues.\n
        the prioirty is: Critical > Warning > General
        """

        while self._running:
            message = None
            # We are using "elif" because we are processing one message at a time.
            # We work with the queues in the priority order( We start from the high priority to low priority)
            if not self.queuesList["Critical"].empty():
                message = self.queuesList["Critical"].get()
            elif not self.queuesList["Warning"].empty():
                message = self.queuesList["Warning"].get()
            elif not self.queuesList["General"].empty():
                message = self.queuesList["General"].get()
            if message is not None:
                self.send(message)

            if not self.queuesList["Config"].empty():
                message2 = self.queuesList["Config"].get()
                if str.lower(message2["Subscribe/Unsubscribe"]) == "subscribe":
                    self.subscribe(message2)
                else:
                    self.unsubscribe(message2)

    def stop(self):
        """Stop all sender threads when the gateway stops"""
        for sender in self.sender_threads.values():
            sender.stop()
        super(threadGateway, self).stop()

# =====================================================================================
