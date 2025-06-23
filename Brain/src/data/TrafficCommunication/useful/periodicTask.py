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
from twisted.internet import task


class periodicTask():
    """Class which will handle the periodic task of sending data to the server

    Args:
        period (int): The period of the task in seconds.
        shrd_mem (sharedMem): A space in memory for mwhere we will get and update data.
        tcp_factory (tcpClient): The tcp client which will send the data to the server.
    """

    def __init__(self, period, shrd_mem, tcp_factory):
        self.period = period
        self.shrd_mem = shrd_mem
        self.tcp_factory = tcp_factory
        self.task = task.LoopingCall(self.periodicCheck)

    def start(self):
        """Start the periodic task"""
        self.task.start(self.period)

    def stop(self):
        """Stop the periodic task"""
        if self.task.running:
            self.task.stop()

    def periodicCheck(self):
        """Checks if there are any errors in the queue and sends them to the server"""
        tosend = self.shrd_mem.get()
        
        if not tosend:
            return
            
        for mem in tosend:
            #print("[PeriodicTask] Enviando al servidor:", mem)
            try:
                self.tcp_factory.send_data_to_server(mem)
                #print("[PeriodicTask] Datos enviados correctamente")
            except Exception as e:
                print("[PeriodicTask] Error al enviar datos:", str(e))
