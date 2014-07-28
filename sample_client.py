#!/usr/bin/python
import struct
import socket
import sys
import math

class RobotTrackerClient:
    # init
    def __init__(self, dst_ip, dst_port):
        self.connect(dst_ip, dst_port);
        
    # create client socket
    def connect(self, dst_ip, dst_port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((dst_ip, dst_port))
        self.sock = sock;
        dprint("Connected")
    
    # close
    def close(self):
        if self.sock: self.sock.close()
        dprint("Closed Connection")
    
    # send data to server
    def send(self, msg):
        self.sock.sendall(msg)
    
    # recieve data from server
    def recv(self):
        MSGLENGTHBYTE = 32;

        data = ""
        while len(data) < MSGLENGTHBYTE:
            incoming = self.sock.recv(32);
            data += incoming

            if not incoming:
                self.end("Connection closed")
                
        if len(data) != MSGLENGTHBYTE:
            self.end("Recieved msg has wrong length:" + str(len(data)) )

        return data
    
    # in case of error or protocol failure
    def end(self, msg=None):
        if msg:
            dprint("End: " + msg)
        self.close()
        exit()
        
    def dprint(self, msg):
        if self.DEBUG:
            print msg
    
    # query for robot with id 
    def query(self, robotid):
    
        robotmsg = None;
    
        try:
            # send robot id
            
            sendmsg = struct.pack('=i', robotid)
            
            self.send(sendmsg)
            
            rsvmsg = self.recv()
            
            robotmsg = struct.unpack('=IifffIIf', rsvmsg)
            
        except Exception, e:
            self.end("Protocol terminated with Error:" + e)
        
        return robotmsg
    
    # Helper: get distance between two robots
    def distanceBetween(self, robotmsgA, robotmsgB):
        if type(robotmsgA) is not tuple or type(robotmsgB) is not tuple \
        or robotmsgA[0] <= 0 or robotmsgB[0] <= 0:
            return 0
            
        return math.hypot(robotmsgA[2]-robotmsgB[2], robotmsgA[3]-robotmsgB[3])
    
    # Helper: get angle from robot A to robot B
    def angleTo(self, robotmsgA, robotmsgB):
        if type(robotmsgA) is not tuple or type(robotmsgB) is not tuple \
        or robotmsgA[0] <= 0 or robotmsgB[0] <= 0:
            return 0
            
        delta_x = robotmsgB[2]-robotmsgA[2]
        delta_y = robotmsgB[3]-robotmsgA[3]
        
        # angle: clockwise from west := 0, east := +-pi
        angle = math.atan2(-delta_y, -delta_x)
        
        return angle
    
# Debug
DEBUG = False;    
def dprint(msg):
    if DEBUG:
        print msg

def main():
    
    # dst_ip dst_port clientcert clientprivkey
    if(len(sys.argv) < 3):
        print("Usage: sample_client.py dst_ip dst_port [robotID]")
        return

    dst_ip = sys.argv[1]
    dst_port = int(sys.argv[2])
    
    if(len(sys.argv) > 3):
        robotid = int(sys.argv[3])
    else:
        robotid = 5

    # open connection
    client = RobotTrackerClient(dst_ip, dst_port)
    
    # ask for specific robot
    robotmsg = client.query(robotid)
    print("Robot Message is" + str(robotmsg))

    # close connection
    client.close()
   
    
if __name__ == "__main__":
    main()
