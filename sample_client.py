#!/usr/bin/python
import struct
import socket
import sys

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
    
    # main action
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
        
        self.close()
        return robotmsg
    
# Debug
DEBUG = False;    
def dprint(msg):
    if DEBUG:
        print msg

def main():

    # dst_ip dst_port clientcert clientprivkey
    if(len(sys.argv) < 3):
        dprint("Usage: sample_client.py dst_ip dst_port [robotID]")
        return

    dst_ip = sys.argv[1]
    dst_port = int(sys.argv[2])
    
    if(len(sys.argv) > 3):
        robotid = int(sys.argv[3])
    else:
        robotid = 5

    client = RobotTrackerClient(dst_ip, dst_port)
    robotmsg = client.query(robotid)
    print("Robot Message is" + str(robotmsg))
   
    
main()
