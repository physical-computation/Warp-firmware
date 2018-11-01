import socket
import time
import random
import numpy as np
import serial
import random

import logging
import multiprocessing as mp
from queue import Queue
import traceback



class WarpConnectorClass(object):
    #TODO: Need to impliment exception handling at each socket use
    dataBuffer = [0,0,0,0,0,0]
    exitProgMode = False

    lastSend = ''
    tempStringBuffer = ""
    recvdData = ""

    sampleLimit = 10
    readCount = 0
    autoReadEnabled = True
    endTag = "Enter selection> "

    progRead = False
    warpConnected = False
    logger = mp.get_logger()


    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 19021        # Port to listen on (non-privileged ports are > 1023)


    def __init__(self): #thread initialisation
        self.warpSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conectionType = "serial"
        self._running = True

    def terminate(self):
        self.logger.debug("Termination Handle called")
        self._running = False

    def run(self,warpAtMenu,dataQueue, dataReady, uInputQueue):

        while not self.warpConnected:
            try :
                warpConnectionHandle = self.warpSocket.connect((self.HOST, self.PORT))
                self.warpConnected = True
            except Exception as e:
                self.warpConnected = False
                self.logger.error(e)
                self.logger.debug(traceback.format_exc())
                time.sleep(2)

        uInput = ""

        while self._running:
            # self.logger.error("# - waiting for data")
            data = self.warpSocket.recv(1024)
            if not data:
                break
                # conn.sendall(data)
            else :
                # dataReady.set()
                # print(data[len(data)- len(self.endTag):])
                print("# (", len(data.decode('ascii')), ") -", data.decode('ascii'))
                # self.logger.error("# - %s", data.decode('ascii'))

                # print("test = ", data.decode('ascii')[1:8])
                if (data.decode('ascii')[0:8] == "progExit"):
                    self.logger.debug("progExit match!")
                    self.exitProgMode = False

                if (data[len(data)- len(self.endTag):].decode('ascii') == self.endTag):
                    self.logger.debug("End matched! -> Menu load completed!")
                    warpAtMenu.set()
                    # self.exitProgMode = False

                if(warpAtMenu.is_set()):
                    # self.exitProgMode = False
                    self.logger.debug("Waiting for user input")
                    uInput = uInputQueue.get()
                    self.logger.debug("User input = %s", uInput)

                    if uInput == "#":
                        warpAtMenu.clear()
                        self.autoReadEnabled = True
                        self.logger.debug("Entering Programatic read mode!")
                        self.warpSocket.send('#9'.encode('ascii'))
                        self.lastSend = '#9'
                        self.progRead = True
                    else: 
                        warpAtMenu.clear()
                        self.warpSocket.send(uInput.encode('ascii'))
                        self.lastSend = uInput
                
                if (self.progRead and (self.readCount < self.sampleLimit)):

                    if (data == b'\r\nAS7262:') or (data == b'\r\nAS7262:,'):
                        if (data == b'\r\nAS7262:'):
                            _ = self.warpSocket.recv(1)

                        # dataBuffer = []
                        self.dataBuffer = [0,0,0,0,0,0]

                        # print("Data Read Begin")

                        self.recvdData = self.warpSocket.recv(1)
                        while self.recvdData.decode('ascii') != '\n':
                            self.tempStringBuffer += self.recvdData.decode('ascii')
                            self.recvdData = self.warpSocket.recv(1)

                        print("data = ", self.tempStringBuffer, " size = ", len(self.tempStringBuffer))

                        if len(self.tempStringBuffer) == 35:
                            tempDataList = self.tempStringBuffer.split(',')

                            for x in range(0,12,2):
                                try:
                                    self.dataBuffer[int(x/2)] = int(tempDataList[x]+tempDataList[x+1],16)
                                    dataReady.set()

                                except Exception as e:
                                    self.logger.error("Error - %s", e)
                        
                        self.warpSocket.send('~'.encode('ascii'))
                        self.lastSend = '~'
                        
                        self.tempStringBuffer = ""

                    if dataReady.is_set():
                        self.readCount += 1
                        self.logger.info("Sample : %d - sending data = %s", self.readCount, self.dataBuffer)
                        try:
                            dataQueue.put(self.dataBuffer)
                            # writeData(self.dataBuffer) ######
                        except Exception as e:
                            self.logger.error("Error - %s", e)
                        dataReady.clear()

                # print("self.readCount  = ", self.readCount)

                if (self.readCount >= self.sampleLimit):
                    self.logger.warn("Read limit reached")
                    self.exitProgMode = True
                    self.autoReadEnabled = False
                    self.progRead = False
                    warpAtMenu.clear()
                    self.readCount = 0

                if self.exitProgMode and not warpAtMenu.is_set() and self.lastSend != '&':
                    self.logger.debug("Exiting programatic read mode")
                    self.lastSend = '&'
                    self.warpSocket.send('&'.encode('ascii'))

        self.warpConnected = False
        self.warpSocket.shutdown()
        self.warpSocket.close()



class UIConnectorClass(object):
    def __init__(self): #thread initialisation
        self._running = True

    def terminate(self):
        self._running = False

    def run(self, dataQueue, dataReady):
        # uiCon = serial.Serial('COM7', 19200)
        logger = mp.get_logger()
        
        while self._running:
            try:
                d = dataQueue.get()
                logger.warn("Data Recieved = %s", d)

            except Exception as e:
                logger.error("Error - %s", e)


    def writeData(self, dataList):
        self.uiCon.write("#".encode('ascii'))  # 0
        # Time
        self.uiCon.write(str(time.ctime() + ',').encode('ascii'))

        # Ultrasonic data
        self.uiCon.write((str(dataList[0]) + ",").encode('ascii'))     #   V
        self.uiCon.write((str(dataList[1]) + ",").encode('ascii'))     #   b
        self.uiCon.write((str(dataList[2]) + ",").encode('ascii'))     #   g
        self.uiCon.write((str(dataList[3]) + ",").encode('ascii'))     #   y
        self.uiCon.write((str(dataList[4]) + ",").encode('ascii'))     #   o
        self.uiCon.write(str(dataList[5]).encode('ascii'))             #   r
        self.uiCon.write("\n".encode('ascii'))


if __name__ == "__main__":

    # warpAtMenu = False
    # self.progRead = False
    terminateFlag = False
    
    mpManager = mp.Manager()
    mpLock = mp.Lock()

    processList = []

    mp.log_to_stderr()
    logger = mp.get_logger()
    logger.setLevel(logging.DEBUG)

    warpAtMenu = mp.Event()
    warpDataReady = mp.Event()
    
    warpAtMenu.clear()

    # userInput = mp.sharedctypes.Array('c',b'',lock=mpLock)
    dataQueueToUI = mp.Queue()
    uInputQueue = mp.Queue()

    warpConnectorInstance = WarpConnectorClass()
    uiConnectorInstance = UIConnectorClass() 


    warpConnectorProcess = mp.Process(target = warpConnectorInstance.run, args=(warpAtMenu,dataQueueToUI, warpDataReady, uInputQueue),name="Warp")
    
    uiConnectorProcess = mp.Process(target = uiConnectorInstance.run, args=(dataQueueToUI, warpDataReady),name="UI")

    uiConnectorProcess.start()
    processList.append(uiConnectorProcess)

    warpConnectorProcess.start()
    processList.append(warpConnectorProcess)

    while not terminateFlag:
        uInput = input()
        if uInput == "&":
            terminateFlag = True
        else:
            uInputQueue.put(uInput)

    # time.sleep(15)

    for p in processList:
        p.terminate()

    for p in processList:
        p.join()


