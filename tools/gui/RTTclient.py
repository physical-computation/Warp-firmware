import socket
import time
import random
import numpy as np
import serial
import random

import multiprocessing


def writeData(dataList):
    uiCon.write("#".encode('ascii'))  # 0
    # Time
    uiCon.write(str(time.ctime() + ',').encode('ascii'))

    # Ultrasonic data
    uiCon.write((str(dataList[0]) + ",").encode('ascii'))     #   V
    uiCon.write((str(dataList[1]) + ",").encode('ascii'))     #   b
    uiCon.write((str(dataList[2]) + ",").encode('ascii'))     #   g
    uiCon.write((str(dataList[3]) + ",").encode('ascii'))     #   y
    uiCon.write((str(dataList[4]) + ",").encode('ascii'))     #   o
    uiCon.write(str(dataList[5]).encode('ascii'))     #   r
    uiCon.write("\n".encode('ascii'))


HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 19021        # Port to listen on (non-privileged ports are > 1023)

endTag = "Enter selection> "
warpAtMenu = False
progRead = False
sampleLimit = 10
readCount = 0
autoReadEnabled = True

exitProgMode = False

dataReady = False

dataBuffer = [0,0,0,0,0,0]
highNibble = 0
lowNibble = 0
lastSend = ''

uiCon = serial.Serial('COM7', 19200)
# uiCon.open()
tempStringBuffer = ""
recvdData = ""




with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    constatus = s.connect((HOST, PORT))

    while True:
        data = s.recv(1024)
        if not data:
            break
            # conn.sendall(data)
        else :
            # dataReady = True
            # print(data[len(data)- len(endTag):])
            print("# - ", data.decode('ascii'))

            if (data.decode('ascii') == "progExit"):
                exitProgMode = False

            if (data[len(data)- len(endTag):].decode('ascii') == endTag):
                print("End Matched!")
                warpAtMenu = True
                # exitProgMode = False

            if(warpAtMenu and autoReadEnabled):
                print("Entering Prog Mode!")
                s.send('#9'.encode('ascii'))
                lastSend = '#9'
                progRead = True
                warpAtMenu = False
            
            if (progRead and (readCount < sampleLimit)):

                if (data == b'\r\nAS7262:') or (data == b'\r\nAS7262:,'):
                    if (data == b'\r\nAS7262:'):
                        _ = s.recv(1)

                    # dataBuffer = []
                    dataBuffer = [0,0,0,0,0,0]

                    # print("Data Read Begin")

                    recvdData = s.recv(1)
                    while recvdData.decode('ascii') != '\n':
                        tempStringBuffer += recvdData.decode('ascii')
                        recvdData = s.recv(1)

                    print("data = ", tempStringBuffer, " size = ", len(tempStringBuffer))

                    if len(tempStringBuffer) == 35:
                        tempDataList = tempStringBuffer.split(',')
                        
                        dataReady = True

                        for x in range(0,12,2):
                            try:
                                dataBuffer[int(x/2)] = int(tempDataList[x]+tempDataList[x+1],16)
                            except Exception as e:
                                print ("Error", e)
                    
                    s.send('~'.encode('ascii'))
                    lastSend = '~'
                    
                    tempStringBuffer = ""

                if dataReady:
                    readCount += 1
                    print("Sample : ", readCount, ", sending data = ", dataBuffer)
                    try:
                        writeData(dataBuffer)
                    except Exception as e:
                        print ("Error", e)
                    dataReady = False

            # print("readcount  = ", readCount)


            if (readCount >= sampleLimit):
                print("RC LIMIT")
                exitProgMode = True
                autoReadEnabled = False
                progRead = False
                warpAtMenu = False
                readCount = 0

            if exitProgMode and not warpAtMenu and lastSend != '&':
                print("exit mode")
                lastSend = '&'
                s.send('&'.encode('ascii'))


            



