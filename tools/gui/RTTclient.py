import socket
import time
import random
import numpy as np
import serial
import random



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
warpReady = False
progRead = False
readCounts = 40
i = 0
autoReadEnabled = True
dataReady = False

dataBuffer = [0,0,0,0,0,0]
highNibble = 0
lowNibble = 0


uiCon = serial.Serial('COM7', 19200)
# uiCon.open()
tempStringBuffer = ""
recvdData = ""

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    constatus = s.connect((HOST, PORT))
    # s.listen()
    # conn, addr = s.accept()
    # with conn:
    # print('Connected by', addr)
    # print(constatus)
    while True:
        data = s.recv(1024)
        if not data:
            break
            # conn.sendall(data)
        else :
            # dataReady = True
            # print(data[len(data)- len(endTag):])
            print(data.decode('ascii'))

            if (data[len(data)- len(endTag):].decode('ascii') == endTag):
                print("End Matched!")
                warpReady = True
            
            if(warpReady and autoReadEnabled):
                s.send('#9'.encode('ascii'))
                progRead = True
                # warpReady = False
            
            if (progRead):
                # print("in prog read")

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
                                # pass
                        
                        # print(dataBuffer)
                    
                    tempStringBuffer = ""

                    # highNibble = 0
                    # lowNibble = 0
                    # for x in range(0,6):
                    #     highNibble = s.recv(1024)
                    #     if highNibble:
                    #         highNibble = highNibble[:len(highNibble)-1]
                    #         lowNibble = s.recv(1024)
                    #         if lowNibble:
                    #             lowNibble = lowNibble[:len(lowNibble)-1]
                    #             dataReady = True 
                    #         else:
                    #             lowNibble = "-"
                    #     else:
                    #         highNibble = "-"

                    #     # print("data from ", x, "is :", highNibble+lowNibble)
                    #     try:
                    #         dataBuffer[x] = int(highNibble.decode('ascii')+lowNibble.decode('ascii'),16)
                    #     except Exception as e:
                    #         print ("Error", e)
                    #         # pass

                    # print(dataBuffer)
                    # try:
                    #     writeData(dataBuffer)
                    # except Exception as e:
                    #     print ("Error", e)

                if dataReady:
                    print("Sample : ", i, ", sending data = ", dataBuffer)
                    try:
                        writeData(dataBuffer)
                    except Exception as e:
                        print ("Error", e)
                    dataReady = False
                    i += 1

                # if (data[0] == "!" and data[10] == "\n"):
                # print("Value in reg {reg} is {value}".format(reg=data[1:4], value=data[6:9]))

                if (i>=readCounts):
                    s.send('x'.encode('ascii'))
                    autoReadEnabled = False
                    i = 0
                    # # progRead = False

            # print("#", data)
            
                # dataReady=False

            



