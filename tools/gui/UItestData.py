import time
import random
import numpy as np
import serial
import random
import time

def testFunction():
    uiCon.write("#".encode('ascii'))  # 0
    # Time
    uiCon.write(str(time.ctime() + ',').encode('ascii'))

    # Ultrasonic data
    uiCon.write((str(random.randint(0 ,65536)) + ",").encode('ascii'))     #   V
    uiCon.write((str(random.randint(0 ,65536)) + ",").encode('ascii'))     #   b
    uiCon.write((str(random.randint(0 ,65536)) + ",").encode('ascii'))     #   g
    uiCon.write((str(random.randint(0 ,65536)) + ",").encode('ascii'))     #   y
    uiCon.write((str(random.randint(0 ,65536)) + ",").encode('ascii'))     #   o
    uiCon.write(str(random.randint(0 ,65536)).encode('ascii'))     #   r
    uiCon.write("\n".encode('ascii'))


try:
    uiCon = serial.Serial('COM7', 19200)
    uiCon.open()
    dummyData = True
except Exception as e:
    print(str(e))


while True:
    try:
        testFunction()
        time.sleep(0.25)
    except Exception as e:
        print (str(e))
