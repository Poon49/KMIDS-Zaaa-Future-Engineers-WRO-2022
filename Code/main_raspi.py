import time
import serial
import RPi.GPIO as GPIO
import cv2 as cv
import numpy as np
import time
import threading
from bno import BNO055
import wiringpi

from digitalio import DigitalInOut
import board
import busio
from adafruit_vl53l0x import VL53L0X

GPIO.setmode(GPIO.BCM)

Button = 17
GPIO.setup(Button,GPIO.IN)

servoPin = 13
wiringpi.wiringPiSetupGpio()
wiringpi.pinMode(servoPin, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)
maxServo = 170
midServo = 135
minServo = 100

wiringpi.pwmWrite(servoPin, midServo)

LastError = 0

LastTime = time.time()*1000

imu_x = int(0)



GPIO.setup(12,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
f=GPIO.PWM(12,100)
b=GPIO.PWM(19,100)
f.start(0) 
b.start(0)
speedMotor = 100
s = 50

greenDetect = 0
redDetect = 0
turning = 0
clockwise = 0
front_wall_detect = 0
wall_error = 0

LastErrorImu = 0
LastTimeImu = 0
LastErrorWall = 0
LastTimeWall = 0

steps = 1
laps = 0
counterDegree = 0
first_read = True
lastSteering = 0
distanceCenter = 8000
distanceRight = 8000
distanceLeft = 8000
lastSeeRed = False
lastSeeGreen = False
timeValueGreen = time.time()*1000
timeValueRed = time.time()*1000

GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.OUT)
GPIO.output(4, 1)

def motorDrive(speeds):
    global s
    if s > speeds:
        s -= 5
    elif s < speeds:
        s += 5
    if abs(s-speeds)<10:
        s = speeds
    if s>100:
        s = 100
    elif s<0:
        s = 0
    print(s)
    f.ChangeDutyCycle(s)

def imuReading(event):
    global imu_x
    global distance
    bno = BNO055(BNO055.OPERATION_MODE_IMUPLUS)      
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setExternalCrystalUse(True)
    while not event.is_set():
        imu_x,y,z = bno.getVector(BNO055.VECTOR_EULER)
        time.sleep(0.02)
        #distance = laser.range
        time.sleep(0.04)
        #print("x ",imu_x," y ",y," z ",z)
        #print(distance)

def findErrorImu(heading):
    error = heading-imu_x
    if(error > 180):
        error =  error-360
    elif(error < -180):
        error =  360+error
    return error

def imuTurn(heading,kp):
    global imu_x
    global LastErrorImu
    global LastTimeImu
    global servoPin
    global lastSteering
    kd = 0.5
    error = heading-imu_x
    if(error > 180):
        error =  error-360
    elif(error < -180):
        error =  360+error
    CurrentTime = (time.time()*1000)
    rateError = (error-LastErrorImu)/(CurrentTime - LastTimeImu) 
    LastErrorImu = error
    LastTimeImu = CurrentTime
    PD = int((kp*error) + (kd*rateError))
    #print("heading ",heading ," error ",error," PD ",PD," imuX ",imu_x)
    pulseServo = midServo+PD
    if(pulseServo>maxServo):
        pulseServo = maxServo
    if(pulseServo<minServo):
        pulseServo = minServo
    lastSteering = pulseServo
    wiringpi.pwmWrite(servoPin, pulseServo)

def imuInvertTurn(heading,kp):
    global imu_x
    global LastErrorImu
    global LastTimeImu
    global servoPin
    global lastSteering
    kd = 0.1
    error = heading-imu_x
    if(error > 180):
        error =  error-360
    elif(error < -180):
        error =  360+error
    CurrentTime = (time.time()*1000)
    rateError = (error-LastErrorImu)/(CurrentTime - LastTimeImu) 
    LastErrorImu = error
    LastTimeImu = CurrentTime
    PD = int((kp*error) + (kd*rateError))
    #print("heading ",heading ," error ",error," PD ",PD," imuX ",imu_x)
    pulseServo = midServo-PD
    if(pulseServo>maxServo):
        pulseServo = maxServo
    if(pulseServo<minServo):
        pulseServo = minServo
    lastSteering = pulseServo
    wiringpi.pwmWrite(servoPin, pulseServo)

inputString = ""
ser = serial.Serial(
        port='/dev/openmv',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
)

def subString():
    global inputString 
    global greenDetect
    global redDetect
    global turning
    global clockwise
    global front_wall_detect
    global wall_error
    global first_read
    global lastSeeRed
    global lastSeeGreen
    global timeValueGreen
    global timeValueRed
    if inputString[0:2] == "#W":
        commandLength = 6
        commaIndex = [0] * commandLength 
        valueTemp = [0] * commandLength 
        #--------------------------
        commaIndex[0] = inputString.index(',') 
        valueTemp[0] = int(inputString[2: commaIndex[0]]) 
        for i in range(1,commandLength,1): #range(start, stop, step)
            commaIndex[i] = inputString.index(',', commaIndex[i - 1] + 1) #7
        for i in range(1,commandLength,1):
            valueTemp[i] = int(inputString[commaIndex[i - 1] + 1: commaIndex[i]])
        #for i in range(commandLength):
        #    print(i ,"   ", valueTemp[i])
        greenDetect = valueTemp[0]
        redDetect = valueTemp[1]
        turning = valueTemp[2]
        clockwise = valueTemp[3] #0 not detect, 1 clockwise , 2 anti clockwise
        front_wall_detect = valueTemp[4]
        wall_error = valueTemp[5]

        if greenDetect:
            lastSeeGreen = True
            timeValueGreen = time.time()*1000
        if time.time()*1000 - timeValueGreen > 1000 or redDetect or front_wall_detect:
            lastSeeGreen = False
        if redDetect:
            lastSeeRed = True
            timeValueRed = time.time()*1000
        if time.time()*1000 - timeValueRed > 1000 or greenDetect or front_wall_detect:
            lastSeeRed = False

        if first_read == True:
            first_read = False
    inputString = ""

def UARTReading(event) :
    global inputString
    while not event.is_set():
        if(ser.in_waiting):
                data = (ser.read())
                if data == b'#':
                        inputString = "#"
                else:
                        inputString = inputString + data.decode()
                if(data == b'*'):
                        data = ''
                        subString()

def headingWall(headError):
    global LastErrorWall
    global LastTimeWall
    global servoPin
    global lastSteering
    kd = 0.4
    kp = 0.06
    error = headError
    CurrentTime = (time.time()*1000)
    rateError = (error-LastErrorWall)/(CurrentTime - LastTimeWall) 
    LastErrorWall = error
    LastTimeWall = CurrentTime
    PD = int((kp*error) + (kd*rateError))
    
    pulseServo = midServo+PD
    if(pulseServo>maxServo-10):
        pulseServo = maxServo - 10
    if(pulseServo<minServo+10):
        pulseServo = minServo +10
    #print("heading ",error ," PD ",PD, "pulse ", pulseServo)
    lastSteering = pulseServo
    wiringpi.pwmWrite(servoPin, pulseServo)

def motor(speeds):
    if speeds > 100:
        speeds = 100
    elif speeds < -100:
        speeds = -100
        
    if speeds < 0:
        speeds = speeds*(-1)
        f.ChangeDutyCycle(0)
        b.ChangeDutyCycle(speeds)
    else:
        b.ChangeDutyCycle(0)
        f.ChangeDutyCycle(speeds)

def Steering(angle):
    global lastSteering
    global servoPin
    global midServo
    global maxServo
    global minServo
    lastSteering = angle
    if(lastSteering>maxServo):
        lastSteering = maxServo
    elif(lastSteering<minServo):
        lastSteering = minServo
    wiringpi.pwmWrite(servoPin, lastSteering)

def controller(event):
    global greenDetect
    global redDetect
    global turning
    global clockwise
    global front_wall_detect
    global wall_error

    global steps
    global laps
    global counterDegree
    global first_read

    global midServo
    global minServo
    global maxServo
    speedMotor = 22
    speedMotorBack = -36
    firstCheck = True
    distanceObs = 150
    distanceObsSide = 110
    distanceObsSideLeft = 80
    distanceObsSideRight = 80
    global imu_x
    global distanceRight
    global distanceCenter
    global distanceLeft

    global lastSeeRed
    global lastSeeGreen
    global timeValueGreen
    global timeValueRed

    bno = BNO055(BNO055.OPERATION_MODE_IMUPLUS)
    i2c = busio.I2C(board.SCL, board.SDA)
    xshut = [
        DigitalInOut(board.D26),
        DigitalInOut(board.D6),
        DigitalInOut(board.D20),
    ]
    for power_pin in xshut:
        power_pin.switch_to_output(value=False)

    vl53 = []
    for i, power_pin in enumerate(xshut):
        power_pin.value = True
        vl53.insert(i, VL53L0X(i2c))
        if i < len(xshut) - 1:
            vl53[i].set_address(i + 0x30)
    time.sleep(0.04)
    vl53[0].continuous_mode()
    vl53[0].start_continuous()
    vl53[1].continuous_mode()
    vl53[1].start_continuous()
    vl53[2].continuous_mode()
    vl53[2].start_continuous()
    distanceRight = vl53[0].range
    distanceCenter = vl53[1].range
    distanceLeft = vl53[2].range
    while False:
        distanceRight = vl53[0].range
        distanceCenter = vl53[1].range
        distanceLeft = vl53[2].range
        print(distanceRight,"  ",distanceCenter,"  ", distanceLeft)
    while first_read or clockwise != 0:
        print("UART connect....")
        motor(0)
        ser.write('#'.encode())
        time.sleep(0.5)

    print("Ready")
    GPIO.output(4, 0)

    while GPIO.input(Button) == 0:
        time.sleep(0.1)

    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(0.1)
    bno.setExternalCrystalUse(True)
    imu_x,y,z = bno.getVector(BNO055.VECTOR_EULER)
    while GPIO.input(Button) == 1:
        GPIO.output(4, 1)
        time.sleep(0.1)
    GPIO.output(4, 1) 
    time.sleep(0.2)
    GPIO.output(4, 0)   
    for i in range(speedMotor):
        motor(i)
        time.sleep(0.05)
    
    lastStepTime = 0
    lastStep1Time = time.time()*1000
    turnTime = time.time()*1000
    lastServo = 0
    vl53[0].start_continuous()
    time.sleep(0.01)
    vl53[1].start_continuous()
    time.sleep(0.01)
    vl53[2].start_continuous()
    time.sleep(0.01)
    stepFindWall = 1
    lastRedDetect = False
    lastCounterDegree = 0
    while not event.is_set():
        #print(steps)
        imu_x,y,z = bno.getVector(BNO055.VECTOR_EULER)
        time.sleep(0.01)
        distanceRight = vl53[0].range
        distanceCenter = vl53[1].range
        distanceLeft = vl53[2].range
        #print("L ",distanceLeft, " C ",distanceCenter," R ",distanceRight)
        if turning and steps == 1 and (time.time()*1000-lastStepTime > 3000):
            steps = 2
            firstCheck = True
            print("step = 2")
            motor(speedMotor) 
                   
          

            turnTime = time.time()*1000
            lastStepTime = time.time()*1000
                


        if steps == 1 and laps != 12 and laps != 0:

            
            if(distanceCenter<distanceObs or distanceLeft<distanceObsSideLeft or distanceRight<distanceObsSideRight):
            #if(distanceCenter<distanceObs ):
                print("4L ",distanceLeft, " C ",distanceCenter," R ",distanceRight, "rr ", lastSeeRed)
                
                if lastSeeRed == False and lastSeeGreen == False:
                        print("1G0R0")
                        if findErrorImu(counterDegree) > 0:
                            motor(speedMotorBack)
                            Steering(minServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(maxServo-10)
                            time.sleep(1.6)
                        elif findErrorImu(counterDegree) < 0:
                            motor(speedMotorBack)
                            Steering(maxServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(minServo-10)
                            time.sleep(1.6)
                        else:
                            motor(speedMotorBack)
                            Steering(midServo)
                            time.sleep(0.6)
                elif lastSeeGreen:
                    print("1G1R0")
                    motor(speedMotorBack)
                    Steering(maxServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(minServo-10)
                    time.sleep(0.5)

                elif lastSeeRed:
                    print("1G0R1")
                    motor(speedMotorBack)
                    Steering(minServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(maxServo)
                    time.sleep(0.5)

                else:
                    motor(speedMotor)
                    imuTurn(counterDegree,0.7)
                    print("imu turn force ", counterDegree , "  ", imu_x) 
                    time.sleep(0.2) 

            elif(greenDetect):
                motor(speedMotor)
                Steering(midServo-int((greenDetect-30)/1.5))
                time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = -1
                distanceCenter = 8000
                
            elif(redDetect):
                motor(speedMotor)
                Steering(midServo+int((redDetect-30)/1.5))
                time.sleep(0.01)
                lastStep1Time = time.time()*1000
                lastServo = 1
                distanceCenter = 8000
            else:
                if(time.time()*1000-lastStep1Time<50):
                    #print("turnnn")
                    motor(speedMotor)
                    distanceCenter = 8000
                elif(time.time()*1000-lastStep1Time<200):
                    motor(0)
                    Steering(midServo)
                    distanceCenter = 8000
                elif(time.time()*1000-lastStep1Time<300):
                    #print("straing")
                    motor(speedMotor)
                    Steering(midServo)
                    distanceCenter = 8000
                    '''
                elif(time.time()*1000-lastStep1Time<800):
                    motor(speedMotor)
                    if lastServo ==- 1:
                        Steering(maxServo-5)
                    else:
                        Steering(minServo+5)
                    '''
                elif(time.time()*1000-lastStep1Time<0):
                    motor(speedMotor)
                    imuTurn(wall_error,1.8) 
                else:
                    motor(speedMotor )
                    headingWall(wall_error) 
                    #imuTurn(counterDegree,0.4) 
        
        if steps == 1 and laps == 0:
            if(distanceCenter<distanceObs or distanceLeft<distanceObsSideLeft or distanceRight<distanceObsSideRight):

                #print("4L ",distanceLeft, " C ",distanceCenter," R ",distanceRight, "rr ", lastSeeRed)
                
                if lastSeeRed == False and lastSeeGreen == False:
                        print("1G0R0")
                        if findErrorImu(counterDegree) > 0:
                            motor(speedMotorBack)
                            Steering(minServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(maxServo)
                            time.sleep(1.6)
                        elif findErrorImu(counterDegree) < 0:
                            motor(speedMotorBack)
                            Steering(maxServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(minServo)
                            time.sleep(1.6)
                        else:
                            motor(speedMotorBack)
                            Steering(midServo)
                            time.sleep(0.6)
                elif lastSeeGreen:
                    print("1G1R0")
                    motor(speedMotorBack)
                    Steering(maxServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(minServo)
                    time.sleep(0.5)
                elif lastSeeRed:
                    print("1G0R1")
                    motor(speedMotorBack)
                    Steering(minServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(maxServo)
                    time.sleep(0.6)

                    Steering(midServo)
                    time.sleep(0.6)
                else:
                    motor(speedMotor)
                    imuTurn(counterDegree,0.7)
                    print("imu turn force ", counterDegree , "  ", imu_x) 
                    time.sleep(0.2) 

                  
            elif(abs(findErrorImu(counterDegree))>90):
                motor(speedMotor)
                imuTurn(counterDegree,0.9)
                print("imu turn force ", counterDegree , "  ", imu_x) 
                time.sleep(0.3)
            elif(greenDetect):
                motor(speedMotor)
                Steering(midServo-int((greenDetect-20)/1.5))
                #time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = -1
                
            elif(redDetect):
                motor(speedMotor)
                Steering(midServo+int((redDetect-20)/1.5))
                time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = 1
            else:
                if(time.time()*1000-lastStep1Time<100):
                    #print("turnnn")
                    motor(speedMotor)
                    distanceCenter = 8000
                elif(time.time()*1000-lastStep1Time<100):
                    motor(0)
                    Steering(midServo)
                    distanceCenter = 8000
                elif(time.time()*1000-lastStep1Time<200):
                    #print("straing")
                    motor(speedMotor)
                    Steering(midServo)
                    distanceCenter = 8000
                elif(time.time()*1000-lastStep1Time<200):
                    motor(speedMotor)
                    imuTurn(wall_error,0.8) 
                else:
                    motor(speedMotor)
                    headingWall(wall_error) 
                    #imuTurn(counterDegree,0.4) 
            
        
        elif steps == 2  :

            if time.time()*1000 - turnTime < 500 :
                    motor(speedMotor+5)
                    imuTurn(lastCounterDegree,0.5)
                    time.sleep(0.01)
            
            elif time.time()*1000 - turnTime < 2600 :
                    motor(speedMotor+5)
                    imuTurn(counterDegree,0.8)
                    time.sleep(0.01)

            elif time.time()*1000 - turnTime < 0 :
                if clockwise == 1:#blue
                    Steering(maxServo)
                elif clockwise == 2:#orange
                    Steering(minServo)
                motor(speedMotor)
            elif time.time()*1000 - turnTime < 2700:
                motor(0)
            elif time.time()*1000 - turnTime < 5800 :
                motor(speedMotorBack)
                if firstCheck:
                    lastCounterDegree = counterDegree
                    firstCheck = False
                    if clockwise == 2:
                        print("blue")#2
                        distanceObsSideLeft = 120
                        distanceObsSideRight = 120
                        counterDegree = counterDegree - 90
                        if counterDegree < 0:
                            counterDegree = 270
                        
                    elif clockwise == 1:
                        print("orange")#1
                        distanceObsSideLeft = 120
                        distanceObsSideRight = 120
                        counterDegree = counterDegree + 90
                        if counterDegree > 359:
                            counterDegree = 0  
                imuInvertTurn(counterDegree,1.8)
            elif time.time()*1000 - turnTime < 5900:
                motor(0)
                Steering(midServo)
            elif time.time()*1000 - turnTime < 6000:
                motor(speedMotor)
                Steering(midServo)
            elif time.time()*1000 - turnTime < 10000:
                Steering(midServo)
                motor(0)
                laps += 1
                steps = 3 
                print("laps = ",laps)
                print("degree = ",counterDegree)
                print("step = 3")
            
        elif steps == 2 and laps == -1:
            if distanceCenter < 400 and stepFindWall == 1 :
                
                turnTime = time.time()*1000
                stepFindWall = 2 
                
            elif time.time()*1000 - turnTime < 200 and stepFindWall == 2 :
                motor(0)
                stepFindWall = 3 
            elif stepFindWall == 3 :
                motor(-50)
                imuInvertTurn(90,0.8)
                turnTime = time.time()*1000
                if (findErrorImu(90)) < 0 :
                    stepFindWall = 4 
            elif time.time()*1000 - turnTime < 2000 and stepFindWall == 4 :
                motor(0)
                print(distanceCenter," ", distanceLeft, " " ,distanceRight)
            
                
                    
                
            else:
                if stepFindWall == 1:
                    motor(50)
                    imuTurn(0,0.8)
                elif stepFindWall == 4:
                    if distanceCenter > 1200 or distanceLeft > 1200 or distanceRight > 1200:
                        clockwise = 1
                        print("orange")#1
                        distanceObsSideLeft = 120
                        distanceObsSideRight = 120
                        counterDegree = counterDegree + 90
                        if counterDegree > 359:
                            counterDegree = 0  
                    else:
                        clockwise = 2
                        print("blue")#2
                        counterDegree = counterDegree - 90
                        if counterDegree < 0:
                            counterDegree = 270
                        laps += 1
                        steps = 3 
                        print("laps = ",laps)
                        print("degree = ",counterDegree)
                        print("step = 3")
                        motor(0)
                        time.sleep(1000)
            

        elif steps == 3:
            if(distanceCenter<distanceObs or distanceLeft<distanceObsSideLeft or distanceRight<distanceObsSideRight):
            #if(distanceCenter<distanceObs ):
                print("4L ",distanceLeft, " C ",distanceCenter," R ",distanceRight, "rr ", lastSeeRed)
                
                if lastSeeRed == False and lastSeeGreen == False:
                        print("4G0R0")
                        if findErrorImu(counterDegree) > 0:
                            motor(speedMotorBack)
                            Steering(minServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(maxServo-10)
                            time.sleep(1.6)
                        elif findErrorImu(counterDegree) < 0:
                            motor(speedMotorBack)
                            Steering(maxServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(minServo-10)
                            time.sleep(1.6)
                        else:
                            motor(speedMotorBack)
                            Steering(midServo)
                            time.sleep(0.6)
                elif lastSeeGreen:
                    print("3G1R0")
                    motor(speedMotorBack)
                    Steering(maxServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(minServo)
                    time.sleep(0.5)
                elif lastSeeRed:
                    print("3G0R1")
                    motor(speedMotorBack)
                    Steering(minServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(maxServo)
                    time.sleep(0.5)

                else:
                    motor(speedMotor)
                    imuTurn(counterDegree,0.7)
                    print("imu turn force ", counterDegree , "  ", imu_x) 
                    time.sleep(0.2) 

                
            elif (abs(findErrorImu(counterDegree))>45): 
                if findErrorImu(counterDegree) < 0:
                    motor(speedMotor)
                    Steering(minServo)
                else:
                    motor(speedMotor)
                    Steering(maxServo)

            elif(greenDetect):
                print("g3")
                motor(speedMotor)
                Steering(midServo-int((greenDetect-30)/2))
                time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = -1
            elif(redDetect):
                print("r3")
                motor(speedMotor)
                Steering(midServo+int((redDetect-20)/1.5))
                time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = 1
            else:
                if(time.time()*1000-lastStep1Time<200):
                    #print("turnnn")
                    motor(speedMotor)
                elif(time.time()*1000-lastStep1Time<0):
                    #print("straing3")
                    motor(speedMotor)
                    Steering(midServo)
                    '''
                elif(time.time()*1000-lastStep1Time<800):
                    motor(speedMotor-10)
                    if lastServo ==- 1:
                        Steering(maxServo-5)
                    else:
                        Steering(minServo+5)
                    '''
                else:
                    motor(speedMotor)
                    imuTurn(counterDegree,0.9) 
            if abs(imu_x-counterDegree)<20:
                steps = 4
                print("steps = 4")
                motor(speedMotor)
                lastStepTime = time.time()*1000

        elif steps == 4:           
            if(distanceCenter<distanceObs or distanceLeft<distanceObsSideLeft or distanceRight<distanceObsSideRight):
            #if(distanceCenter<distanceObs ):
                print("4L ",distanceLeft, " C ",distanceCenter," R ",distanceRight, "rr ", lastSeeRed)
                
                if lastSeeRed == False and lastSeeGreen == False:
                        print("4G0R0")
                        if findErrorImu(counterDegree) > 0:
                            motor(speedMotorBack)
                            Steering(minServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(maxServo-10)
                            time.sleep(1.6)
                        elif findErrorImu(counterDegree) < 0:
                            motor(speedMotorBack)
                            Steering(maxServo)
                            time.sleep(0.6)
                            motor(0)
                            time.sleep(0.2)
                            motor(speedMotor)
                            Steering(minServo-10)
                            time.sleep(1.6)
                        else:
                            motor(speedMotorBack)
                            Steering(midServo)
                            time.sleep(0.6)
                elif lastSeeGreen:
                    print("4G1R0")
                    motor(speedMotorBack)
                    Steering(maxServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(minServo-10)
                    time.sleep(0.5)

                elif lastSeeRed:
                    print("4G0R1")
                    motor(speedMotorBack)
                    Steering(minServo)
                    time.sleep(0.6)
                    motor(0)
                    time.sleep(0.2)
                    motor(speedMotor)
                    Steering(maxServo)
                    time.sleep(0.5)

                else:
                    motor(speedMotor)
                    imuTurn(counterDegree,0.7)
                    print("imu turn force ", counterDegree , "  ", imu_x) 
                    time.sleep(0.2) 

                
            elif (abs(findErrorImu(counterDegree))>45): 
                if findErrorImu(counterDegree) < 0:
                    motor(speedMotor)
                    Steering(minServo)
                else:
                    motor(speedMotor)
                    Steering(maxServo)
                
            elif(greenDetect):
                print("g4")
                motor(speedMotor)
                Steering(midServo-int((greenDetect-30)/2))
                #time.sleep(0.1)
                lastStep1Time = time.time()*1000
                lastServo = -1
                distanceCenter  = 8000
                
            elif(redDetect):
                print("r4")
                motor(speedMotor)
                print(midServo+int((redDetect-20)/1.5))
                Steering(maxServo)
                lastStep1Time = time.time()*1000
                lastServo = 1
                lastRedDetect = True
            else:
                '''
                if lastRedDetect:
                    Steering(minServo)
                    time.sleep(0.1)
                    lastRedDetect = False
                    '''
                motor(speedMotor)
                imuTurn(counterDegree,3) 
                time.sleep(0.01)
                #imuTurn(counterDegree,0.4)
                #headingWall(speedMotor)
            if time.time()*1000 - lastStepTime > 7000 and laps != 12:
                steps = 1
                print("steps = 1 reset")
                motor(speedMotor)
                lastStepTime = time.time()*1000
            elif time.time()*1000 - lastStepTime > 4000 and laps == 12 and lastSeeGreen == False and lastSeeRed == False:
                steps = 1
                print("steps = 1 reset")
                motor(speedMotor)
                lastStepTime = time.time()*1000

        elif steps == 1 and laps == 12:
            motor(speedMotor)
            headingWall(wall_error) 
            if time.time()*1000 - lastStepTime > 1000:
                motor(0)
                break

    Steering(midServo)
    print("Kill programe")
if __name__ == '__main__':
    #threading.Thread(target = controller).start()
    #threading.Thread(target = UARTReading).start()
    #threading.Thread(target = imuReading).start()
    try:
        event = threading.Event()
        thread1 = threading.Thread(target=controller, args=(event,))
        thread2 = threading.Thread(target=UARTReading, args=(event,))
        #thread3 = threading.Thread(target=imuReading, args=(event,))
        thread1.start()
        thread2.start()
        #thread3.start()
        event.wait()  # wait forever but without blocking KeyboardInterrupt exceptions
    except KeyboardInterrupt:
        time.sleep(0.5)
        f.stop()
        wiringpi.pwmWrite(servoPin, midServo)
        GPIO.cleanup()
        print("cleanup")
        event.set()  # inform the child thread that it should exit
