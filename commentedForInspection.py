#!/usr/bin/env micropython
#? For explaining declarations.
#* For explaning classes and functions
#! Something that is important to look at later, or might be confusing
#~ For explaining very long and confusing calulcations.
#^ For explaining a chunk of code, or a group of functions
from math import sin, cos, degrees, atan2, radians, sqrt, fmod
from time import time, sleep
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from ev3dev2.motor import LargeMotor, MoveTank, MoveSteering, MediumMotor
from ev3dev2.power import PowerSupply
from ev3dev2.sound import Sound
from ev3dev2.console import Console
from ev3dev2.button import Button
from os import system
from ev3dev2.led import Leds
#? Imports
gyroCorrection = 0.99174
#? The gyroscope of the robot counts 363 degrees in one rotation rather than 360, so we always multiply it by this constant
gyroOffset = 0
currentX = 0
currentY = 0
rotationsText = open("rotations.txt", "r+")
wheelDiameter = float(rotationsText.readline())
rotationsText.close()
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
system("chmod +x loadCtrlC.sh")
system("sh ../miau/loadCtrlC.sh")
#? Remapping the "up" key to the ctrl-c key combination
speaker = Sound()
console = Console()
console.set_font(font='Lat15-Terminus32x16', reset_console=False)
button = Button()
leds = Leds()
battery = PowerSupply()
#? Defining all built in sensors and outputs of the robot
class Port:
    def __init__(self, id, type):
        self.id = id
        self.type = type
def diagnostics(ports):
    badPorts = []
    console.set_font(font='Lat15-Fixed15.psf.gz', reset_console=False)
    for port in ports:
        sleep(0.05)
        try:
            leds.all_off()
            eval(port.type + "('"+port.id+"')")
        except:
            print("|X| " + port.type + " " + port.id)
            leds.set_color("LEFT", "RED", 1)
            leds.set_color("RIGHT", "RED", 1)
            speaker.tone([(880, 75, 10),(440, 150, 10),(220, 225, 10),(110, 300, 10)])
            sleep(0.2)
            badPorts.append(port)
        else:
            print("|✓| " + port.type + " " + port.id,)
            leds.set_color("LEFT", "GREEN", 1)
            leds.set_color("RIGHT", "GREEN", 1)
            speaker.tone([(440, 75, 10),(523, 75, 10),(659, 75, 10)])
    #? Iterating trough all the inputs and outputs and checking if they are connected.
    if(len(badPorts) != 0):
        print("Connect all ports!")
        for badPort in badPorts:
            print(port.type + " " + port.id + " |x|")
    while len(badPorts) != 0:
        sleep(0.05)
        for badPort in badPorts:
            try:
                eval(badPort.type + "('"+badPort.id+"')")
            except:
                leds.set_color("LEFT", "RED", 1)
                leds.set_color("RIGHT", "RED", 1)
                speaker.play_tone(frequency=1000, duration=0.1)
                pass
            else:
                print(port.type + " " + port.id + " connected")
                leds.set_color("LEFT", "GREEN", 1)
                leds.set_color("RIGHT", "GREEN", 1)
                speaker.tone([(440, 75, 10),(523, 75, 10),(659, 75, 10)])
                badPorts.remove(badPort)
                sleep(0.2)
    #? If any ports are disconnected wait untill all of them are disconnected.
    leds.animate_flash('GREEN', sleeptime=0.75, duration=20, block=False)
    console.set_font(font='Lat15-Terminus32x16', reset_console=True)
diagnostics([Port("in2", "GyroSensor"), Port("in3", "ColorSensor"), Port("in4", "ColorSensor"), Port("outA", "MediumMotor"), Port("outD", "MediumMotor"), Port("outB", "LargeMotor"), Port("outC", "LargeMotor")])
#? Checking if all ports
leftMotor = LargeMotor("outB")
rightMotor = LargeMotor("outC")
#? Individual motors defined
tankMovement = MoveTank(leftMotor.address, rightMotor.address)
steeringMovement = MoveSteering(leftMotor.address, rightMotor.address)
#? Getting the adresses of previously set motors and defining the movements based on those.
yHand = MediumMotor("outD")
xHand = MediumMotor("outA")
#? Defining the Medium Motors operating the vertical motion of the robot, and the extensions
gyro = GyroSensor("in2")
rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")
gs = GyroSensor("in2")
gs.mode = GyroSensor.MODE_GYRO_ANG
rotations = 0

class Vector:
    """#* This class is used for the bezier curve, it's a simple 2D vector."""
    def __init__(self, x, y):
        self.x = -x
        self.y = y
def optimizeFloat(num):
    """#* This function just rounds the float to a given value, so it uses less computing power as working with long doubles isn't as efficient"""
    return round(float(num), 4)
def sign(num):
    """#* Returns the sign of a number, and returns a 1 if the number is 0"""
    if(num == 0):
        return 1
    return(num / abs(num))
def findBigger(num1, num2):
    """#* Subtracts the smallest number from all numbers in a vector and returns it."""
    smallest = min(num1, num2)
    num1 -= smallest
    num2 -= smallest
    return [num1, num2]
def distance(x1, y1, x2, y2):
    """#* A basic distance calculatoion for a two dimentional vector, with the distance being calculated insid the function."""
    return sqrt((x2 - x1) **2 + (y2 - y1) ** 2)
def getPointOnBezier(controllPoints, t):
    """#* Returns the X and Y value of a single point on a bezier curve based on a decimal of [0, 1]"""
    n = len(controllPoints) -1
    x = 0
    y = 0
    for i in range(n + 1):
        bi = bernstein(n, i, t)
        x += controllPoints[i].x * bi
        y += controllPoints[i].y * bi
    return Vector(x, y)
def bernstein(n, i, t):
    if i < 0 or i > n:
        return 0
    if n == 0 and i == 0:
        return 1
    binomial = factorial(n) / (factorial(i) * factorial(n - i))
    t1 = pow(1 -t, n - i)
    t2 = pow(t, i)
    return binomial * t1 * t2
def factorial(n):
    """#* Returns the factorial of a number"""
    if(n <= 1): return 1
    result = 1
    for i in range(2, n + 1):
        result *= i
    return result
def bezierLenght(controlPoints, n = 100):
        """#* Returns the lenght of a bezier curve based on controllpoints and the number of sections (n)"""
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        previousPoint = getPointOnBezier(controlPoints, 0)
        
        for _ in range(0, n):
            point = getPointOnBezier(controlPoints, t)
            totalLength += distance(previousPoint.x, previousPoint.y, point.x, point.y)
            t += dt
            previousPoint = point
        return totalLength
#^ Functions that are only mathematical, and don't have any direct connection to the robot
def gsAngle():
    """#* Returns the angle of the gyroscope, (this also takes into account the offset and the correction of the gyroscope)"""
    return fmod(optimizeFloat((gyro.angle + gyroOffset) * gyroCorrection), 360)
def getRotations():
    """#* Returns the current rotation of the two large motors"""
    return optimizeFloat((leftMotor.rotations + rightMotor.rotations) / 2 * wheelDiameter)
def setPosition(angle, x, y):
    ""#* Sets the gyroscope angle and the x and y coordinates of the robot."""
    global gyroOffset
    global currentX
    global currentY
    gyroOffset = angle
    currentX = x
    currentY = y
def shortest_angle(angle, target_angle):
    """#* Calculates the shortest path the robot has the make to reach a desired angle."""
    diff = target_angle - angle
    #? Calculate the absolute difference between the angles
    if abs(diff) > 180:
        #? Check if the difference is more than 180 degrees
        target_angle -= 360
        target_angle *= sign(diff) 
        #? Recalculate the target angle with the shortest path
    return target_angle
def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance, shouldSpeedUp):
    """"#* This function calculates the speed at a given distance, for linearly speeding up and slowing down."""
    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target
    returnSpeed = maxSpeed
    if(deltaDistance < speedUp and shouldSpeedUp == True):
        #? If the motors rotations aren't higher than the specified speedUp value it will continue to accelerate the robot.
        returnSpeed = (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
        #~               a decimal of [0,1]        subtracting minSpeed because it will always go with atleast minSpeed
    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        #? If the distance from the target is larger than the distance of the target in the beggining and the distance from which you should be slowing down, you start to slow down
        if(motorStop != False):
            minSpeed = motorStop
        #? If motorStop isn't false, the robot will only slow down to the provided percentage (and it will keep it's motors on after the function)
        returnSpeed = maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~                                         decimal of [0, 1]          inversse of the previous calculation of speeding up + minSpeed
    if(abs(returnSpeed) > 100): returnSpeed = sign(returnSpeed) * 100
    #? Checks if the returned percentage is larger than 100
    return round(float(returnSpeed), 2)
#^ Functions that don't directly move the robot
def moveRobotOnLine(motor, szinSzenzor, minFeny, maxSebesseg, KP):
    """#* Moves the specified side of the robot based on that color sensors data."""
    returnSpeed = (minFeny - szinSzenzor.reflected_light_intensity) * KP
    #~ Calculating the speed of the motor based on the light of the Color sensor on the provided side of the robot.
    if(abs(returnSpeed) > maxSebesseg):            
        returnSpeed = maxSebesseg * (returnSpeed / abs(returnSpeed))
    #? Making sure calculated speed isn't larger than the provided maxSpeed
    motor.on(returnSpeed)
    return round(float(returnSpeed), 2)
    #? Returns the calculated speed (rounded because of computation) so the program knows when the speed is 0 so it knows the robot has found the line.
def goOnLine(KP, maxIdo, maxSebesseg, minimumFeny):
    """#* Makes to robot perpendicularly go on a line, works with 2 sensors only."""
    elozoIdo = time()
    while True:            
        elteltIdo = time() - elozoIdo
        if(moveRobotOnLine(leftMotor, leftSensor, minimumFeny, maxSebesseg, KP) == 0 and moveRobotOnLine(rightMotor, rightSensor, minimumFeny, maxSebesseg, KP) == 0):
            #? If the calculated speed for both motors are 0, the program stops.
            tankMovement.stop(None, False)
            break
        if(elteltIdo >= maxIdo):
            tankMovement.stop(None, False)
            break
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False):
    """#* Make the robot go straight in a specified degree for a specified distance (cm)"""
    direction = sign(maxSpeed)
    minSpeed *= direction
    if(shouldSpeedUp == True):
        tankMovement.on(minSpeed, minSpeed)
    else:
        tankMovement.on(maxSpeed, maxSpeed)
    #? Starts the motors before some minor computations to save some time.
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    if(speedingUp == False):
        speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
        if(speedingUp > (2 * wheelDiameter)):
            speedingUp = (2 * wheelDiameter)
    if(slowingDown == False):
        slowingDown = distance * 0.6
        if(slowingDown > (2*wheelDiameter)):
            slowingDown = (2 * wheelDiameter)
    #? Sets the default values for speeding up and slowing down.
    while abs(getRotations() - startRotations) <= distance:
        if(calibrate == False):
            currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
            currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
            rotations = getRotations()
        #? If it tries to calibrate the rotations get reset it would go for a long time if it didn't find the line.
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #? Checks if it should stop when it detects a line.
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #? Checks if it should perpendiculalrly go on a line.
                tankMovement.stop(None, False)
                break
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed+1)) * 2
        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5
        calculatedSensitivity = optimizeFloat(sensitivity / sensitivityMultiplier)
        #? The sensitivity decreases as the robot goes faster.
        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            calculatedAngle *= direction
            calculatedAngle /= 3.6
            #? The movesteering only accepts a percentage rather than an angle.
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            steeringMovement.on(calculatedAngle, calculatedSpeed)
    if(calibrate):
        newWheelDiameter = optimizeFloat(calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop):
        tankMovement.on(motorStop, motorStop)
    else:
        tankMovement.stop(None, False)
    if(calibrate):
        leftMotor.reset()
        rightMotor.reset()
        return (newWheelDiameter)
def turn(angle, maxSpeed, sensitvity, relative = True, stopMargin = 2, minSpeed = 2, timeout = 2):
    angle = angle * -1
    fordulatszam = 0
    hasStopped = False
    if(relative == True):
        fordulatszam = gsAngle()
    #? Wether it should turn to the targetAngle based on 0 or the current gyroscope angle.
    turnConstant = (fordulatszam-angle)
    stopMargin *= sign(turnConstant)
    startTime = time()
    while gsAngle() != fordulatszam - angle:
        if(time() - startTime >= timeout): break
        if(turnConstant - stopMargin <= gsAngle() <= turnConstant + stopMargin and hasStopped == False and stopMargin != 0):
            #? Stops the robot completely for one cycle if it's near to the targetAngle.
            hasStopped = True
            tankMovement.stop()
            continue
        calculatedSpeed = (turnConstant - gsAngle()) * sensitvity
        if(calculatedSpeed > maxSpeed / 3.5):
            calculatedSpeed = maxSpeed
        #? This is just to make a robot go with maxSpeed for a little longer, which still won't make it overturn.
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(maxSpeed) * sign(calculatedSpeed))
        if(abs(calculatedSpeed) < minSpeed): calculatedSpeed = (abs(minSpeed) * sign(calculatedSpeed))
        tankMovement.on(-calculatedSpeed, calculatedSpeed)
    tankMovement.stop()
#^ Functions having to do with movement of the robot
def calibrate():
    """#* Runs a short calibration script, that calculates how many cm is one rotation."""
    dist = 83
    gyro.calibrate()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, 0, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -100, 0, 3.6, 20, False, False, False, True, True, 0, 0)
    rotationsText = open("rotations.txt", "w+")
    rotationsText.write(str(wheelDiameter))
    rotationsText.close()
def run1():
    setPosition(-44, 24, 18.25)
    print(wheelDiameter)
    yHand.on_for_rotations(80, 0.7, block=False)
    straight(29.5, 70, -41, 4.8, 3, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(-100, 0.5)
    xHand.on_for_rotations(70, -0.55)
    #? Pushing the water cell away, so the normal cell can roll back to home.
    yHand.on_for_rotations(100, 0.6, block=False)
    straight(4.9, 20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(100, -0.65, block=False)
    #? Going into the dam and lifting the hand when it's just below the spinning part, so the green cell comes out.
    yHand.on_for_rotations(100, 1)
    straight(0.8, -20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(90, -2.22, block=False)
    #? Going back a bit and extending the arm fully so the robot hooks onto the water cell previously pushed away.
    straight(25, -70, -40, 1.1, 5, False, False, False, True, True, 0, 0)
    straight(2.5, 40, -40, 1.1, 40)
    #? Coming back and going forward a bit so the operators can remove the water cell because the robot isn't touching it anymore
    xHand.on_for_rotations(15, 0.7, block=False)
    yHand.on_for_rotations(30, 2.5, block=False)
    straight(17, 55, -20, 3.5, 4, motorStop=30)
    xHand.on_for_rotations(-100, 1, block=False)
    straight(35, 70, 5, 5, 30, motorStop=70)
    steeringMovement.on_for_rotations(0, 30, 0.8)
    #? Going to the energy storage (the bezier only goes there about 3/4 because it has to move it's hand out of the way)
    tankMovement.on_for_rotations(-20, -20, 0.03)
    yHand.on_for_rotations(-100, 1, block=True)
    yHand.on_for_rotations(-100, 1.5, block=False)
    #? Moving the arm and going to energy storage.
    """sleep(0.2)
    gyro.reset()
    setPosition(0, 180, 97)
    sleep(0.2)"""
    straight(distance=10.5,maxSpeed=-50,targetAngle=5,sensitivity=0,minSpeed=4,speedingUp=0.3,slowingDown=0.8)
    xHand.on_for_rotations(100, 1.4)
    raiseSpeed = 100
    raiseHeight = 0.8
    yHand.reset()
    yHand.on_for_rotations(raiseSpeed, 1.15, block=True)
    for _ in range(0,3):
        yHand.on_for_rotations(-raiseSpeed, raiseHeight, block=True)
        yHand.on_for_rotations(raiseSpeed, raiseHeight, block=True)
    yHand.on_for_rotations(-raiseSpeed, 1.15, block=True)
    xHand.on_for_rotations(-60, 0.5, False, False)
    turn(-85, 70, 0.5, False, 10, 2, 0.5)
    xHand.on_for_rotations(100, 0.5, False, False)
    straight(distance=6,maxSpeed=40,targetAngle=-87,sensitivity=0.8,minSpeed=5,speedingUp=0.5,slowingDown=0.3)
    turn(-86, 70, 0.55, False, 10, 1, 1)
    straight(distance=23,maxSpeed=-55,targetAngle=-86,sensitivity=4,minSpeed=30,speedingUp=0.5)
    xHand.on_for_rotations(-100, 0.5, False, False)
    straight(distance=7,maxSpeed=40,targetAngle=-90,sensitivity=0.8,minSpeed=5,speedingUp=0.5)
    straight(distance=5,maxSpeed=-75,targetAngle=-90,sensitivity=4,minSpeed=30,speedingUp=0.5)
    straight(distance=5,maxSpeed=75,targetAngle=-90,sensitivity=5,minSpeed=30,speedingUp=0.5)
    turn(8, 40, 0.3, False, 10, 2, 1)
    xHand.on_for_rotations(100, 0.25, False, False)
    straight(distance=50,maxSpeed=-70,targetAngle=-30,sensitivity=2,minSpeed=5,speedingUp=0.5, shouldSlowDown=False)
    turn(-90, 80, 0.6, False, 10, 2, 0.5)
    tankMovement.on_for_rotations(-70, -70, 1)
def run2():
    yHand.on_for_rotations(30, 1.2, True, False)
    setPosition(-29, 33.5, 19.5)
    straight(90, 70, -28.5, 5, 20)
    turn(-90, 30, 0.45, relative=False, timeout=0.6)
    straight(10, 40, -90, 4.5, 25, motorStop=30)
    turn(-98, 30, 0.45, relative=False, timeout=0.6)
    yHand.on_for_rotations(-60, 1.2, True, False)
    straight(5, 40, -98, 5, 25)
    straight(8, 30, -98, 6.5, 25, shouldSpeedUp=False)
    straight(0.35, -10, -90, 4, 5)
    turn(-78, 15, 0.6, relative=False, timeout=0.6)
    yHand.on_for_rotations(100, 0.8, True, False)
    turn(-180, 20, 0.55, relative=False, timeout=0.8)
    straight(18, -70, -180, 6, 30, motorStop=-70)
    sleep(0.1)
    gyro.reset()
    setPosition(180, 106, 112)
    sleep(0.15)
    straight(7.5, 40, 180, 1, 25)
    straight(17, 50, 231.5, 5, 25, motorStop=33)
    yHand.on_for_rotations(60, 1.4, True, False)
    straight(27, 33, 185, 10, 3, shouldSpeedUp=False)
    straight(13, -30, 310.5, 3.6, 20)
    yHand.on_for_rotations(-70, 2.2, True, False)
    turn(207, 30, 0.6, False, 2, 2, 0.6)
    straight(20, 15, 205, 8, 6, True, False)
    straight(5.8, 20, 215, 6, 1)
    turn(180, 60, 0.55, False, 2, 2, 0.45)
    straight(22, 30, 180, 6, 10, slowingDown=0.7)
    straight(0.09, -10, 180, 9, 1)
    xHand.on_for_rotations(40, 1.2, True, True)
    straight(10, -100, 180, 5, 20)
    turn(245, 50, 0.55, False, 2, 2, 0.4)
    straight(29, 100, 252, 5, 30, slowingDown=0.05, motorStop=100)
    straight(65, 100, 287, 5, 10, slowingDown=0.05, speedingUp=0.95, shouldSpeedUp=False)
def run3():
    setPosition(0, 200, 18)
    straight(27, 70, 0, 3.5, 20, motorStop=25)
    straight(13.5, 35, 0, 3.5, 15, shouldSpeedUp=False)
    straight(11, -60, 0, 1.2, 20)
    turn(40, 60, 0.4, False, 2, 5, 0.8)
    xHand.on_for_rotations(-100, 3, block=False)
    straight(12, 60, 40, 3.5, 20, motorStop=55)
    xHand.on_for_rotations(100, 0.5, block=False)
    straight(30, 80, 54, 7, 10, shouldSpeedUp=False, motorStop=80)
    straight(18.5, 80, 44, 7, 10, shouldSpeedUp=False)
    xHand.on_for_rotations(100, 2)
    xHand.on_for_rotations(100, 1, block=False)
    straight(0.1, -60, 44, 6, 10, motorStop=-60)
    yHand.on_for_rotations(100, 0.5, block=False)
    straight(16.5, -60, 44, 6, 10,shouldSpeedUp=False) 
    yHand.on_for_rotations(-100, 0.5, block=False)
    xHand.on_for_rotations(100, 0.5, block=False)
    sleep(0.05)
    turn(-42, 60, 0.475, False, 2, 4, 1)
    straight(16, 80, -42, 6, 10, motorStop=80)
    tankMovement.on_for_rotations(80, 80, 1)
    xHand.on_for_rotations(-100, 1.5, block=False)
    for _ in range(0,3):
        straight(2, -40, -42, 4, 10, motorStop=-40)
        xHand.on_for_rotations(-100, 0.8, block=False)
        straight(2, -40, -42, 4, 10, shouldSpeedUp=False)
        sleep(0.2)
        xHand.on_for_rotations(100, 0.8, block=False)
        straight(16, 60, -42, 4, 10, shouldSlowDown=False)
        sleep(0.3)
    xHand.on_for_rotations(100, 1.5, block=False)
    straight(20, -70, -52, 4, 10, motorStop=-25)
    straight(15, -25, -52, 4, 10, shouldSpeedUp=False)
    straight(1.5, 25, -52, 4, 10)
    turn(125, 60, 0.4, False, 2, 4, 1)
    xHand.on_for_rotations(-100, 1, block=True)
    xHand.on_for_rotations(-100, 2, block=False)
    straight(0.8, 25, 130, 4, 10)
    yHand.on_for_rotations(100, 1, block=False)
    xHand.on_for_rotations(100, 1.5)
    xHand.on_for_rotations(100, 1.5, block=False)
    straight(14, -40, 180, 0.25, 10)
    yHand.on_for_rotations(-100, 1.5, block=False)
    turn(208, 60, 0.25, False, 2, 4, 0.7)
    straight(70, 100, 220, 4, 10, shouldSpeedUp=False)
    yHand.reset()
    yHand.on_for_rotations(50, 0.72, brake=True)
    xHand.on_for_rotations(10, 0.01, brake=True)
def run4():
    setPosition(80, 200, 3)
    sleep(1)
    straight(77, 65, 74, 3, 5, motorStop=35)
    straight(30, 65, 38, 4.5, 5, shouldSpeedUp=False, motorStop=60)
    tankMovement.on_for_rotations(20, 20, 0.5, brake=True)
    turn(42, 60, 0.6, False, 2, 4, 0.3)
    tankMovement.on_for_rotations(10, 10, 0.2, brake=False)
    xHand.on_for_rotations(-50, 1.8)
    yHand.on_for_rotations(100, 0.6)
    straight(4, -80, 42, 3, 6, shouldSpeedUp=False)
    tankMovement.on_for_rotations(0, -100, 0.02)
    yHand.on_for_rotations(100, 2, block=False)
    tankMovement.on_for_rotations(0, -80, 0.95)
    straight(90, -100, -70, 10, 5, shouldSpeedUp=False)
    yHand.on_for_rotations(100, -3, block=False)
def run5():
    setPosition(-30, 24, 12)
    straight(27, 80, -30, 3, 5, motorStop=55)
    xHand.on_for_rotations(40, 0.07, block=False)
    yHand.on_for_rotations(60, 0.4, block=False)
    straight(50, 80, 1.5, 5, 6, shouldSpeedUp=False)
    xHand.on_for_rotations(50, 1.35, block=False)
    tankMovement.on_for_rotations(-100, -100, 0.15)
currentRun = "Run1"
#? Ez az a futás ami éppen ki van választva, ami alapból az egyes.
futas2Ujra = 0
futas5Ujra = 0
#? Ennél a 2 futásnál még az előtte lévő futásban felemeli a kart, ezért ha újra kell indítani akkor rosz helyen lehett,
#?  emiatt azt számoljuk és hogyha már nem elsőre indítjuk el, akkor előtte a 0-hoz képest felemeli annyira a kart amennyire kéne
yHand.on_for_rotations(75, 0.75)
yHand.on_for_rotations(-75, 0.75)
xHand.on_for_rotations(-75, 0.75)
xHand.on_for_rotations(75, 0.75)
#* Megnézi, hogy bármelyik kar kapar e
#* Itt lentebb egy gombal kettő akciót lehet kiválasztani, ezt mind a négy szélső gombal meg lehet csinálni. 
#* Ha egyszer megnyomod akkor az egyik akciót, ha mégegyszer akkor a másik akciót, aztán újra az elsőt választja ki.
def up(state):
    if state:
        return
    else: 
        if(currentRun == "Run2"):
            writeRun("Run5")  
        else:
            writeRun("Run1")      
def right(state):
    if state:
        return
    else:
        if(currentRun == "Run1"):
            writeRun("Run2")
        else:
            writeRun("Run1") 
def down(state):
    if state:
        return
    else: 
        if(currentRun == "Run3"):
            writeRun("Run4")
        else:
            writeRun("Run3")  
def left(state):
    if state:
        return
    else:
        if(currentRun == "Run5"):
            writeRun("Calibrate")
        else:
            writeRun("Run5")
def enter(state):
    if state:
        return
    else: 
        if(currentRun != None):
            startRun()
button.on_left = left
button.on_right = right
button.on_up = up
button.on_down = down
button.on_enter = enter
#? A gombok eseménykezelője
def writeGyro():
    angle = gs.angle
    leftside = "    "
    if(angle <= -100):
        leftside = "   "
    #* Ez a rész arra ügyel, hogy mindig pontosan középen legyen a kiírt érték

    console.text_at("   " +"G: %03d" % (angle) + leftside, column=2, row=1, reset_console=True, inverse=True)
    #? Kiírja a giroszkóp fordulatszámát, hogy látszódjon, hogyha "mászik"
def writeRun(futas):
    console.text_at(futas, column=2, row=3, reset_console=False, inverse=True)
    global currentRun
    currentRun = futas
#? Kiirja a jelenlegi futást, hogy látszódjon melyik van kiválasztva
running = False
def startRun():
    global running
    if(running == True):
        return
    running = True
    global currentRun
    global futas5Ujra
    global futas2Ujra
    if(currentRun == "none" or currentRun == None):
        return
    #* Megnézi, hogy valóban ki van e választva valami
    gs.reset()
    tankMovement.stop()
    sleep(0.1)
    #* Lenullázza a giroszkópot, hogy abszolútértékesen is lehessen számolni, és ne kelljen minden function elején megcsinálni
    if currentRun == "Calibrate":
        gyro.calibrate()
    if currentRun == "Run1":
        run1()
        futas5Ujra = 0
        futas2Ujra = 0
    if currentRun == "Run2":
        run2() 
    if currentRun == "Run3":
        run3()
    if currentRun == "Run4":
        run4()
        tankMovement.stop()
    if currentRun == "Run5":
        run5()
        tankMovement.stop()
        yHand.reset()
        xHand.reset()
        tankMovement.reset()
    #* Ezek maguk a futások, és hogyha megnyomod a fölső gombot, akkor megszakítja mindegyiket, ezt "Key Remapping" segítségével érjük el
    if(currentRun.replace("Run", "").isdigit() and currentRun.replace("Run", "") != "5"):
        currentRun = "Run" + str((int(currentRun.replace("Run", "")) + 1))
    else:
        currentRun = "Run1"
    #* Ha egy futás van kiválasztva, akkor a következő futást választja ki ami számrendben utána jön automatikusan
    running = False
    writeGyro()
previousGyro = 0
previousRun = "none"
rotations = getRotations()
tooLowColor = False
leds.animate_stop()
while True: 
    try:
        if(leftSensor.reflected_light_intensity < 2 or rightSensor.reflected_light_intensity < 2):
            tooLowColor = True
            print("ColorSensor value too low!")
            speaker.play_tone(frequency=1200, duration=0.1)
            leds.set_color("LEFT", "RED", 1)
            leds.set_color("RIGHT", "RED", 1)
            sleep(0.1)
            continue
        if(tooLowColor):
            sleep(1)
        print("Calibrating rotations...")
        try:
            leds.animate_cycle(('RED', 'YELLOW', 'AMBER'), duration=25, block=False)
            calibrate()
        except ZeroDivisionError:
            continue
    except KeyboardInterrupt:
        tankMovement.stop()
        tankMovement.reset()
        print("V: " + str(battery.measured_volts) + "\nD: " + str(wheelDiameter) + "\nPress enter to retry")
        if(button.wait_for_bump("enter", 2000)):
            continue
        else:
            rotationsText = open("rotations.txt", "r+")
            wheelDiameter = float(rotationsText.readline())
            rotationsText.close()
    else:
        rotationsText = open("rotations.txt", "w+")
        rotationsText.write(str(wheelDiameter))
        rotationsText.close()
    try:
        print("V: " + str(battery.measured_volts) + "\nD: " + str(wheelDiameter)+"\nPress down to skip")
        button.wait_for_bump("down", 5000)
    except KeyboardInterrupt:
        continue
    tankMovement.stop()
    tankMovement.reset()
    break

leds.animate_stop()
leds.set_color("LEFT", "GREEN", 1)
leds.set_color("RIGHT", "GREEN", 1)
speaker.tone([
        (400, 75, 10),  
        (600, 75, 10), 
        (800, 75, 10), 
        (1000, 75, 10),
        (1200, 75, 10) 
        ])
        
#? Plays a tone to let the operators know everything is loaded
while True:
    try: 
        button.process()
        #* A gombokat ellenőrzi
        if(gs.angle != previousGyro or currentRun != previousRun):
            #* Hogyha változott a giroszkóp értéke, vagy változott a kiírt futás, akkor ábrázolja
            writeGyro()
            if(currentRun and currentRun != "none"):
                console.text_at("R: " + currentRun, column=2, row=3, reset_console=False, inverse=True)
            previousGyro = gs.angle
            previousRun = currentRun
    except KeyboardInterrupt:
        tankMovement.stop()
        tankMovement.reset()
        running = False
        button.wait_for_released("up", 2000)
        speaker.play_tone(frequency=2500, duration=0.50)
        #* Ha a fölső gombot valaki megnyomja akkor megáll a robot.