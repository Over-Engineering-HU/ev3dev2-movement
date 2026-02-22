#!/usr/bin/env micropython

#! Used for actual matches

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
from ev3dev2.console import Console
from ev3dev2.button import Button
from os import system
#? Imports
battery = PowerSupply()
#? Defining the sensors of the robot.
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
console = Console()
console.set_font(font='Lat15-Terminus32x16', reset_console=False)
button = Button()
#? Inicializálja a gombokat, a konzolt, a girót, és a hangszoórót
class port:
    def __init__(self, id, type):
        self.id = id
        self.type = type
ports = [port("in2", "GyroSensor"), port("in3", "ColorSensor"), port("in4", "ColorSensor"), port("outA", "MediumMotor"), port("outD", "MediumMotor"), port("outB", "LargeMotor"), port("outC", "LargeMotor")]
badPorts = []
console.set_font(font='Lat15-Fixed15.psf.gz', reset_console=False)
for aPort in ports:
    sleep(0.05)
    try:
        eval(aPort.type + "('"+aPort.id+"')")
    except:
        print("|X| " + aPort.type + " " + aPort.id)
        sleep(0.2)
        badPorts.append(aPort)
    else:
        print("|✓| " + aPort.type + " " + aPort.id,)
if(len(badPorts) != 0):
    print("Connect all ports!")
    for badPort in badPorts:
        print(aPort.type + " " + aPort.id + " |x|")
while len(badPorts) != 0:
    sleep(0.05)
    for badPort in badPorts:
        try:
            eval(badPort.type + "('"+badPort.id+"')")

        except:
            pass
        else:
            print(aPort.type + " " + aPort.id + " connected")
            badPorts.remove(badPort)
            sleep(0.2)
print("Press enter to skip.")
button.wait_for_bump("enter", 3000)
console.set_font(font='Lat15-Terminus32x16', reset_console=True)
#? Diagnostincs Complete
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
class Vector:
    def __init__(self, x, y):
        self.x = -x
        self.y = y
def optimizeFloat(num):
    return round(float(num), 4)
def sign(num):
    """Returns the sign of a number, and returns a 1 if the number is 0"""
    if(num == 0):
        return 1
    return(num / abs(num))
def findBigger(num1, num2):
    """Subtracts the smallest number from all numbers in a vector and returns it."""
    smallest = min(num1, num2)
    num1 -= smallest
    num2 -= smallest
    return [num1, num2]
#* Functions that are only mathematical, and don't have any direct connection to the robot
def gsAngle():
    """Returns the angle of the gyroscope, (this also takes into account the offset and the correction of the gyroscope)"""
    return fmod(optimizeFloat((gyro.angle + gyroOffset) * gyroCorrection), 360)
def getRotations():
    """Returns the current rotation of the two large motors"""
    return optimizeFloat((leftMotor.rotations + rightMotor.rotations) / 2 * wheelDiameter)
def setPosition(angle, x, y):
    """Sets the gyroscope angle and the x and y coordinates of the robot."""
    global gyroOffset
    global currentX
    global currentY
    gyroOffset = angle
    currentX = x
    currentY = y
def shortest_angle(angle, target_angle):
    # Calculate the absolute difference between the angles
    diff = target_angle - angle
    # Check if the difference is more than 180 degrees
    if abs(diff) > 180:
        # Recalculate the target angle with the shortest path
        target_angle -= 360
        target_angle *= sign(diff) 
    return target_angle
def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance, shouldSpeedUp):
    """This function calculates the speed, for linearly speeding up and slowing down."""
    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target
    returnSpeed = maxSpeed
    
    if(deltaDistance < speedUp and shouldSpeedUp == True):
        #* Ha a kezdet óta a fordulatok száma még kisebb annál a cél-fordulat számnál amit megadtunk, akkor tovább gyorsul
        returnSpeed = (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
        #~   [              0 és 1 közötti szám           ]   maximum elérhető érték (nem számítjuk a minimum sebességet) + alap sebesség
    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        if(motorStop != False):
            minSpeed = motorStop
        #* Ha ez be van kapcsolva, akkor csak egy adott sebességig lassul, és utána bekapcsolva hagyja a motort
        returnSpeed = maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~               [                        1 és 0 közötti szám                      ]    legalacsonyabb sebessége a minimum érték lehet
    if(abs(returnSpeed) > 100): returnSpeed = sign(returnSpeed) * 100
    
    return round(float(returnSpeed), 2)
#* Functions that don't directly move the robot
def moveRobotOnLine(motor, szinSzenzor, minFeny, maxSebesseg, KP):
    returnSpeed = (minFeny - szinSzenzor.reflected_light_intensity) * KP
    #& Egyik oldali motor sebességének kiszámolása, egy célérték (minLight) és egy érzékenység (KP) alapján
    if(abs(returnSpeed) > maxSebesseg):            
        returnSpeed = maxSebesseg * (returnSpeed / abs(returnSpeed))
    #* Semmiképp se legyen az sebesség magasabb a megadott maximum sebességnél
    motor.on(returnSpeed)
    #* Elindítja a motort a kiszámolt sebességgel
    return round(float(returnSpeed), 2)
    #* Visszaadja a sebességet, hogy meg lehesen nézni hogy, 0, és mindkét motornak 0 lett a sebessége, akkor leáll a program.
def goOnLine(KP, maxIdo, maxSebesseg, minimumFeny):
    """Makes to robot perpendicularly go on a line, works with 2 sensors only."""
    elozoIdo = time()
    #? Vonalra állás kezdetének időpotját lementi
    while True:            
        elteltIdo = time() - elozoIdo
        #* Fordulás óta eltelt idő
        if(moveRobotOnLine(leftMotor, leftSensor, minimumFeny, maxSebesseg, KP) == 0 and moveRobotOnLine(rightMotor, rightSensor, minimumFeny, maxSebesseg, KP) == 0):
            tankMovement.stop(None, False)
            break
        #* Elindítja a motorokat a funkciókkal és megnézi, hogy mindkettő 0
        #* ha igen akkor leállítja a programot, mert elivleg sikeresen ráállt a vonalra
        if(elteltIdo >= maxIdo):
            tankMovement.stop(None, False)
            break
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False, timeout = False):
    """Make the robot go straight in a specified degree (cm)"""
    direction = sign(maxSpeed)
    startTime = time()
    minSpeed *= direction
    if(shouldSpeedUp == True):
        tankMovement.on(minSpeed, minSpeed)
    else:
        tankMovement.on(maxSpeed, maxSpeed)
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    timesBad = 0
    if(speedingUp == False):
        speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
        if(speedingUp > (2 * wheelDiameter)):
            speedingUp = (2 * wheelDiameter)
    if(slowingDown == False):
        slowingDown = distance * 0.6
        if(slowingDown > (2*wheelDiameter)):
            slowingDown = (2 * wheelDiameter)
    while abs(getRotations() - startRotations) <= distance:
        if(timeout and startTime + timeout <= time()):
            break
        if(calibrate == False):
            currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
            currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
            rotations = getRotations()
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                tankMovement.stop(None, False)
                break
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed+1)) * 2
        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5
        calculatedSensitivity = optimizeFloat(sensitivity / sensitivityMultiplier)
        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység
            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél
            calculatedAngle /= 3.6
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni
            steeringMovement.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
        """else:
            if(timesBad + correctMargin != timesGood):
                steeringMovement.on((-previousAngle * direction), calculatedSpeed)
            else:
                steeringMovement.on((drift * direction), calculatedSpeed)
            timesGood += 1
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
            print("Sensitivity: " + str(sensitivityMultiplier))"""
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
    turnConstant = (fordulatszam-angle)
    stopMargin *= sign(turnConstant)
    startTime = time()
    while gsAngle() != fordulatszam - angle:
        if(time() - startTime >= timeout): break
        if(turnConstant - stopMargin <= gsAngle() <= turnConstant + stopMargin and hasStopped == False and stopMargin != 0):
            hasStopped = True
            tankMovement.stop()
            continue
        calculatedSpeed = (turnConstant - gsAngle()) * sensitvity
        if(calculatedSpeed > maxSpeed / 3.5):
            calculatedSpeed = maxSpeed
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(maxSpeed) * sign(calculatedSpeed))
        if(abs(calculatedSpeed) < minSpeed): calculatedSpeed = (abs(minSpeed) * sign(calculatedSpeed))
        tankMovement.on(-calculatedSpeed, calculatedSpeed)
    tankMovement.stop()
#* Functions having to do with movement of the robot
def calibrate():
    """Runs a short calibration script, that calculates how many cm is one rotation."""
    dist = 83
    gyro.calibrate()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, 0, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -100, 0, 5, 20, False, False, False, True, True, 0, 0)
    rotationsText = open("rotations.txt", "w+")
    rotationsText.write(str(wheelDiameter))
    rotationsText.close()
def run1():
    yHand.on_for_rotations(-100, 1, False, True)
    yHand.reset()
    setPosition(-44, 24, 18.25)
    yHand.on_for_rotations(80, 0.7, block=False)
    straight(29.5, 80, -39.7, 4.8, 3.2, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(-100, 0.5)
    xHand.on_for_rotations(70, -0.55)
    yHand.on_for_rotations(100, 0.6, block=False)
    straight(4.9, 20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(100, -0.65, block=False)
    yHand.on_for_rotations(100, 1)
    straight(0.8, -20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(90, -2.22, block=False)
    straight(26, -70, -40, 4, 5, False, False, False, True, True, 0, 0)
    straight(2.5, 40, -40, 4, 40)
    xHand.on_for_rotations(15, 0.7, block=False)
    yHand.on_for_rotations(30, 2.5, block=False)
    straight(17, 55, -20, 3.5, 4, motorStop=30)
    xHand.on_for_rotations(-80, 1, block=False)
    straight(35, 70, 5.5, 5, 30, motorStop=70)
    steeringMovement.on_for_rotations(0, 30, 0.3)
    tankMovement.on_for_rotations(-20, -20, 0.01)
    yHand.on_for_rotations(-100, 1.5, block=True)
    yHand.on_for_rotations(-100, 1, block=False)
    straight(distance=16.5,maxSpeed=-50,targetAngle=5,sensitivity=5,minSpeed=4,speedingUp=0.3,slowingDown=False)
    xHand.on_for_rotations(100, 1.4)
    raiseSpeed = 100
    raiseHeight = 0.85
    yHand.reset()
    yHand.on_for_rotations(100, 0.3)
    for _ in range(0,4):
        yHand.reset()
        yHand.on_for_rotations(raiseSpeed, raiseHeight)
        sleep(0.08)
        yHand.reset()
        yHand.on_for_rotations(-raiseSpeed, raiseHeight, block=True)
    xHand.on_for_rotations(-60, 0.5, False, False)
    turn(-85, 70, 0.5, False, 10, 2, 0.5)
    xHand.on_for_rotations(100, 0.5, False, False)
    straight(distance=9,maxSpeed=40,targetAngle=-87,sensitivity=0.8,minSpeed=5,speedingUp=0.5,slowingDown=0.3)
    turn(-85, 70, 0.4, False, 10, 1, 1)
    straight(distance=23,maxSpeed=-55,targetAngle=-85,sensitivity=7,minSpeed=30,speedingUp=0.5)
    xHand.on_for_rotations(-100, 0.5, False, False)
    straight(distance=12,maxSpeed=40,targetAngle=-90,sensitivity=0.8,minSpeed=5,speedingUp=0.5)
    straight(distance=15,maxSpeed=-75,targetAngle=-90,sensitivity=4,minSpeed=30,speedingUp=0.5)
    straight(distance=13,maxSpeed=75,targetAngle=-90,sensitivity=5,minSpeed=10,speedingUp=0.5)
    turn(8, 40, 0.3, False, 10, 2, 1)
    xHand.on_for_rotations(100, 0.25, False, False)
    straight(distance=75,maxSpeed=-70,targetAngle=-30,sensitivity=2,minSpeed=5,speedingUp=0.5, shouldSlowDown=False)
def run2():
    yHand.on_for_rotations(-100, 1, False, True)
    yHand.reset()
    yHand.on_for_rotations(30, 1.2, True, False)
    setPosition(-29, 33.5, 19.5)
    #27.5
    straight(90, 70, -26.5, 4.5, 20, drift=2)
    turn(-90, 30, 0.45, relative=False, timeout=0.6)
    straight(10, 40, -90, 4.5, 25, motorStop=30)
    turn(-98, 30, 0.45, relative=False, timeout=0.6)
    yHand.on_for_rotations(-45, 1.2, True, False)
    straight(5, 40, -98, 5, 25)
    straight(8, 30, -98, 6.5, 25, shouldSpeedUp=False)
    straight(0.22, -10, -90, 4, 5)
    turn(-78, 15, 0.6, relative=False, timeout=0.6)
    yHand.on_for_rotations(100, 0.8, True, False)
    turn(-180, 20, 0.55, relative=False, timeout=1.4)
    straight(18, -70, -180, 4, 30, motorStop=-70, timeout=2)
    sleep(0.1)
    gyro.reset()
    setPosition(180, 106, 112)
    sleep(0.15)
    straight(7.5, 40, 180, 1, 25)
    straight(18.7, 50, 231.5, 5, 25, motorStop=33)
    yHand.on_for_rotations(60, 1.4, True, False)
    straight(25, 33, 185, 10, 3, shouldSpeedUp=False)
    straight(13, -30, 310.5, 3.6, 20)
    yHand.on_for_rotations(-70, 2.2, True, False)
    turn(207, 30, 0.6, False, 2, 2, 0.6)
    straight(20, 15, 210, 8, 6, True, False, shouldSlowDown=False, shouldSpeedUp=False)
    straight(4.9, 20, 260, 6, 8, shouldSlowDown=False, shouldSpeedUp=False)
    turn(178, 60, 0.6, False, 2, 2, 0.45)
    straight(22, 30, 178, 10, 10, slowingDown=0.5, motorStop=0)
    straight(0.12, -10, 178, 9, 1)
    xHand.on_for_rotations(40, 1.2, True, True)
    straight(13.5, -100, 180, 5, 20)
    turn(245, 50, 0.55, False, 2, 2, 0.4)
    straight(23, 100, 250, 5, 30, slowingDown=0.05, motorStop=100)
    straight(70, 100, 279, 5, 10, slowingDown=0.05, speedingUp=0.95, shouldSpeedUp=False)
def run3():
    yHand.reset()
    setPosition(0, 200, 18)
    straight(37, 70, 0, 3.5, 20, motorStop=20)
    straight(10, 30, 0, 3.5, 15, shouldSpeedUp=False)
    straight(16, -60, 0, 1.2, 20)
    turn(34, 60, 0.4, False, 2, 5, 0.8)
    xHand.on_for_rotations(-100, 3, block=False)
    straight(25, 60, 32.5, 3.5, 20, motorStop=35)
    xHand.on_for_rotations(100, 0.15, block=False)
    straight(3.35, 40, 62, 7.2, 10, shouldSpeedUp=False, motorStop=40)
    straight(30, 80, 54, 8, 10, shouldSpeedUp=False, motorStop=40)
    straight(20.5, 34.5, 46, 8, 2, shouldSpeedUp=False)
    xHand.on_for_rotations(35, 0.75)
    xHand.on_for_rotations(35, 2.25, block=False)
    straight(2, -60, 44, 6, 10)
    yHand.on_for_rotations(100, 2, block=False)
    straight(13.7, -80, 44, 6, 10,shouldSpeedUp=False) 
    xHand.on_for_rotations(100, 0.5, block=False)
    sleep(0.05)
    yHand.on_for_rotations(-100, 2, block=False)
    turn(-42, 60, 0.42, False, 2, 4, 0.85)
    straight(16, 80, -42, 6, 10, motorStop=80)
    tankMovement.on_for_rotations(80, 80, 1.5)
    xHand.on_for_rotations(-100, 2, block=False)
    for _ in range(0,3):
        straight(1, -100, -42, 0, 10, motorStop=-100)
        xHand.on_for_rotations(-100, 0.8, block=False)
        straight(1.2, -100, -42, 0, 10, shouldSpeedUp=False)
        sleep(0.2)
        xHand.on_for_rotations(100, 0.8, block=False)
        straight(20, 80, -42, 4, 10, shouldSlowDown=False)
        sleep(0.3)
    xHand.on_for_rotations(100, 1.5, block=False)
    straight(34, -70, -49.5, 4, 10, motorStop=0)
    straight(1, 20, -49.5, 4, 10)
    turn(125, 60, 0.4, False, 2, 4, 1)
    xHand.on_for_rotations(-100, 1, block=True)
    xHand.on_for_rotations(-100, 2, block=False)
    straight(0.8, 25, 130, 4, 10)
    yHand.on_for_rotations(100, 1.5, block=False)
    xHand.on_for_rotations(100, 1.5)
    xHand.on_for_rotations(100, 1.5, block=False)
    straight(14, -40, 180, 0.25, 10)
    yHand.on_for_rotations(-100, 1.5, block=False)
    turn(208, 60, 0.25, False, 2, 4, 0.7)
    straight(70, 100, 230, 4, 10, shouldSpeedUp=False)
    yHand.reset()
    xHand.on_for_rotations(10, 0.01, brake=True)
def run4():
    yHand.on_for_rotations(50, 0.85, brake=True)
    xHand.on_for_rotations(10, 0.01, brake=True)
    setPosition(80, 200, 3)
    straight(77.3, 65, 74, 3, 5, motorStop=35)
    straight(30, 65, 37, 5.5, 5, shouldSpeedUp=False, motorStop=20)
    tankMovement.on_for_rotations(20, 20, 0.2, brake=True)
    turn(42, 60, 0.6, False, 2, 4, 0.3)
    tankMovement.on_for_rotations(10, 10, 0.2, brake=False)
    xHand.on_for_rotations(-50, 1.8)
    yHand.on_for_rotations(100, 0.1)
    straight(2, -80, 42, 3, 6, shouldSpeedUp=False)
    straight(1, -80, 42, 0, 6, shouldSpeedUp=False)
    yHand.on_for_rotations(100, 0.9, block=True)
    yHand.on_for_rotations(100, 1.2, block=False)
    tankMovement.on_for_rotations(0, -80, 0.95)
    straight(75, -100, -70, 10, 5, shouldSpeedUp=False, shouldSlowDown=False)
    yHand.on_for_rotations(100, -3, block=False)
def run5():
    setPosition(-30, 24, 12)
    straight(35.3, 80, -30, 3, 5, motorStop=55)
    xHand.on_for_rotations(40, 0.1, block=False)
    yHand.on_for_rotations(60, 0.4, block=False)
    straight(50, 80, 1.5, 6, 5, shouldSpeedUp=False)
    xHand.on_for_rotations(50, 10, block=False)
    tankMovement.on_for_rotations(-100, -100, 0.065)
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
    try:
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
            xHand.stop()
            xHand.reset()
            futas5Ujra = 0
            futas2Ujra = 0
        if currentRun == "Run2":
            run2() 
            xHand.stop()
            xHand.reset()
        if currentRun == "Run3":
            run3()
            xHand.stop()
            xHand.reset()
        if currentRun == "Run4":
            run4()

        if currentRun == "Run5":
            run5()
            yHand.reset()
            xHand.reset()
        tankMovement.reset()
        tankMovement.stop()
        #* Ezek maguk a futások, és hogyha megnyomod a fölső gombot, akkor megszakítja mindegyiket, ezt "Key Remapping" segítségével érjük el
        if(currentRun.replace("Run", "").isdigit() and currentRun.replace("Run", "") != "5"):
            currentRun = "Run" + str((int(currentRun.replace("Run", "")) + 1))
        elif(currentRun.replace("Run", "") == "5"):
            currentRun="Run5"
        else:
            currentRun = "Run1"
        #* Ha egy futás van kiválasztva, akkor a következő futást választja ki ami számrendben utána jön automatikusan
        running = False
        writeGyro()
    except:
        tankMovement.stop()
        tankMovement.reset()
        leftMotor.reset()
        rightMotor.reset()
        running = False
        button.wait_for_released("up", 2000)
        #speaker.play_tone(frequency=2500, duration=0.50)
        #* Ha a fölső gombot valaki megnyomja akkor megáll a robot.
previousGyro = 0
previousRun = "none"
rotations = getRotations()
tooLowColor = False

while True: 
    try:
        if(leftSensor.reflected_light_intensity < 2 or rightSensor.reflected_light_intensity < 2):
            tooLowColor = True
            print("ColorSensor value too low!")
            sleep(0.1)
            continue
        if(tooLowColor):
            sleep(1)
        print("Calibrating rotations...")
        try:
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

#? Plays a tone to let the operators know everything is loaded
while True:
    try: 
        button.process()
        #* A gombokat ellenőrzi
        if(not running and gs.angle != previousGyro or currentRun != previousRun):
            #* Hogyha változott a giroszkóp értéke, vagy változott a kiírt futás, akkor ábrázolja
            writeGyro()
            if(currentRun and currentRun != "none"):
                console.text_at("R: " + currentRun, column=2, row=3, reset_console=False, inverse=True)
            previousGyro = gs.angle
            previousRun = currentRun
    except KeyboardInterrupt or ValueError:
        tankMovement.stop()
        tankMovement.reset()
        leftMotor.reset()
        rightMotor.reset()
        running = False
        button.wait_for_released("up", 2000)
        continue
        #speaker.play_tone(frequency=2500, duration=0.50)
        #* Ha a fölső gombot valaki megnyomja akkor megáll a robot.