#!/usr/bin/env micropython

##! Code used for on the spot championships in 2023 Long Beach


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
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
#? Imports
speaker = Sound()
console = Console()
console.set_font(font='Lat15-Terminus32x16', reset_console=False)
button = Button()
leds = Leds()
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
wheelDiameter = 17.5
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
system("chmod +x loadCtrlC.sh")
system("sh ../miau/loadCtrlC.sh")
#? Remapping the "up" key to the ctrl-c key combination
#? Inicializálja a gombokat, a konzolt, a girót, és a hangszoórót
class port:
    def __init__(self, id, type):
        self.id = id
        self.type = type
ports = [port("in2", "GyroSensor"), port("in3", "ColorSensor"), port("in4", "ColorSensor"), port("outA", "MediumMotor"), port("outD", "MediumMotor"), port("outB", "LargeMotor"), port("outC", "LargeMotor")]
badPorts = []
console.set_font(font='Lat15-Fixed15.psf.gz', reset_console=False)
leds.animate_flash('GREEN', sleeptime=0.75, duration=20, block=False)
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
gs.calibrate()
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
        if(stopOnLine == True):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                tankMovement.stop(None, False)
                break
        if(calibrate):
             if(leftSensor.reflected_light_intensity <= 92 or rightSensor.reflected_light_intensity <= 92):
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
    dist = 16
    gyro.calibrate()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, 0, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -100, 0, 5, 20, False, False, False, True, True, 0, 0)
    rotationsText = open("rotations.txt", "w+")
    rotationsText.write(str(wheelDiameter))
    rotationsText.close()
def run1(height):
    yHand.on_for_rotations(-100, 1, False, True)
    yHand.reset()
    yHand.on_for_rotations(70, height, True)
    straight(87, 70, 0, 3.5, 5, drift=-1, speedingUp=0.1)
    xHand.on_for_rotations(100, 0.2)
    xHand.on_for_rotations(-100, 0.2, block = False)
    #straight(95, -100, 0, 3.5, 25, shouldSlowDown=False, shouldSpeedUp=False)
    yHand.on_for_rotations(-100, height, True)
def run2():
    yHand.on_for_rotations(-100, 1, False, True)
    yHand.reset()
    yHand.on_for_rotations(30, 1.2, True, False)
    setPosition(-29, 33.5, 19.5)
    
def run3():
    yHand.reset()
    setPosition(0, 200, 18)
    
def run4():
    yHand.on_for_rotations(50, 0.85, brake=True)
    xHand.on_for_rotations(10, 0.01, brake=True)
    setPosition(80, 200, 3)
    
def run5():

    setPosition(-30, 24, 12)
    straight(35.3, 80, -30, 3, 5, motorStop=55)
currentRun = "Run1"
currentHeight = 0
heights = [0, 2.3]
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
    global currentHeight
    if state:
        return
    else:
        if(currentHeight != 1):
            currentHeight += 1
        else:
            currentHeight = 0
def down(state):
    if state:
        return
    else: 
        pass
def left(state):
    global currentHeight
    if state:
        return
    else:
        if(currentHeight != 0):
            currentHeight -= 1
        else:
            currentHeight = 1
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
    console.text_at(currentHeight, column=2, row=3, reset_console=False, inverse=True)
    global currentRun
    currentRun = futas
#? Kiirja a jelenlegi futást, hogy látszódjon melyik van kiválasztva
running = False
def startRun():
    global currentHeight
    try:
        global running
        if(running == True):
            return
        running = True
        global currentRun
        if(currentRun == "none" or currentRun == None):
            return
        #* Megnézi, hogy valóban ki van e választva valami
        gs.reset()
        tankMovement.stop()
        sleep(0.1)
        #* Lenullázza a giroszkópot, hogy abszolútértékesen is lehessen számolni, és ne kelljen minden function elején megcsinálni
        #speaker.play_file('run.wav', play_type=speaker.PLAY_NO_WAIT_FOR_COMPLETE)
        run1(heights[currentHeight])
        
        tankMovement.reset()
        tankMovement.stop()
        #* Ezek maguk a futások, és hogyha megnyomod a fölső gombot, akkor megszakítja mindegyiket, ezt "Key Remapping" segítségével érjük el
        #* Ha egy futás van kiválasztva, akkor a következő futást választja ki ami számrendben utána jön automatikusan
        running = False
        writeGyro()
        #speaker.play_file('build.wav', play_type=speaker.PLAY_NO_WAIT_FOR_COMPLETE)
    except:
        tankMovement.stop()
        tankMovement.reset()
        leftMotor.reset()
        rightMotor.reset()
        running = False
        button.wait_for_released("up", 2000)
        speaker.play_tone(frequency=2500, duration=0.50)
        #speaker.play_file('build.wav', play_type=speaker.PLAY_NO_WAIT_FOR_COMPLETE)
        #* Ha a fölső gombot valaki megnyomja akkor megáll a robot.
previousGyro = 0
previousRun = "none"
rotations = getRotations()
tooLowColor = False
leds.animate_stop()
#? Plays a tone to let the operators know everything is loaded
leds.animate_stop()
leds.all_off()
#speaker.play_file('build.wav', play_type=speaker.PLAY_WAIT_FOR_COMPLETE)
while True:
    try: 
        button.process()
        #* A gombokat ellenőrzi
        if(not running and gs.angle != previousGyro or currentRun != previousRun):
            #* Hogyha változott a giroszkóp értéke, vagy változott a kiírt futás, akkor ábrázolja
            writeGyro()
            if(currentRun and currentRun != "none"):
                console.text_at("H: " + str(currentHeight), column=2, row=3, reset_console=False, inverse=True)
            previousGyro = gs.angle
            previousRun = currentRun
    except KeyboardInterrupt or ValueError:
        tankMovement.stop()
        tankMovement.reset()
        leftMotor.reset()
        rightMotor.reset()
        running = False
        button.wait_for_released("up", 2000)
        speaker.play_tone(frequency=2500, duration=0.50)
        #speaker.play_file('build.wav', play_type=speaker.PLAY_NO_WAIT_OR_COMPLETE)
        continue
        #* Ha a fölső gombot valaki megnyomja akkor megáll a robot.