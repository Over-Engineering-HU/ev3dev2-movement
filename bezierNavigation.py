from math import sin, cos, degrees, atan2, radians, sqrt, fmod
from time import time, sleep
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from ev3dev2.motor import LargeMotor, MoveTank, MoveSteering, MediumMotor
from ev3dev2.power import PowerSupply
#? Imports
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
battery = PowerSupply()
#? Defining the sensors of the robot.
gyroCorrection = 0.99174
#? The gyroscope of the robot counts 363 degrees in one rotation rather than 360, so we always multiply it by this constant
gyroOffset = 0
currentX = 0
currentY = 0
wheelDiameter = 17.5198
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
class Vector:
    def __init__(self, x, y):
        self.x = -x
        self.y = y
def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) **2 + (y2 - y1) ** 2)
def getPointOnBezier(controllPoints, t):
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
    if(n <= 1): return 1
    
    result = 1
    for i in range(2, n + 1):
        result *= i
    return result
def bezierLenght(controlPoints, n = 100):
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        previousPoint = getPointOnBezier(controlPoints, 0)
        
        for i in range(0, n):
            point = getPointOnBezier(controlPoints, t)
            totalLength += distance(previousPoint.x, previousPoint.y, point.x, point.y)
            #print(degrees(atan2(point.y - previousPoint.y, point.x - previousPoint.x)))
            t += dt
            previousPoint = point
        return totalLength
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
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False):
    """Make the robot go straight in a specified degree (cm)"""
    direction = sign(maxSpeed)
    minSpeed *= direction
    if(shouldSpeedUp == True):
        tankMovement.on(minSpeed, minSpeed)
    else:
        tankMovement.on(maxSpeed, maxSpeed)
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
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
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
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
def gotoXY(targetX, targetY, maxSpeed, minSpeed, sensitvity, margin = 4, speedUp =0.3, slowDown = 0.8, debug = False, motorStop = False, shouldSlow = True, shouldSpeed = True, curve = False):
    """Goes to the specified coordinates (distance from the two sides in cm) using trigonometry"""
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
    startDistance = distance
    startGsAngle = gsAngle()
    speedUp *= startDistance 
    slowDown *= startDistance 
    startX = currentX
    startY = currentY
    if(maxSpeed < 0): minSpeed *= -1
    if(curve == True):
        coordinates = findBigger(abs(targetX - currentX), abs(targetY - currentY))
        curveX = -(coordinates[0] * sign(targetX - currentX))
        curveY = -(coordinates[1] * sign(targetY - currentY))
        curveTargetAngle = (((degrees(atan2((targetX - (targetX - curveX)), (targetY - (targetY -curveY))))))) 
        print("Curve Target Angle: " + str(curveTargetAngle))
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(curve == True):
            if(curveY == 0):
                distanceDecimal = (1- abs((targetY - currentY) / (targetY - startY)))
                print("TargetY: " + str(targetY) + "\tCurrentY: " + str(currentY))
            if(curveX == 0):
                distanceDecimal = (1 - abs((targetX - currentX) / (targetX - startX)))
                print("TargetX: " + str(targetX) + "\tCurrentX: " + str(currentX)) 
            targetAngle = curveTargetAngle * distanceDecimal
            targetAngle += startGsAngle
            print(distanceDecimal)
        else:
            targetAngle = degrees(atan2(-(targetX - currentX), targetY - currentY))
        #targetAngle = shortest_angle(gsAngle(), targetAngle)
        if(maxSpeed < 0):
            targetAngle = ( targetAngle - 180)
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "\tY: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "\ttargetY: " + str(targetY))
            #print("Current Angle: " + str(gsAngle()) + "\tTarget Angle: " + str(round(targetAngle, 2)))
            #print("Left speed: " + str(leftm.speed) + "\tRight speed: " + str(rightm.speed))
            print("slowDwon: "+str(slowDown) + "\tspeedUp: "+str(speedUp))
            print("Predicted Distance: " + str(distance) + "\tStart Distance: " + str(startDistance))
            print("Target: " + str(targetAngle) + "\tCalculated: " + str(calculatedAngle) + "\tCurrent: " + str(gsAngle()))

        calculatedAngle = fmod(calculatedAngle, 360)
        calculatedAngle /= 3.6
        #* Ne forduljon meg a robot hátra menésnél

        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on(calculatedAngle, calculatedSpeed)
        sleep(0.01)

    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
    print("__________")
    print("D__O__N__E")
    print("__________")
def turn(angle, maxSpeed, sensitvity, relative = True, stopMargin = 2, minSpeed = 2, timeout = 2):
    angle = angle * -1
    #? Így megy a jó irányba, gyro meg van fordítva
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
def bezier(controllPoints, minSpeed, maxSpeed, sensitvity, margin=4, speedUp=0.3, slowDown = 0.8, motorStop=False, shouldSpeed= True, shouldSlow = True):
    targetX = controllPoints[len(controllPoints)-1].x
    targetY = controllPoints[len(controllPoints)-1].y
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    startRotations = rotations
    startGsAngle = gsAngle()
    distance = bezierLenght(controllPoints, 100)
    startDistance = distance
    #print(distance)

    previousPoint = getPointOnBezier(controllPoints, 0)
    previousDecimal = -1
    distanceDecimal = 0
    
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = getRotations() - startRotations
        distanceDecimal = (distance / startDistance)
        if(distanceDecimal > 1):
            break
        #print("Decimal: " + str(distanceDecimal))
        
        currentPoint = getPointOnBezier(controllPoints, distanceDecimal)
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        #print("TargetX: " + str(targetX) + "\tCurrentX: " + str(currentX))
        #print("TargetY: " + str(targetY) + "\tCurrentY: " + str(currentY))
        rotations = getRotations()
        if(previousDecimal < distanceDecimal):
            targetAngle = -degrees(atan2(currentPoint.y - previousPoint.y, currentPoint.x - previousPoint.x))
            targetAngle -= gsAngle()
            targetAngle *= -sign(targetAngle)
            targetAngle *= -sign(targetAngle)
            targetAngle -= startGsAngle
            targetAngle *= -1
        print(distanceDecimal)
        targetAngle = shortest_angle(gsAngle(), targetAngle)
        #print("Target angle: " + str(targetAngle))
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        #print("Calculated angle: " + str(calculatedAngle))
        #print("___")
        calculatedSpeed = calculateSpeed(distanceDecimal, 0, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distanceDecimal, shouldSpeedUp = shouldSpeed)
        if(sign(maxSpeed) == -1):
            calculatedAngle = -calculatedAngle
        #* Ne forduljon meg a robot hátra menésnél
        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on((calculatedAngle / 3.6), calculatedSpeed)
        previousPoint = currentPoint
        previousDecimal = distanceDecimal
        sleep(0.01)
    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
#* Functions having to do with movement of the robot
def calibrate():
    """Runs a short calibration script, that calculates how many cm is one rotation."""
    gyro.calibrate()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, 0, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -50, 0, 1.1, 5, False, False, False, True, True, 0, 0)
    print(wheelDiameter)
def futas1():
    setPosition(-44, 24, 18.25)
    yHand.on_for_rotations(80, 0.7, block=False)
    straight(28, 70, -41, 4.8, 3, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(-100, 0.5)
    xHand.on_for_rotations(70, -0.55)
    yHand.on_for_rotations(100, 0.6, block=False)
    straight(4.9, 20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(100, -0.65, block=False)
    yHand.on_for_rotations(100, 1)
    straight(0.8, -20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(90, -2.22, block=False)
    straight(25, -70, -40, 1.1, 5, False, False, False, True, True, 0, 0)
    straight(2.5, 40, -40, 1.1, 40)
    xHand.on_for_rotations(15, 0.7, block=False)
    yHand.on_for_rotations(30, 2.5, block=False)
    #bezier(controllPoints=[Vector(50, 36),Vector(53, 36),Vector(53.2, 70)],minSpeed=10,maxSpeed=70,sensitvity=1,speedUp=0.3,slowDown=0.8, motorStop=40)
    straight(22, 55, -20, 3.5, 4, motorStop=30)
    xHand.on_for_rotations(-100, 1, block=False)
    straight(35, 70, 5, 5, 30, motorStop=70)
    steeringMovement.on_for_rotations(0, 30, 0.5)
    tankMovement.on_for_rotations(-20, -20, 0.03)
    yHand.on_for_rotations(-100, 1, block=True)
    yHand.on_for_rotations(-100, 1.5, block=False)
    """sleep(0.2)
    gyro.reset()
    setPosition(0, 180, 97)
    sleep(0.2)"""
    straight(distance=10.5,maxSpeed=-50,targetAngle=5,sensitivity=0,minSpeed=4,speedingUp=0.3,slowingDown=0.8)
    xHand.on_for_rotations(100, 1.2)
    raiseSpeed = 100
    raiseHeight = 0.7
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
    tankMovement.on_for_rotations(-70, -70, 0.4)
def futas2():
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
    turn(-180, 50, 0.55, relative=False, timeout=0.8)
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
    straight(4.5, 20, 215, 6, 1)
    turn(180, 60, 0.55, False, 2, 2, 0.45)
    straight(22, 30, 180, 6, 10)
    straight(0.04, -10, 180, 9, 1)
    xHand.on_for_rotations(40, 1.2, True, True)
    straight(10, -100, 180, 5, 20)
    turn(245, 50, 0.55, False, 2, 2, 0.4)
    straight(29, 100, 252, 5, 30, slowingDown=0.05, motorStop=100)
    straight(100, 100, 273, 5, 10, slowingDown=0.05, speedingUp=0.95, motorStop=100, shouldSpeedUp=False)
def futas3():
    setPosition(0, 200, 18)
    straight(27, 70, 0, 3.5, 20, motorStop=25)
    straight(13.5, 25, 0, 3.5, 15, shouldSpeedUp=False)
    straight(11, -60, 0, 1.2, 20)
    turn(40, 60, 0.4, False, 2, 5, 0.8)
    xHand.on_for_rotations(-100, 3, block=False)
    straight(12, 60, 40, 3.5, 20, motorStop=55)
    xHand.on_for_rotations(100, 0.4, block=False)
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
        straight(16, 60, -42, 4, 10)
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
    yHand.on_for_rotations(-100, 1, block=False)
    turn(208, 60, 0.25, False, 2, 4, 0.7)
    straight(70, 100, 208, 4, 10, shouldSpeedUp=False)
def futas4():
    gyro.reset()
    setPosition(80, 200, 3)
    yHand.on_for_rotations(50, 0.535)
    xHand.on_for_rotations(1, 0.1)
    sleep(1)
    straight(74, 65, 74, 3, 5, motorStop=55)
    straight(30, 65, 38, 4.5, 5, shouldSpeedUp=False, motorStop=60)
    tankMovement.on_for_rotations(20, 20, 0.5, brake=True)
    turn(42, 60, 0.6, False, 2, 4, 0.3)
    tankMovement.on_for_rotations(10, 10, 0.2, brake=False)
    xHand.on_for_rotations(-50, 1.8)
    yHand.on_for_rotations(100, 0.5)
    straight(4, -80, 42, 3, 6, shouldSpeedUp=False)
    tankMovement.on_for_rotations(0, -100, 0.025)
    yHand.on_for_rotations(100, 3, block=False)
    tankMovement.on_for_rotations(0, -80, 0.95)
    straight(90, -100, -70, 10, 5, shouldSpeedUp=False)
def futas5():
    gyro.reset()
    setPosition(-30, 24, 12)
    straight(26, 80, -30, 3, 5, motorStop=55)
    xHand.on_for_rotations(40, 0.07, block=False)
    yHand.on_for_rotations(60, 0.4, block=False)
    straight(43, 80, 2, 5, 6, shouldSpeedUp=False)
    xHand.on_for_rotations(50, 1.35, block=False)
    tankMovement.on_for_rotations(-100, -100, 0.15)
rotations = getRotations()
dist = 83
wheelDiameter = 17.3450
if(not True):
    calibrate()
elif(False == True):
    wheelDiameter = 2.1580075734754408290035266817997 * battery.measured_volts
print(battery.measured_volts)
print(wheelDiameter)
#7.547666 = 17.3822
#8.036733 = 17.6648
#8.1724 = 17.5661
#8.3884 = 17.3119
wheelDiameter = optimizeFloat(wheelDiameter)
input("Start? ")
setPosition(0, 40, 40)
gyro.reset()
leftMotor.reset()
rightMotor.reset()
"""try:
    setPosition(0, 0, 0)
    gotoXY(0, 40, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=False, debug=True)
    gotoXY(0, 0, -50.00, 5.00, 1, 4, 0.30, 0.7, curve=False, debug=True)
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
exit("this wont even work")"""
def handTest():
    xHand.on_for_rotations(100, 3)
    xHand.on_for_rotations(-100, 3)
    xHand.on_for_rotations(100, 0.8)
    xHand.on_for_rotations(-100, 0.8)
try:
    print(battery.measured_volts)
    speed = 40
    tankMovement.on_for_rotations(speed, speed, 0.5, block=False    )
    sleep(0.6)
    print(battery.measured_volts)
    sleep(1)
    #futas1()   
    print("meow")
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
    xHand.stop()
    yHand.stop()
    xHand.reset()
    yHand.reset()
    print("Exited the program")
tankMovement.stop()
tankMovement.reset()
xHand.stop()
yHand.stop()
xHand.reset()
yHand.reset()
exit("this wont even work")
controlPoints = [Vector(0, 0), Vector(25, 25), Vector(75, 25)]
setPosition(0, 0, 0)
controlPoints = [Vector(75, 25), Vector(25, 25), Vector(0, 0)]
setPosition(0, 75, 25)
try:
    bezier(controlPoints, 10, 70, 0.5, 4)
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
exit("meow")
try:
    gotoXY(120, 80, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, motorStop=50)
    gotoXY(150, 55, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False, motorStop=50)
    gotoXY(37.5, 40.5, 50.00, 5.00, 0.8, 2, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False)
    bestSong = "https://youtube.com/shorts/MS2ZXbbZp3o?feature=share"
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
    print("Exited the program")
tankMovement.stop()
tankMovement.reset()