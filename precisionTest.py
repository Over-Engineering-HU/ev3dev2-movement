from ev3dev2.motor import MoveTank, LargeMotor, MoveSteering
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from time import time
#? Mindent beimportálunk
wheelDiameter = 1
leftm = LargeMotor("outB")
rightm = LargeMotor("outC")
m = MoveTank("outB", "outC")
s = MoveSteering("outB", "outC")
gs = GyroSensor("in2")
gs.reset()
rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")
def getRotations():
    return (leftm.rotations + rightm.rotations) / 2 * wheelDiameter
def sign(num):
    if(num == 0):
        return 1
    return(num / abs(num))
def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance):
    """This function calculates the speed, for linearly speeding up and slowing down."""

    deltaDistance = abs(abs(currentDistance) - startDistance)


    if(deltaDistance < speedUp):
            #* Ha a kezdet óta a fordulatok száma még kisebb annál a cél-fordulat számnál amit megadtunk, akkor tovább gyorsul
            return (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
            #~   [              0 és 1 közötti szám           ]   maximum elérhető érték (nem számítjuk a minimum sebességet) + alap sebesség

    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        if(motorStop != False):
            minSpeed = motorStop
        #* Ha ez be van kapcsolva, akkor csak egy adott sebességig lassul, és utána bekapcsolva hagyja a motort
        return maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~               [                        1 és 0 közötti szám                      ]    legalacsonyabb sebessége a minimum érték lehet

    else:
        return maxSpeed
def raallSzog(motor, szinSzenzor, minFeny, maxSebesseg, KP):
    sebesseg = (minFeny - szinSzenzor.reflected_light_intensity) * KP
    #& Egyik oldali motor sebességének kiszámolása, egy célérték (minLight) és egy érzékenység (KP) alapján

    if(abs(sebesseg) > maxSebesseg):            
        sebesseg = maxSebesseg * (sebesseg / abs(sebesseg))
    #* Semmiképp se legyen az sebesség magasabb a megadott maximum sebességnél

    motor.on(sebesseg)
    #* Elindítja a motort a kiszámolt sebességgel

    return sebesseg
    #* Visszaadja a sebességet, hogy meg lehesen nézni hogy, 0, és mindkét motornak 0 lett a sebessége, akkor leáll a program.
def raall(KP, maxIdo, maxSebesseg, minimumFeny):
    elozoIdo = time()
    #? Vonalra állás kezdetének időpotját lementi

    while True:            
        elteltIdo = time() - elozoIdo
        #* Fordulás óta eltelt idő

        if(raallSzog(leftm, leftSensor, minimumFeny, maxSebesseg, KP) == 0 and raallSzog(rightm, rightSensor, minimumFeny, maxSebesseg, KP) == 0):
            m.stop(None, False)
            break
        #* Elindítja a motorokat a funkciókkal és megnézi, hogy mindkettő 0
        #* ha igen akkor leállítja a programot, mert elivleg sikeresen ráállt a vonalra

        if(elteltIdo >= maxIdo):
            m.stop(None, False)
            break
        #*
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSlowDown = True, drift = 0, margin = 0, calibrate = False, debug = False):
    """Make the robot go staright in a specified degree (cm)"""
    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
    timesBad = 0
    status = "Starting"
    speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
    slowingDown = distance * 0.6
    direction = sign(maxSpeed)
    minSpeed *= direction
    if(speedingUp > (2 * wheelDiameter)):
        speedingUp = (2 * wheelDiameter)
    if(slowingDown > (2*wheelDiameter)):
        slowingDown = (2 * wheelDiameter)
    m.on(minSpeed, minSpeed)
    while abs(getRotations() - startRotations) <= distance:
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    raall(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                m.stop(None, False)
                break
             
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance)
        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed)) * 2
        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5
        calculatedSensitivity = sensitivity / sensitivityMultiplier
        if(abs(gs.angle - targetAngle) > 0):
            calculatedAngle = ((gs.angle) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység
            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni
            """if(gs.angle == irany):
                pontos += 1
            osszesMeres += 1"""
            #* Pontosságot számolja
            s.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
        else:
            if(timesBad + margin != timesGood):
                s.on(-previousAngle, calculatedSpeed)
            else:
                s.on(drift, calculatedSpeed)
            timesGood += 1
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
            print(sensitivityMultiplier)
    newWheelDiameter = (calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop != False):
        m.on(motorStop, motorStop)
    else:
        m.stop(None, False)
    if(calibrate):
        leftm.reset()
        rightm.reset()
        return (newWheelDiameter)
    #* Ha így bekapcsolva marad a 
def fordul2(angle, maxSpeed, sensitvity, relative = True, stopMargin = 2, minSpeed = 2):
    angle = angle * -1
    #? Így megy a jó irányba, gyro meg van fordítva
    fordulatszam = 0
    hasStopped = False
    if(relative == True):
        fordulatszam = gs.angle
    turnConstant = (fordulatszam-angle)
    stopMargin *= sign(turnConstant)
    while gs.angle != fordulatszam - angle:
        if(turnConstant - stopMargin <= gs.angle <= turnConstant + stopMargin and hasStopped == False and stopMargin != 0):
            hasStopped = True
            m.stop()
            print(str(turnConstant - stopMargin) + " | " + str(gs.angle))
            continue
        calculatedSpeed = (turnConstant - gs.angle) * sensitvity
        if(calculatedSpeed > maxSpeed / 3.5):
            calculatedSpeed = maxSpeed
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(maxSpeed) * sign(calculatedSpeed))
        if(abs(calculatedSpeed) < minSpeed): calculatedSpeed = (abs(minSpeed) * sign(calculatedSpeed))
        m.on(-calculatedSpeed, calculatedSpeed)
        
    m.stop()
    print("\nDONE\n")
drift = -2
gs.reset()
straight(1, 35, 0, 1.2, 5, False, False, False, True, True, -2, 0)
fordul2(90, 70, 0.38, False, 8, 7)
#fordul(90, 80, 0.3, 0.5, hibahatar=2, idotullepes=3, relativ=False, stopAfter=50, waitMargin=1, debug=False, minSpeed=3)
print(gs.angle)
fordul2(-90, 70, 0.58, False, 6, 7)
#fordul(-90, 80, 0.55, 0.5, idotullepes=3, relativ=False, stopAfter=10, waitMargin=1, debug=False, minSpeed=3)
print(gs.angle)
fordul2(1, 70, 0.38, False, 10, 7)
#fordul(1, 80, 0.3, 0.5, idotullepes=2, relativ=False, stopAfter=45, waitMargin=1, debug=False, minSpeed=3)
print(gs.angle)
straight(1, -35, 0, 1.2, 5, False, False, False, True, True, -2, 0)