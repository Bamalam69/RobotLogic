import time
from typing import DefaultDict
import RPi.GPIO as GPIO
from threading import Thread

class SensorBase():
    def __init__(self):
        if GPIO.getmode() != GPIO.BOARD:
            GPIO.setmode(GPIO.BOARD)

class StringPuller(SensorBase):
    def __init__(self, transistor_pin=35): # GPIO19
        SensorBase.__init__(self)
        self.transistor_pin = transistor_pin
        GPIO.setup(self.transistor_pin, GPIO.OUT)

    def pullString(self, duration=15):
        timeStarted = time.time()
        while timeStarted - time.time() <= duration:
            GPIO.output(self.transistor_pin, GPIO.HIGH)
        GPIO.output(self.transistor_pin.GPIO.LOW)


class UltrasonicSensor(SensorBase):
    # GPIO4 - ECHO, GPIO6 - TRIGGER
    def __init__(self, trig_pin=31, echo_pin=7):
        SensorBase.__init__(self)
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        # Initialize pins via RPi.GPIO
        # Start reading thread...
        self.thread = Thread(target=self._distancingThread, args=(self))
        self.threadShouldRun = True
        self.thread.start()

    def _distancingThread(self):
        while self.threadShouldRun:
            # Read from ultrasonic sensor...
            # Save value in class...
            GPIO.output(self.trig_pin, True)  # pings US to emit wave
            time.sleep(0.00001)
            GPIO.output(self.trig_pin, False)  # pings US to stop emiting Wave

            StartTime = time.time()  # defines time
            StopTime = time.time()  # difines time

            while GPIO.input(self.echo_pin) == 0:  # when it is not receiving the wave
                StartTime = time.time()

            while GPIO.input(self.echo_pin) == 1:  # when it receives a response wave
                StopTime = time.time()

            # calculates time taken to emit and recieve wave
            TimeElapsed = StopTime - StartTime
            # math to calculate distance
            self.lastReadDistance = (TimeElapsed * 34300) / 2

    def getLastDistance(self):
        return self.lastReadDistance  # actual distance

    def __del__(self):
        self.threadShouldRun = False
        self.thread.join()


class LineSensor(SensorBase):
    def __init__(self, io_pin):
        SensorBase.__init__(self)
        self.io_pin = io_pin
        # Initialize pins via RPi.GPIO
        GPIO.setup(self.io_pin, GPIO.IN)   # 1  Left

    def isOnTrack(self):
        # Read from IR pin...
        return GPIO.input(self.io_pin)

class TrackDetector():
    # L, LC, RC, R (GPIO13, GPIO5, GPIO27, GPIO22)
    def __init__(self, io_pin0=33, io_pin1=29, io_pin2=13, io_pin3=15): 
        SensorBase.__init__(self)
        self.io_pin0 = io_pin0
        self.io_pin1 = io_pin1
        self.io_pin2 = io_pin2
        self.io_pin3 = io_pin3

        self.lineSensors = [LineSensor(i)
                            for i in [io_pin0, io_pin1, io_pin2, io_pin3]]

    def onMiddleLine(self):
        return [self.lineSensors[i].isOnTrack() for i in range(4)] == [False, True, True, False]

    def isLeftOfLine(self):
        return [self.lineSensors[i].isOnTrack() for i in [0, 1]] == [True, True]

    def isRightOfLine(self):
        return [self.lineSensors[i].isOnTrack() for i in [3, 4]] == [True, True]

    def splitDetected(self):
        return [self.lineSensors[i].isOnTrack() for i in range(4)] == [True for i in range(4)]


class Motor(SensorBase):
    DEFAULT_SPEED = 65
    DEFAULT_TURN_DIFFERENTIAL = 3.0
    DEFAULT_PWM_FREQ = 100

    FORWARD = GPIO.HIGH
    BACKWARDS = GPIO.LOW

    # GPIO16, GPIO26, GPIO20, GPIO21
    def __init__(self, directionAPin=36, pwmAPin=37, directionBPin=38, pwmBPin=40):
        SensorBase.__init__(self)
        self.directionAPin = directionAPin
        self.pwmAPin = pwmAPin

        self.directionBPin = directionBPin
        self.pwmBPin = pwmBPin

        # Setup pins for output
        GPIO.setup(self.directionAPin, GPIO.OUTPUT)
        GPIO.setup(self.directionBPin, GPIO.OUTPUT)
        GPIO.setup(self.pwmAPin, GPIO.OUTPUT)
        GPIO.setup(self.pwmBPin, GPIO.OUTPUT)

        # Set initial drive direction to forward:
        self.setDirection(Motor.FORWARD)

        self.pwmA = GPIO.PWM(self.pwmAPin, Motor.DEFAULT_PWM_FREQ)
        self.pwmB = GPIO.PWM(self.pwmBPin, Motor.DEFAULT_PWM_FREQ)
        self.pwmA.start(0)
        self.pwmB.start(0)

    def setDirection(self, direction):
        GPIO.output(self.directionAPin, direction)
        GPIO.output(self.directionBPin, direction)

    def turnRight(self):
        # PWM right higher than left by higher value initially...
        self.pwmA.ChangeDutyCycle(0.5 * self.default_speed)
        self.pwmAPrevious = 0.5 * self.DEFAULT_SPEED

        self.pwmB.start(1 * self.default_speed)
        self.pwmBPrevious = 1 * self.DEFAULT_SPEED

    def turnLeft(self):
        # PWM left more than right initially...
        self.pwmA.start(1 * self.default_speed)
        self.pwmAPrevious = 1 * self.DEFAULT_SPEED

        self.pwmB.start(0.5 * self.default_speed)
        self.pwmBPrevious = 0.5 * self.DEFAULT_SPEED

    def reverse(self):
        self.setDirection(Motor.BACKWARDS)

    def increaseLeft(self, incrementation=2.0):
        self.pwmA.ChangeDutyCycle(self.pwmAPrevious + incrementation)
        self.pwmAPrevious += incrementation

    def increaseRight(self, incrementation=2.0):
        self.pwmB.ChangeDutyCycle(self.pwmBPrevious + incrementation)
        self.pwmBPrevious += incrementation

class Servo(SensorBase):
    # GPIO17
    def __init__(self, ctrl_pin=11):
        # Servo PIN
        self.pwmPin = ctrl_pin
        GPIO.setup(self.pwmPin, GPIO.OUTPUT)
        self.pwmControl = GPIO.PWM(self.pwmPin, 100)

        # Start thread...
        self.thread = Thread(target=self._servoThread, args=(self), name="Control")
        self.threadShouldRun = True
        self.thread.start()

    
    def sweep(self):
        if self.thread != None or self.thread.getName() != "Sweep":
            self.threadShouldRun = False
            self.thread.join()
            del(self.thread)

        self.thread = Thread(target=self._sweepThread, args=(self), name="Sweep")
        self.threadShouldRun = True
        self.thread.start()

    def goto(self, position):
        self.nextPosition = position;

    def _servoThread(self):
        self.pwm.start(0)
        while self.threadShouldRun:
            self.pwm.ChangeDutyCycle(self.nextPosition)
        self.pwm.stop()

    def _sweepThread(self):
        self.pwmControl.start(0)
        value = 0
        direction = 1
        while self.threadShouldRun:
            value += direction
            if value > 180:
                direction *= -1
                value = 180

            self.pwmControl.ChangeDutyCycle(100 * value / 180.0)
            time.sleep(0.01)
        self.pwmControl.stop()

    def __del__(self):
        self.threadShouldRun = False
        self.thread.join()
      


# class Finalespeed


# MotorR_speed.start(speed)
# time.wait(0.020)

# # may need a function to check if the Robot is on the line
# (5)
# servo_enspeede == 0speedncreaseRight(self, incrementation=2.0):
#     for speed in La
# GPIO.OUTPUT(A1, TRUE)  # Does Donut
# MotorL_speed.start(100)
# s360's""BurnoutsCelebration dance                                                                         GPIO.OuUTPITUT(SERSERVO_CNTRL, TURRUE)GPIO.OUTPUT(SERVO_CNTRL, TRUE)FALSE
