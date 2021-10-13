from sensors import *


class FSM:
    STATIONARY = 0
    MOVING_FORWARD = 1
    TURNING_LEFT = 2
    TURNING_RIGHT = 3
    REVERSING = 4

    def __init__(self, turns):
        self.current_state = FSM.STATIONARY
        self.capturedTime = self.timeNow()

        self.turnQueue = TurnQueue(turns)
        self.ultrasonicSensor = UltrasonicSensor()
        self.trackDetector = TrackDetector(14, 15, 16, 17)
        self.motor = Motor()

    def setNextState(self, new_state):
        self.next_state = new_state
        if new_state == FSM.TURNING_LEFT or new_state == FSM.TURNING_RIGHT:
            self.previous_turn = new_state
            self.captureTime()

    def isCurrently(self, query_state):
        return self.current_state == query_state

    def moveState(self):
        self.current_state = self.next_state

    def captureTime(self):
        self.capturedTime = self.timeNow()

    def getCapturedTime(self):
        return self.captureTime()

    def doCurrentState(self):
        if self.isCurrently(FSM.STATIONARY):
            self.doStationary()
        elif self.isCurrently(FSM.MOVING_FORWARD):
            self.doMovingForward()
        elif self.isCurrently(FSM.TURNING_LEFT):
            self.doTurningLeft()
        elif self.isCurrently(FSM.TURNING_RIGHT):
            self.doTurningRight()
        elif self.isCurrently(FSM.REVERSING):
            self.doReverse()

    def doMovingForward(self):
        # Continue to adjust position to follow middle line...
        if self.trackDetector.isLeftOfLine():
            self.motor.increaseLeft()
        elif self.trackDetector.isRightOfLine():
            self.motor.increaseRight()

        # But check if obstacles or turn is present...
        # Turn check:
        if self.trackDetector.splitDetected():
            # Perform queued turn...
            nextTurn = self.turnQueue.getNext()
            if nextTurn == True:
                self.setNextState(FSM.TURNING_RIGHT)
            else:
                self.setNextState(FSM.TURNING_LEFT)
            self.turnQueue.pop()

        # Reverse if obstacle detected...
        if self.ultrasonicSensor.getLastDistance() < 200:
            self.setNextState(FSM.REVERSING)

    def doStationary(self):
        # If track is present, go...
        if self.trackDetector.onMiddleLine():
            self.setNextState(FSM.MOVING_FORWARD)
        # If track is not present, stay stationary.

    def doReverse(self):
        # Reverse until split detected...
        self.motor.reverse()

        if self.trackDetector.splitDetected():
            # Get do opposite turn of the last one...
            previousTurn = self.getLastTurn()
            if previousTurn == FSM.TURNING_LEFT:
                self.setNextState(FSM.TURNING_RIGHT)
            elif previousTurn == FSM.TURNING_RIGHT:
                self.setNextState(FSM.TURNING_LEFT)

    def doTurningLeft(self):
        # Ensure motor is turning left...
        self.motor.turnLeft()

        # Ignore right sensors for couple secs...
        timeStartedTurning = self.getCapturedTime()
        if self.timeNow() - timeStartedTurning <= 2:
            if self.trackDetector.readLeftSensors() == [True, False]:
                self.motor.increaseLeft()

    def doTurningRight(self):
        # Ensure motor is turning right...
        self.motor.turnRight()

        # Ignore left sensor for couple of seconds...
        timeStartedTurning = self.getCapturedTime()
        if self.timeNow() - timeStartedTurning <= 2:
            if self.trackDetector.readRightSensors() == [False, True]:
                self.motor.increaseRight()


class TurnQueue():
    def fill(self, turns):
        self.queue = turns

    def length(self):
        return len(self.queue)

    def isEmpty(self):
        return self.length() <= 0

    def getNext(self):
        return self.queue[0]

    def pop(self):
        del self.queue[0]
