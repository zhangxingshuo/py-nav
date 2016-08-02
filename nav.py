import cv2
import numpy as np 
import time
import socket
from Matcher import Matcher
from local import Localize 
# from arduino import Arduino
from GUI import *

NUM_COLUMNS = 7
NUM_ROWS = 3

class Navigator(object):

    def __init__(self):
        # self.robot = Arduino()
        self.dest = [None, None]
        self.counter = 0
        self.state = 'Idle'
        self.command = 's'
        self.host = '134.173.27.40'
        self.ipadHost = '134.173.29.21'
        self.ipadPort = 5001
        self.port = 5000
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Connecting...")
        self.robot.connect((self.host,self.port))
        self.ipad.connect((self.ipadHost, self.ipadPort))
        print("Connected!!")
        self.tempDest = None
        self.distances = {( (0,0), (1,0) ):37,
        ( (1,0), (2,0) ):31.5,
        ( (2,0), (3,0) ):30.5,
        ( (3,0), (4,0) ):32,
        ( (4,0), (5,0) ):26.5,
        ( (5,0), (6,0) ):23,
        ( (5,0), (5,1) ):42.5,
        ( (5,1), (5,2) ):37,
        ( (1,0), (0,0) ):37,
        ( (2,0), (1,0) ):31.5,
        ( (3,0), (2,0) ):30.5,
        ( (4,0), (3,0) ):32,
        ( (5,0), (4,0) ):26.5,
        ( (6,0), (5,0) ):23,
        ( (5,1), (5,0) ):42.5,
        ( (5,2), (5,1) ):37}
        # self.distances = {((0,0),(1,0)): 37, ((1,0),(2,0)): 31.5, ((2,0),(3,0)): 30.5, ((3,0),(4,0)): 32, (4,,5): 26.5, (5,6): 23, (6,7): 42.5, (7,8):37 }

    def read(self, filename):
        file = open(filename, 'r')
        raw_content = file.read().split('\n')[:-1]
        mapped = [list(map(float, item.split(', ')[:-1])) for item in raw_content]
        results = []
        for L in mapped:
            results.append([L[0], L[1:]])
        return results

    def ts(self, message):
       self.robot.send(str(message).encode()) 
       # data = ''
       data = self.robot.recv(1024).decode()
       print (data)

    def write(self, filename):
        file = open(filename, 'w')
        file.write(self.command)

    def handleAction(self, currAngle, destAngle, currentCircle, destCircle):
        print(self.state)
        currentColumn = currentCircle.relativeCoord[0]
        currentRow = currentCircle.relativeCoord[1]
        destColumn = destCircle.relativeCoord[0]
        destRow = destCircle.relativeCoord[1]
        
        if self.tempDest is not None:
            destColumn = self.tempDest[0]
            destRow = self.tempDest[1]

        def transition(counter, nextState):
            self.counter += 1
            if self.counter == counter:
                self.counter = 0
                self.ts('s')
                self.state = nextState

        def wait(counter, nextState):
            self.counter += 1
            if self.counter == counter:
                self.counter = 0
                self.state = nextState

        def determineAngle(destination, nextState):
            difference = destination - currAngle
            if difference > 0:
                if difference > 180: 
                    self.command = 'l'
                else:
                    self.command = 'r'
            elif difference < 0:
                if difference < -180:
                    self.command = 'r'
                else:
                    self.command = 'l'
            self.state = nextState


        if self.state == 'Initializing':
            if currentColumn == destColumn and currentRow == destRow:
                self.state = 'Pointing Direction'
            elif currentColumn != destColumn and currentRow != destRow:
                self.state = 'Planning'
            else:
                self.state = 'Determine Direction'

        ################################################
        ### Pointing (Intra-circle Angle Transition) ###
        ################################################

        elif self.state == 'Pointing Direction':
            determineAngle(destAngle, 'Pointing')

        elif self.state == 'Pointing':
            if currentColumn == destColumn and currentRow == destRow:
                if abs(currAngle - destAngle) <= 10:
                    self.state = 'Done'
                    return
            if currentColumn == destColumn:
                if currentRow < destRow:
                    destination = 0
                else:
                    destination = 180
            else:
                if currentColumn < destColumn:
                    destination = 90
                else:
                    destination = 270
            if abs(currAngle - destination) <= 10:
                self.state = 'Move Command'
            else:
                self.ts(self.command)
                self.state = 'Transitioning'

        elif self.state == 'Transitioning':
            transition(3, 'Focusing')

        elif self.state == 'Focusing':
            wait(10, 'Pointing')

        ###########################################################
        ### Determine Direction (Inter-circle Angle Transition) ###
        ###########################################################

        elif self.state == 'Determine Direction':
            if currentColumn == destColumn:
                if currentRow < destRow:
                    destination = 0
                else:
                    destination = 180
            else:
                if currentColumn < destColumn:
                    destination = 90
                else:
                    destination = 270
            determineAngle(destination, 'Pointing')

        ######################
        ### Moving Forward ###
        ######################

        elif self.state == 'Move Command':
            self.command = 'f'
            self.ts(self.command)
            self.state = 'Moving'

        elif self.state == 'Moving':
            self.counter += 1
            if currentRow == destRow and currentColumn == destColumn:
                self.tempDest = None
                self.state = 'Initializing'

            # self.ts('r')
            # self.ts(self.command)
            
            if abs(currAngle - 0) < 10 or abs(currAngle - 360) < 10:
                directionAngle = 0
                if currentRow == destRow:
                    self.state = 'Pointing Direction'
                    return
                for circle in self.circles:
                    if circle.relativeCoord == (currentColumn, currentRow + 1):
                        nextCircle = circle
            elif abs(currAngle - 90) < 10:
                directionAngle= 90
                if currentColumn == destColumn:
                    self.state = 'Pointing Direction'
                    return
                for circle in self.circles:
                    if circle.relativeCoord == (currentColumn + 1, currentRow):
                        nextCircle = circle
            elif abs(currAngle - 180) < 10:
                directionAngle = 180
                if currentRow == destRow:
                    self.state = 'Pointing Direction'
                    return
                for circle in self.circles:
                    if circle.relativeCoord == (currentColumn, currentRow - 1):
                        nextCircle = circle
            elif abs(currAngle - 270) < 10:
                directionAngle = 270
                if currentColumn == destColumn:
                    self.state = 'Pointing Direction'
                    return
                for circle in self.circles:
                    if circle.relativeCoord == (currentColumn - 1, currentRow):
                        nextCircle = circle
            else:

                self.command = 's'
                self.ts(self.command)
                self.state = 'Determine Direction'
                self.counter = 0
                return
            if self.counter == int(2.9 * self.distances[( (currentColumn, currentRow), nextCircle.relativeCoord )]):
                self.bestCircle = nextCircle
                self.counter = 0


        ################
        ### Planning ###
        ################

        elif self.state == 'Planning':
            if currentRow == 0:
                self.tempDest = (destColumn, 0)
            else:
                self.tempDest = (currentColumn, destRow)
            self.state = 'Determine Direction'

    def initializeCircle(self):
        circles = [None] * (NUM_COLUMNS+NUM_ROWS)
        for i in range(NUM_COLUMNS+ NUM_ROWS):
            circles[i] = Circle(50, 141 + 150 * i, 400, 'map/'+str(i), [150, 150, 150], (i,0))
        circles[7].relativeCoord = (5,1)
        circles[7].x, circles[7].y = 891, 250
        circles[8].relativeCoord = (5,2)
        circles[8].x, circles[8].y = 891, 100 
        return circles

    def readGyro(self):
        data = self.ipad.recv(1024).decode('utf-8')
        angle = float(str(data).split(',')[-1])
        return angle

    def readBestGuess(self, filename):
        '''this function reads the list of best guesses of the robot's position at every position'''
        file = open(filename, 'r')
        content = file.read().split('\n')[:-1]
        content = list(map(int, content))
        bestGuesses = [[content[x], content[x+1]] for x in range(len(content) - 1 ) [::2]    ]
        return bestGuesses
    

    def IntervalRun(self):
        img = np.zeros((480 + 150*NUM_ROWS,200 + 150 * NUM_COLUMNS,3), np.uint8)
        cv2.namedWindow('GUI')

        # make sure the robot is on the right mode
        self.ts('RESET')

        # Initiating Circles
        self.circles = self.initializeCircle()

        # Initiating Arrows
        arrows = []
        for circle in self.circles:
            arrows.append(getArrows(circle, 25))

        def click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                for circle in self.circles:
                    if circle.inCircle((x,y)):
                        if self.state == 'Idle' or self.state == 'Done':
                            self.state = 'Initializing'

                        # Reset drawing
                        drawCircle(self.circles, img)
                        for arrow in arrows:
                            drawArrows(arrow, img)

                        # Draw angle
                        angle = math.atan2(circle.y-y, circle.x-x)
                        angle *= 180/math.pi
                        angle = int(15 * round(angle/15))
                        angle -= 90
                        if angle < 0:
                            angle += 360

                        self.dest[1] = angle
                        # cirlceCoordinate = circle.relativeCoord
                        self.dest[0] = circle 
                        print(self.dest[0])

        self.ipad.close()
        # l = Localize(self.robot)
        # l.localize()
        # l.analyze()
        self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ipad.connect((self.ipadHost, self.ipadPort))

        cv2.setMouseCallback('GUI', click)
        currentAngleIndex = self.readBestGuess('bestGuess.txt')[-1][1]
        currentCircleIndex = self.readBestGuess('bestGuess.txt')[-1][0]
        self.bestCircle = self.circles[currentCircleIndex]
        self.bestAngle = currentAngleIndex * 15 *math.pi/180
        displacement = 0
        imageIndex = 0
        prevAng = self.readGyro()
        file = open('commands.txt', 'w')
        while True:
            img = np.zeros((480,200 + 150 * NUM_COLUMNS,3), np.uint8)

            currentAngleIndex = self.readBestGuess('bestGuess.txt')[-1][1]
            currentCircleIndex = self.readBestGuess('bestGuess.txt')[-1][0]
            localizedCircle = self.circles[currentCircleIndex]
            localizedAngle = currentAngleIndex * 15 *math.pi/180

            drawCircle(self.circles, img)
            for arrow in arrows:
                drawArrows(arrow, img)

            # # Draw best guess angle
            cv2.arrowedLine(img, (self.bestCircle.x, self.bestCircle.y, ), 
                (int(self.bestCircle.x - 60*math.cos(self.bestAngle + math.pi/2)), 
                    int(self.bestCircle.y - 60*math.sin(self.bestAngle + math.pi/2))), 
                (0, 255, 0), 3)

            cv2.arrowedLine(img, (localizedCircle.x, localizedCircle.y, ),
                (int(localizedCircle.x - 60 * math.cos(localizedAngle + math.pi/2)), int(localizedCircle.y - 60*math.sin(localizedAngle + math.pi/2))),
                (0, 255, 0), 3 )

             # Check for current location
            self.bestAngle = self.bestAngle - 1.0 *displacement 
            currentAng = self.readGyro()

            if self.dest[0] != None and self.dest[1] != None and self.state != 'Done':
                # file = open('commands.txt', 'a')
                circle = self.dest[0]
                angle = self.dest[1]
                
                # Draw destination arrow
                cv2.arrowedLine(img, (circle.x, circle.y), 
                    (int(circle.x-60*math.cos(angle*math.pi/180+math.pi/2)), int(circle.y-60*math.sin(angle*math.pi/180+math.pi/2))), 
                    (255, 255, 255), 3)
                self.handleAction((self.bestAngle*180./math.pi) % 360, self.dest[1], self.bestCircle, self.dest[0])
                print(self.command)
                # l.save(imageIndex)
                # file.write(str(imageIndex).zfill(4) + ':' + self.command + '\n')
                imageIndex += 1
            else:
                self.dest[0] = None
                self.dest[1] = None
                imageIndex = 0 
            displacement = (currentAng - prevAng)
            prevAng = currentAng

            cv2.imshow('GUI', img)
            k = cv2.waitKey(1)
            if k == 27:
                break


if __name__ == '__main__':
    nav = Navigator()
    nav.IntervalRun()

