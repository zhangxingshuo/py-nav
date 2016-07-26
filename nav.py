import cv2
import numpy as np 
import time
import socket
from Matcher import Matcher
from local import Localize 
# from arduino import Arduino
from GUI import *


class Navigator(object):

    def __init__(self):
        # self.robot = Arduino()
        self.dest = [None, None]
        self.counter = 0
        self.state = 'Idle'
        self.command = 's'
        self.host = '134.173.25.106'
        self.ipadHost = '134.173.24.116'
        self.ipadPort = 5003
        self.port = 5000
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Connecting...")
        self.robot.connect((self.host,self.port))
        self.ipad.connect((self.ipadHost, self.ipadPort))
        print("Connected!!")

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
       data = ''
       data = self.robot.recv(1024).decode()
       print (data)

    def write(self, filename):
        file = open(filename, 'w')
        file.write(self.command)

    def doAction(self, currAngle, destAngle):
        print(self.state)
        if abs(currAngle-destAngle) <= 15:
            self.state = 'Idle'
            self.counter = 0
            return
        if self.state == 'Turning':
            self.command = 'l'
            self.ts(self.command)
            self.state = 'Waiting'
        elif self.state == 'Waiting':
            self.counter += 1
            if self.counter == 5:
                self.counter = 0
                self.command = 's'
                self.ts(self.command)
                self.state = 'Focusing'
        elif self.state == 'Focusing':
            self.counter += 1
            if self.counter == 300:
                self.counter = 0
                self.state = 'Turning'
        elif self.state == 'Check Angle':
            self.counter += 1
            if self.counter == 100:
                if abs(currAngle-destAngle) <= 15:
                    self.state = 'Idle'
                else:
                    self.state = 'Turning'
                self.counter = 0

    def betterDoAction(self, currAngle, destAngle, currentCircle, destCircle):
        print(self.state)

        def determineAngle(destination, nextState):
            difference = destination - currAngle
            if abs(difference) <= 15:
                self.state = 'Done'
                return
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

        if self.state == 'Initializing':
            if currentCircle == destCircle:
                self.state = 'Determine Direction'
            elif abs(currAngle - 90) <= 15:
                self.state = 'Move Command'
            else:
                self.state = 'Determine Turning'

        elif self.state == 'Determine Turning':
            determineAngle(90, 'Turning')

        # Turn to face the destination circle
        elif self.state == 'Turning':
            self.ts(self.command)
            self.state = 'Waiting'

        elif self.state == 'Waiting':
            if abs(currAngle - 90) <= 15:
                self.state = 'Move Command'
            else:
                transition(3, 'Focusing')

        elif self.state == 'Focusing':
            wait(10, 'Turning')

        # Move to the destination circle
        elif self.state == 'Move Command':
            if currentCircle < destCircle:
                self.command = 'f'
            else:
                self.command = 'b'
            self.ts(self.command)
            self.state = 'Moving'

        elif self.state == 'Moving':
            self.counter += 1
            if self.counter == 70 * abs(currentCircle-destCircle):
                self.state = 'Determine Direction'
                self.counter = 0

        elif self.state == 'Determine Direction':
           determineAngle(destAngle, 'Pointing')

        # Point in the right direction once in the destination circle
        elif self.state == 'Pointing':
            if abs(currAngle - destAngle) <= 15:
                self.state = 'Done'
                # self.ipad.close()
                # l = Localize(self.robot)
                # l.localize()
                # l.analyze()
                # self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # self.ipad.connect((self.ipadHost, self.ipadPort))
            else:
                self.ts(self.command)
                self.state = 'Waiting 2'

        elif self.state == 'Waiting 2':
            if abs(currAngle - destAngle) <= 15:
                self.state = 'Done'

            else:
                transition(3, 'Focusing 2')

        elif self.state == 'Focusing 2':
            wait(10, 'Pointing')


    def initializeCircle(self):
        circles = [None] * NUM_LOCATIONS
        for i in range(NUM_LOCATIONS):
            circles[i] = Circle(50, 141 + 150 * i, 221, 'map/'+str(i), [150, 150, 150])
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
    
    def CheckAndRun(self):
        img = np.zeros((480,200 + 150 * NUM_LOCATIONS,3), np.uint8)
        cv2.namedWindow('GUI')

        # make sure the robot is on the right mode
        self.ts('RESET')

        # Initiating Circles and Matches
        # circle1 = Circle(50, 141, 221, 'map/0', [150, 150, 150])
        # circle2 = Circle(50, 304, 207, 'map/1', [150, 150, 150])
        # circle3 = Circle(50, 498, 196, 'map/2', [150, 150, 150])
        # circles = [circle1, circle2, circle3]
        circles = initializeCircle()

        # Initiating Arrows
        arrows = []
        for circle in circles:
            arrows.append(getArrows(circle, 25))

        def click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                for circle in circles:
                    if circle.inCircle((x,y)):
                        if self.state == 'Idle':
                            self.state = 'Turning'

                        # Reset drawing
                        drawCircle(circles, img)
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
                        self.dest[0] = circle

        cv2.setMouseCallback('GUI', click)

        previousProbs = [[1, [1/75] * 25 ], [1,[1/75] * 25 ] , [1,[1/75] * 25]]

        while True:
            img = np.zeros((480,200 + 150 * NUM_LOCATIONS,3), np.uint8)

            probL = self.read('out.txt')
            while not probL:
                probL = self.read('out.txt')

            illustrateProb(circles, arrows, probL)

            drawCircle(circles, img)
            for arrow in arrows:
                drawArrows(arrow, img)

            circleIndex = probL.index(max(probL, key=lambda item:item[0]))
            angleIndex = probL[circleIndex][1].index(max(probL[circleIndex][1]))
            bestCircle = circles[circleIndex]
            bestAngle = (angleIndex*15) * math.pi/180

            # Draw best guess angle
            cv2.arrowedLine(img, (bestCircle.x, bestCircle.y, ), 
                (int(bestCircle.x - 60*math.cos(bestAngle + math.pi/2)), int(bestCircle.y - 60*math.sin(bestAngle + math.pi/2))), 
                (0, 255, 0), 3)

            if self.dest[0] != None and self.dest[1] != None:
                circle = self.dest[0]
                angle = self.dest[1]
                
                # Draw destination arrow
                cv2.arrowedLine(img, (circle.x, circle.y), 
                    (int(circle.x-60*math.cos(angle*math.pi/180+math.pi/2)), int(circle.y-60*math.sin(angle*math.pi/180+math.pi/2))), 
                    (255, 255, 255), 3)
                self.doAction(self.dest[1], angleIndex*15)
                self.write('command.txt')
            

            cv2.imshow('GUI', img)
            k = cv2.waitKey(1)
            if k == 27:
                break

        cv2.destroyAllWindows()
        self.ts('s')

    def IntervalRun(self):
        img = np.zeros((480,200 + 150 * NUM_LOCATIONS,3), np.uint8)
        cv2.namedWindow('GUI')

        # make sure the robot is on the right mode
        self.ts('RESET')

        # Initiating Circles
        circles = initializeCircle()

        # Initiating Arrows
        arrows = []
        for circle in circles:
            arrows.append(getArrows(circle, 25))

        def click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                for circle in circles:
                    if circle.inCircle((x,y)):
                        if self.state == 'Idle' or self.state == 'Done':
                            self.state = 'Initializing'

                        # Reset drawing
                        drawCircle(circles, img)
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
                        self.dest[0] = circle

        self.ipad.close()
        l = Localize(self.robot)
        l.localize()
        l.analyze()
        self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ipad.connect((self.ipadHost, self.ipadPort))

        cv2.setMouseCallback('GUI', click)
        currentAngleIndex = self.readBestGuess('bestGuess.txt')[-1][1]
        currentCircleIndex = self.readBestGuess('bestGuess.txt')[-1][0]
        bestCircle = circles[currentCircleIndex]
        bestAngle = currentAngleIndex * 15 *math.pi/180
        displacement = 0
        imageIndex = 0
        prevAng = self.readGyro()
        file = open('commands.txt', 'w')
        while True:
            img = np.zeros((480,200 + 150 * NUM_LOCATIONS,3), np.uint8)

            currentAngleIndex = self.readBestGuess('bestGuess.txt')[-1][1]
            currentCircleIndex = self.readBestGuess('bestGuess.txt')[-1][0]
            localizedCircle = circles[currentCircleIndex]
            localizedAngle = currentAngleIndex * 15 *math.pi/180

            drawCircle(circles, img)
            for arrow in arrows:
                drawArrows(arrow, img)

            # # Draw best guess angle
            cv2.arrowedLine(img, (bestCircle.x, bestCircle.y, ), 
                (int(bestCircle.x - 60*math.cos(bestAngle + math.pi/2)), int(bestCircle.y - 60*math.sin(bestAngle + math.pi/2))), 
                (0, 255, 0), 3)

            cv2.arrowedLine(img, (localizedCircle.x, localizedCircle.y, ),
                (int(localizedCircle.x - 60 * math.cos(localizedAngle + math.pi/2)), int(localizedCircle.y - 60*math.sin(localizedAngle + math.pi/2))),
                (0, 255, 0), 3 )

             # Check for current location
            bestAngle = bestAngle - 1.0 *displacement 
            currentAng = self.readGyro()

            if self.dest[0] != None and self.dest[1] != None and self.state != 'Done':
                # file = open('commands.txt', 'a')
                circle = self.dest[0]
                angle = self.dest[1]
                
                # Draw destination arrow
                cv2.arrowedLine(img, (circle.x, circle.y), 
                    (int(circle.x-60*math.cos(angle*math.pi/180+math.pi/2)), int(circle.y-60*math.sin(angle*math.pi/180+math.pi/2))), 
                    (255, 255, 255), 3)
                self.betterDoAction((bestAngle*180./math.pi) % 360, self.dest[1], currentCircleIndex, circles.index(self.dest[0]))
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

