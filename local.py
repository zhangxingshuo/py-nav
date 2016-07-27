import cv2
import numpy as np 
import socket
import re
import time
import math
import os

import glob
from analyze import analyzer
from Matcher import Matcher
from urllib.request import urlopen

url = 'http://134.173.25.106:8080/?action=stream'

class Localize(object):

    def __init__(self, robot):
        self.h, self.w = 320, 240
        self.numLocations = 7
        # host = '134.173.24.116'
        # port = 5003
        # print('Waiting for Connection....')
        # self.ipad= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.ipad.connect((host,port))
        # print('Connected!')
        self.matcher = Matcher('BOW', None, self.h, self.w)
        self.frame = self.readImage()
        self.robot = robot

        # for tracking the image during runs
        self.imageIndex = 0

    def BOWMatch(self, image):
        self.matcher.setImage(image)
        results = []
        for i in range(self.numLocations):
            self.matcher.setDirectory('map/' + str(i))
            # matcher.setFeatureIndex(self.featureIndices[i])
            totalMatches, probL = self.matcher.run()
            results.append([totalMatches, probL])
        return results

    def write(self, probL, filename):
        file = open(filename, 'w')
        for circle in probL:
            totalMatches = circle[0]
            probs = circle[1]
            file.write(str(totalMatches) + ', ')
            for prob in probs:
                file.write(str(prob) + ', ')
            file.write('\n')

    def readImage(self):
        stream = urlopen(url)
        stream.readline()

        sz = 0
        rdbuffer = None

        clen_re = re.compile(b'Content-Length: (\d+)\\r\\n')
        stream.readline()                    # content type
        
        try:                                 # content length
            m = clen_re.match(stream.readline()) 
            clen = int(m.group(1))
            # indexOfImage += 1
        except:
            print('oops')
        
        stream.readline()                    # timestamp
        stream.readline()                    # empty line
        
        # Reallocate buffer if necessary
        if clen > sz:
            sz = clen*2
            rdbuffer = bytearray(sz)
            rdview = memoryview(rdbuffer)
        
        # Read frame into the preallocated buffer
        stream.readinto(rdview[:clen])
        
        stream.readline() # endline
        stream.readline() # boundary
            
        # This line will need to be different when using OpenCV 2.x
        img = cv2.imdecode(np.frombuffer(rdbuffer, count=clen, dtype=np.byte), flags=cv2.IMREAD_COLOR)
        return img

    def ts(self, message):
       self.robot.send(str(message).encode()) 
       data = ''
       data = self.robot.recv(1024).decode()
       print (data)

    def localize(self):
        file = open('commands.txt', 'w')
        # self.host = '134.173.25.106'
        # self.port = 5000
        # self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # print("Connecting...")
        # self.robot.connect((self.host,self.port))
        # print("Connected!!")
        self.ts('RESET')
        imageName = 0
        while True:
            self.ts('r')
            time.sleep(0.2)
            self.ts('s')
            time.sleep(3)
            image = self.readImage()
            cv2.imwrite('cam1_img/' + str(imageName).zfill(4) + '.png', image)
            file.write(str(imageName).zfill(4) + ":" + 'r\n')
            imageName += 1
            if imageName == 7:
                self.ts('s')
                break

    def save(self, index):
        self.frame = self.readImage()
        image = self.frame
        cv2.imwrite('cam1_img/' + str(index).zfill(4) + '.png', image)

    def delete(self):
        for file in glob.glob('cam1_img/*.png'):
            os.remove(file)

    def analyze(self):
        # a = analyzer('BOW', 800, 600)
        a = analyzer('SIFT', 320, 240)
        a.createRawP()
        a.processRaw()
        self.delete()

        # bestGuess = readBestGuess('bestGuess.txt')
        # return bestGuess[-1]

    def run(self):
        # print('Analyzing...')
        # previousProbs = []
        # for i in range(self.numLocations):
        #     previousProbs.append([1, [1/75] * 25 ])

        # while True:
        #     # Reading Angles and Gyro
        #     previousAngle = (self.readGyro() * 180./math.pi) % 360
        #     self.frame = self.readImage()
        #     # cv2.imwrite('cam/' + str(counter).zfill(4) + '.png', self.frame)
        #     currentAngle = (self.readGyro() * 180./math.pi) % 360
        #     command = 's'
        #     cv2.imshow('captured', self.frame)
        #     cv2.waitKey(1)
        #     # Calculating Action
        #     diff = currentAngle - previousAngle
        #     if (diff > 1 and diff < 300) or diff < -300:
        #         command = 'l'
        #     elif (diff < -1 and diff > -300) or diff > 300:
        #         command = 'r'

        #     blurFactor = self.Laplacian(self.frame)
        #     probL = self.BOWMatch(self.frame)
        #     # print(probL)
        #     accountAction = self.accountCommand(command, previousProbs)
        #     adjusted = self.prevWeight(accountAction, probL)
        #     blurCorrect = self.blurCorrect(previousProbs, probL, blurFactor)
        #     previousProbs = blurCorrect
        #     self.write(blurCorrect, 'out.txt')
        #     # counter += 1
        self.localize()
        self.analyze()

    ############################
    ### Probability Updating ###
    ############################

    def blurCorrect(self, previousP, currentP, blurFactor):
        '''this function weighted the probability list according to the blurriness factor'''
        currentWeight = 0
        if blurFactor > 20:
            currentWeight = 0.85
        else:
            currentWeight = (blurFactor / 200) * 0.85
        previousWeight = 1 - currentWeight

        # Assigning the weight to each list
        truePosition = []
        for i in range(self.numLocations):
            truePosition.append([0, []])


        for circleIndex in range(len(truePosition)):
            currentCircle = currentP[circleIndex]
            previousCircle = previousP[circleIndex]

            # Number of matches 
            current_num_matches = currentCircle[0]
            previous_num_matches = previousCircle[0]
            
            # Each probability list
            current_probList = currentCircle[1]
            previous_probList = previousCircle[1]


            truePosition[circleIndex][0] = (currentWeight * current_num_matches + previousWeight * previous_num_matches)
            for probIndex in range(len(currentP[circleIndex][1])): 

                current_prob = current_probList[probIndex]
                previous_prob = previous_probList[probIndex]

                truePosition[circleIndex][1].append(currentWeight * current_prob + previousWeight * previous_prob)

        return truePosition

    def prevWeight(self, previousP, currentP):
        '''this function weighted the probability list according to the blurriness factor'''
        currentWeight = 0.7
        previousWeight = 1- currentWeight

        # Assigning the weight to each list
        truePosition = []
        for i in range(self.numLocations):
            truePosition.append([0, []])

        for circleIndex in range(len(truePosition)):
            currentCircle = currentP[circleIndex]
            previousCircle = previousP[circleIndex]

            # Number of matches 
            current_num_matches = currentCircle[0]
            previous_num_matches = previousCircle[0]
            
            # Each probability list
            current_probList = currentCircle[1]
            previous_probList = previousCircle[1]


            truePosition[circleIndex][0] = (currentWeight * current_num_matches + previousWeight * previous_num_matches)
            for probIndex in range(len(currentP[circleIndex][1])): 

                current_prob = current_probList[probIndex]
                previous_prob = previous_probList[probIndex]

                truePosition[circleIndex][1].append(currentWeight * current_prob + previousWeight * previous_prob)

        return truePosition

    def accountCommand(self, command, previousP):
        '''this funciton accounts for the command robot is given at the moment'''
        # Left
        copy = previousP[:]
        if command == 'l':
            for circles in copy:
                circles[1] = circles[1][1:] + circles[1][0:1]
        elif command == 'r':
            for circles in copy:
                circles[1] = circles[1][-1:] + circles[1][0:-1]
        elif command == 'f':
            bestCircleIndex = previousP.index(max(previousP))
            bestAngleIndex = previousP[bestCircleIndex][1].index(max(previousP[bestCircleIndex][1]))
            factor = 0.05 * abs(math.sin(bestAngleIndex*15 * 180/math.pi))
            if bestCircleIndex < self.numLocations - 1 and bestAngleIndex*15 < 180 and bestAngleIndex > 0:
                copy[bestCircleIndex+1][0] *= (1 + factor)
            elif bestCircleIndex > 0 and bestAngleIndex*15 > 180 and bestAngleIndex*15 < 360: 
                copy[bestCircleIndex-1][0] *= (1 + factor)
        return copy

    def Laplacian(self, img):
        ''' this function calcualte the blurriness factor'''
        # img = cv2.imread(imagePath, 0)
        var = cv2.Laplacian(img, cv2.CV_64F).var()
        return var


if __name__ == '__main__':
    localize = Localize()
    localize.run()
