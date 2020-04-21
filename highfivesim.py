## This is the high-five simulator that will make it all happen
import numpy as np
import pandas as pd
import math

class HighFiveSim():
    def __init__(self, handFile, robotFile):
        self.handFile = handFile
        self.robotFile = robotFile

        self.robotTime = 0
        self.handTime = 0
        self.robotGoal = 0
        self.handGoal_MIN = 112
        self.handGoal_MAX = 136

        self.rewardMemory = 0
        self.rewardVelocityMeter = 1

    def importData(self):
        self.handData = pd.read_csv(self.handFile, engine='python').fillna(0.0)
        # self.handGoal = len(self.handData) - 2 ##why?
        self.handData = self.handData.drop(columns=['t', ' keyframeid'])
        self.robotData = pd.read_csv(self.robotFile, engine='python').fillna(0.0)
        self.robotGoal = len(self.robotData) - 2 
        self.robotData = self.robotData.drop(columns=['t'])

    def setDataFiles(self, hand, robot):
        self.handFile = hand
        self.robotFile = robot

    def getState(self):
        hdata = self.handData.iloc[self.handTime]
        # print(self.robotTime)
        # print(hdata.head())
        rdata = self.robotData.iloc[self.robotTime]
        bigdata = pd.concat([hdata, rdata], ignore_index=False, sort=False)
        bd = bigdata.to_numpy()
        bd = np.expand_dims(bd, axis=0)
        # print(bd.shape)
        return bd

    def getTimeStep(self, who = 'h'):
        if who == 'r':
            return self.robotTime
        if who == 'h':
            return self.handTime

    def performAction(self, action):
        '''as soon as it gets out of bounds, end trial'''
        self.robotTime += action
        self.handTime += 1
        if self.robotTime >= self.robotGoal or self.handTime >= self.handGoal_MAX:
            return -1
        return action

    def isGoal(self):
        ''' Test if goal state. This is true if the robot end state is acheived 
            within the bounds of the hand goal state.'''

        if self.handGoal_MIN <= self.handTime <= self.handGoal_MAX:
            if self.robotTime == self.robotGoal:
                return True
        return False

    def goalDistance(self):
        '''return frame-count distance from current robot state to ideal hand goal state'''
 
        return (self.handGoal_MAX+self.handGoal_MIN)/2

    def getReward(self): #needs work ... will improve.. we promise
        ''' Two components: distance from goal and velocity'''

        handGoal_MEAN = (self.handGoal_MAX+self.handGoal_MIN)/2

        ## distance
        x = self.goalDistance()
        if x > 0:
            distanceReward = 10/(1+math.exp((x/10)-200))
        else:
            distanceReward = 0

        ## velocity
        if self.rewardMemory == self.robotTime:
            self.rewardVelocityMeter = self.rewardVelocityMeter + 1
            velocityReward = 10/self.rewardVelocityMeter
        else:
            self.rewardVelocityMeter = 1
            velocityReward = 10
        

        self.rewardMemory = self.robotTime
        return distanceReward + velocityReward
           

        # old
        # if self.isGoal():
        #     return 1
        # else:
        #     return -1000
        
    def reset(self):
        self.robotTime = 0
        self.handTime = 0

# if __name__ == "__main__":

#     robot_data = 'robot_data/202048_13287.txt'
#     hand_data = 'hand_data/Data2_keyframes.txt'
#     hf = HighFiveSim(hand_data, robot_data)
#     hf.importData()
#     hf.getState()


     