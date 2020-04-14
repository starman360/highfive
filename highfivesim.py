## This is the high-five simulator that will make it all happen
import numpy as np
import pandas as pd

class HighFiveSim():
    def __init__(self, handFile, robotFile):
        self.handFile = handFile
        self.robotFile = robotFile

        self.robotTime = 0
        self.handTime = 0
        self.robotGoal = 0
        self.handGoal = 0

    def importData(self):
        self.handData = pd.read_csv(self.handFile, engine='python').fillna(0.0)
        self.handGoal = len(self.handData) - 2 
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
        self.robotTime += action
        self.handTime += 1
        if self.robotTime >= self.robotGoal or self.handTime >= self.handGoal:
            return -1
        return action

    def isGoal(self):
        if 112 <= self.handTime <= 136:
            if self.robotTime == self.robotGoal:
                return True
        return False

    def getReward(self): #needs work ... will improve.. we promise
        
        if self.isGoal():
            return 1
        else:
            return -1000
        
    def reset(self):
        self.robotTime = 0
        self.handTime = 0

# if __name__ == "__main__":

#     robot_data = 'robot_data/202048_13287.txt'
#     hand_data = 'hand_data/Data2_keyframes.txt'
#     hf = HighFiveSim(hand_data, robot_data)
#     hf.importData()
#     hf.getState()


     