
#==============================   People responsible for this devistating tradegy   ==============================

## Anmol Modur, Reid Kovacs 
# Deep Q Learning for High Fives

#==================================================   Imports   ==================================================
import matplotlib.pyplot as plt
import numpy as np
import random
import keras
from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.utils import np_utils
from keras.optimizers import Adam
from collections import deque
import pandas as pd 
import os

from keras.utils import plot_model
from IPython.display import SVG
from keras.utils import model_to_dot

from highfivesim import HighFiveSim

#================================================   Model Class   ================================================

class DeepQModel:
    def __init__(self, environment , gamma = 0.85, epsilon = 1):

        self.environment = environment

        self.memory = deque(maxlen=1000)    #
        self.gamma = gamma                  # Future reward depreciation factor
        self.epsilon = epsilon              # Fraction of time dedicated to exploring
        self.eps_min = 0.1                  # epsilon minimum
        self.eps_decay = 0.995              # epsilon decay
        self.learning_rate = 0.005          # learning rate
        self.tau = 0.125

        self.model  = self.create_model()
        self.model_target = self.create_model()

        self.actions = [0,1,2,3]                # 0 is no go, 1 is go

    def create_model(self):
        ''' Create Keras Sequential Model '''

        self.environment.importData()
        state_shape = self.environment.getState().shape
        
        model = Sequential()
        # print(state_shape)
        # model.add(Dropout(0.1))
        # model.add(Dense(16, input_dim = state_shape[0], activation = 'relu'))   #
        model.add(Dense(40, input_dim = state_shape[1], activation = 'relu'))   #
        model.add(Dense(32, activation = 'relu'))
        model.add(Dense(16, activation = 'relu'))
        model.add(Dense(4,  activation = 'linear')) #how far to go

        plot_model(model, show_shapes = True)
        # input("Press Enter to continue....")

        model.compile(loss='mean_squared_error', optimizer=Adam(lr=self.learning_rate))

        return model

    def choose_action(self, state):
        ''' 
        Chose an action depending on state unless the model is under-developed. During the beginning stages of the training, the actions will be random and slowly it will begin to trust itself
        
        :param state: The current state as to which an action will be decided on
        :return: action
        '''

        self.epsilon = self.epsilon * self.eps_decay
        self.epsilon = max(self.eps_min, self.epsilon)
        if np.random.random() < self.epsilon:
            return self.actions[int(np.round(np.random.random()*3))]         #choose action randomly
        # print("act " + str(state.shape))
        # print(type(state))
        # print(state)
        return np.argmax(self.model.predict(state)[0])
        
    def record(self, state, action, reward, new_state, goal):
        ''' record state information to memory '''
        
        self.memory.append([state, action, reward, new_state, goal])

    def replay(self):
        ''' ???? '''
        batch_size = 32 
        if len(self.memory) < batch_size:
            return

        samples = random.sample(self.memory, batch_size)
        for sample in samples:
            state, action, reward, new_state, goal = sample
            # print("replay " + str(state.shape))
            # print(state)
            target = self.model_target.predict(state)
            if goal:
                target[0][action] = reward
            else:
                Q_future = max(self.model_target.predict(new_state)[0])
                target[0][action] = reward + Q_future * self.gamma
            self.model.fit(state, target, epochs=1, verbose=0)

    def target_train(self):
        ''' '''

        weights = self.model.get_weights()
        target_weights = self.model_target.get_weights()
        for i in range(len(target_weights)):
            target_weights[i] = weights[i] * self.tau + target_weights[i] * (1-self.tau)
        self.model_target.set_weights(target_weights)


    # def save_model(self, fn):
    #     self.model.save(fn)

#================================================   Main Function   ================================================

def main():
    # Loading Simulator
    robot_data = 'robot_data/202048_13287.txt'
    hand_data = 'hand_data/Data2_keyframes.txt'

    environment = HighFiveSim(hand_data, robot_data)
    highFiveAgent = DeepQModel( environment=environment)

    trials = 100
    trial_len = 189 # duration of hand time series

    for trial in range(trials):

        current_state = environment.getState() #??

        for step in range(trial_len):
            action = environment.performAction(highFiveAgent.choose_action(current_state))
            ## anmol look here
            new_state = environment.getState()
            reward = environment.getReward()
            #reward if goal 
            goal = environment.isGoal()
            highFiveAgent.record(current_state, action, reward, new_state, goal )

            highFiveAgent.replay()
            highFiveAgent.target_train()

            current_state = new_state

            if goal:
                break
        if goal:
            print("completed in {} trials!".format(trial))
            break
        else:
            print("failed trial {}".format(trial))

if __name__ == "__main__":
    main()




    
