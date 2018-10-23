from LabDeltaRobot import *
import time

#### Call DeltaRobot class ####
deltarobot = DeltaRobot()

#### Initial setup ####
deltarobot.SetPID1(2000,1000,4700)
deltarobot.SetPID2(2000,1000,4700)
deltarobot.SetPID3(2000,1000,4700)
# Feedforward Set
FF1_Gain1 = 100
FF2_Gain1 = 50
deltarobot.SetFFGain1(FF1_Gain1,FF2_Gain1)
FF1_Gain2 = 100
FF2_Gain2 = 50
deltarobot.SetFFGain2(FF1_Gain2,FF2_Gain2)
FF1_Gain3 = 100
FF2_Gain3 = 50
deltarobot.SetFFGain3(FF1_Gain3,FF2_Gain3)
# Goal Current Set ###
deltarobot.SetGoalCurrent(150)

deltarobot.SetProfile1(60,15)
deltarobot.SetProfile2(60,15)
deltarobot.SetProfile3(60,15)

deltarobot.TorqueOff()

deltarobot.DeltaTeachPoint()
