from LabDeltaRobot import *
import time

deltarobot = DeltaRobot()

deltarobot.SetPID1(3000,2000,4000)
deltarobot.SetPID2(3000,2000,4000)
deltarobot.SetPID3(3000,2000,4000)

deltarobot.TorqueOff()

deltarobot.DeltaKinematics()

