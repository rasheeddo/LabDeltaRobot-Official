import time
import numpy
import math
import pygame
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class DeltaRobot:
	def __init__(self):

		#### Initialize joy stick ####
		pygame.init()
		self.j = pygame.joystick.Joystick(0)
		self.j.init()

		############################## Math Constant ####################################
		self.rad2deg = 180/math.pi
		self.deg2rad = math.pi/180
		self.pi = math.pi
		self.root3 = math.sqrt(3)

		########################## Robot's  parameters ##################################

		self.sb = 315.339
		self.sp = 88.335
		self.L = 293         #270
		self.l = 555         #545

		self.wb = (self.root3/6)*self.sb
		self.ub = (self.root3/3)*self.sb
		self.wp = (self.root3/6)*self.sp
		self.up = (self.root3/3)*self.sp

		########################## Workspace parameters ##################################

		self.maxR = 250
		self.minStoke = -350
		self.maxStoke = -715 


		####################################################### Set Servo Configuration #############################################################
		# Control table address

		self.ADDR_PRO_MODEL_NUMBER       		 = 0

		self.ADDR_PRO_OPERATING_MODE     		 = 11

		self.ADDR_PRO_CURRENT_LIMIT      		 = 38
		self.ADDR_PRO_ACCELERATION_LIMIT 		 = 40
		self.ADDR_PRO_VELOCITY_LIMIT     		 = 44

		self.ADDR_PRO_TORQUE_ENABLE      		 = 64               # Control table address is different in Dynamixel model

		self.ADDR_PRO_POSITION_D_GAIN    		 = 80
		self.ADDR_PRO_POSITION_I_GAIN    		 = 82
		self.ADDR_PRO_POSITION_P_GAIN    		 = 84

		self.ADDR_PRO_FEEDFORWARD_2nd_GAIN		 = 88
		self.ADDR_PRO_FEEDFORWARD_1st_GAIN 		 = 90

		self.ADDR_PRO_GOAL_CURRENT       		 = 102
		self.ADDR_PRO_GOAL_VELOCITY      		 = 104

		self.ADDR_PRO_PROFILE_ACCELERATION  	 = 108
		self.ADDR_PRO_PROFILE_VELOCITY   		 = 112

		self.ADDR_PRO_GOAL_POSITION      		 = 116
		self.ADDR_PRO_MOVING             		 = 122
		self.ADDR_PRO_MOVING_STATUS       		 = 123

		self.ADDR_PRO_PRESENT_CURRENT    		 = 126 
		self.ADDR_PRO_PRESENT_POSITION   		 = 132


		# Operating Mode Number
		self.CURRENT_CONTROL                     = 0
		self.POSITION_CONTROL                    = 3 # Default
		self.CURRENT_BASED_POSITION_CONTROL      = 5

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# ID
		self.DXL1_ID                      = 1                          
		self.DXL2_ID                      = 2                             
		self.DXL3_ID                      = 3                            
		self.DXL4_ID                      = 4


		self.BAUDRATE                    = 57600             # Dynamixel default self.BAUDRATE : 57600
		self.DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
		                                                	 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()

		# Set port BAUDRATE
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the BAUDRATE")
		else:
			print("Failed to change the BAUDRATE")
			print("Press any key to terminate...")
			getch()
			quit()

	def getButton(self):
		#Read input from the two joysticks
		pygame.event.pump()
		button0 = self.j.get_button(0)
		button1 = self.j.get_button(1)
		button2 = self.j.get_button(2)
		button3 = self.j.get_button(3)
		button4 = self.j.get_button(4)
		button5 = self.j.get_button(5)
		button6 = self.j.get_button(6)
		button7 = self.j.get_button(7)
		button8 = self.j.get_button(8)
		button9 = self.j.get_button(9)
		button10 = self.j.get_button(10)
		joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]

		return joy_button

	def getAxis(self):
		#Read input from the two joysticks
		pygame.event.pump()
		axis0 = self.j.get_axis(0)
		axis1 = self.j.get_axis(1)
		axis2 = self.j.get_axis(2)
		axis3 = self.j.get_axis(4)
		axis4 = self.j.get_axis(3)
		axis5 = self.j.get_axis(5)
		joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
		return joy_axis

	def getHat(self):
		pygame.event.pump()
		hat0 = self.j.get_hat(0)
		joy_hat = hat0

		return joy_hat

	def map(self, val, in_min, in_max, out_min, out_max):
                                                                          
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def ReadModelNumber(self,ID):
		# MX106 -> 321
		# XM540 -> 1120
		# XM430 -> 1020
		model_number, dxl_comm_result, dxl_error = self.self.packetHandler.read2ByteTxRx(self.portHandler, ID, self.ADDR_PRO_MODEL_NUMBER)

		if model_number == 321:
			print("MX106 is on Servo ID%d" %ID)
		elif model_number == 1020:
			print("XM430 is on Servo ID%d" %ID)
		elif model_number == 1120:
			print("XM540 is on Servo ID%d" %ID)
		else:
			print("Unknown model...")

		return model_number

	def SetOperatingMode(self,MODE):

		self.TorqueOff()

		dxl_comm_result, dxl_error = self.self.packetHandler.write1ByteTxRx(self.self.portHandler, self.self.DXL1_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.self.packetHandler.write1ByteTxRx(self.self.portHandler, self.self.DXL2_ID, self.ADDR_PRO_OPERATING_MODE, MODE)
		dxl_comm_result, dxl_error = self.self.packetHandler.write1ByteTxRx(self.self.portHandler, self.self.DXL3_ID, self.ADDR_PRO_OPERATING_MODE, MODE)

		present_mode, dxl_comm_result, dxl_error = self.self.packetHandler.read1ByteTxRx(self.self.portHandler, self.self.DXL1_ID, self.ADDR_PRO_OPERATING_MODE)
		if present_mode == 0:
		    # Current (Torque) Control Mode
			print("Now Operating Mode is Torque Control")
		elif present_mode == 3:
		    # Position Control Mode
			print("Now Operating Mode is Position Control")
		elif present_mode == 5:
		    # Current-based Position Control Mode
			print("Now Operating Mode is Current-based Position Control")
		else:
			print("In other Mode that didn't set!")

	def SetGoalCurrent(self,SetCur):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_CURRENT, SetCur)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Goal Current is set")		

	def GripperCheck(self):
		servo_com4 = 1500
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		time.sleep(1)
		servo_com4 = 3600
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		time.sleep(1)
		servo_com4 = 1500
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)

	def DeltaFWD(self,deg1, deg2, deg3):

		deg1 = deg1*self.deg2rad
		deg2 = deg2*self.deg2rad
		deg3 = deg3*self.deg2rad

		A1v = numpy.array([0, -self.wb - self.L*math.cos(deg1) + self.up, -self.L*math.sin(deg1)])
		A2v = numpy.array([(self.root3/2)*(self.wb + self.L*math.cos(deg2)) - self.sp/2, 0.5*(self.wb+ self.L*math.cos(deg2)) - self.wp, -self.L*math.sin(deg2)])
		A3v = numpy.array([(-self.root3/2)*(self.wb + self.L*math.cos(deg3)) + self.sp/2, 0.5*(self.wb+ self.L*math.cos(deg3)) - self.wp, -self.L*math.sin(deg3)])

		r1 = self.l
		r2 = self.l
		r3 = self.l

		x1 = A1v[0]
		y1 = A1v[1]
		z1 = A1v[2]

		x2 = A2v[0]
		y2 = A2v[1]
		z2 = A2v[2]

		x3 = A3v[0]
		y3 = A3v[1]
		z3 = A3v[2]

		#Select the method to calculate
		#Depends on the height of virtual spheres center
		#Method 1 is used when the height of z1 z2 z3 are equal
		#Method 2, 3, 4 are trying to avoid 0 division at a13 and a23

		if ((z1==z2) and (z2==z3) and (z1==z3)):
			method = 1
		elif ((z1 != z3) and (z2 != z3)):
			method = 2
		elif ((z1 != z2) and (z1 != z3)):
			method = 3
		else:
			method = 4

		if method == 1:
			zn = z1  # z1 = z2 = z3 = zn

			a = 2*(x3 - x1)
			b = 2*(y3 - y1)
			c = r1**2 - r3**2 - x1**2 - y1**2 + x3**2 + y3**2
			d = 2*(x3 - x2)
			e = 2*(y3 - y2)
			f = r2**2 - r3**2 - x2**2 -y2**2 + x3**2 + y3**2

			numX = c*e - b*f
			denX = a*e - b*d
			x = numX/denX
			if x < 0.000001:
				x = 0

			numY = a*f - c*d
			denY = a*e - b*d
			y = numY/denY
			if y < 0.000001:
				y = 0

			A = 1
			B = -2*zn
			C = zn**2 - r1**2 + (x-x1)**2 + (y-y1)**2

			z = [None]*2

			z[0] = (-B + math.sqrt(B**2 - 4*C))/2;
			z[1] = (-B - math.sqrt(B**2 - 4*C))/2;

			realANS = [None]*3

			if z[0] < 0: 
				realANS = numpy.array([x,y,z[0]])
			elif z[1] < 0:
				realANS = numpy.array([x,y,z[1]])
			else:
				showError = "Error: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		elif method ==2:

			a11 = 2*(x3 - x1)
			a12 = 2*(y3 - y1)
			a13 = 2*(z3 - z1)

			a21 = 2*(x3 - x2)
			a22 = 2*(y3 - y2)
			a23 = 2*(z3 - z2)

			b1 = r1**2 - r3**2 - x1**2 - y1**2 - z1**2 + x3**2 + y3**2 + z3**2
			b2 = r2**2 - r3**2 - x2**2 - y2**2 - z2**2 + x3**2 + y3**2 + z3**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2;
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1);
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2;
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY)
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5;
			x[1] = a4*y[1] + a5;
			z[0] = a6*y[0] + a7;
			z[1] = a6*y[1] + a7;

			realANS = [None]*3

			if z[0] < 0:
				realANS = numpy.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = numpy.array([x[1],y[1],z[1]])
			else:
				showError = "Error: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		elif method == 3:
			a11 = 2*(x1 - x2)
			a12 = 2*(y1 - y2)
			a13 = 2*(z1 - z2)

			a21 = 2*(x1 - x3)
			a22 = 2*(y1 - y3)
			a23 = 2*(z1 - z3)

			b1 = r2**2 - r1**2 - x2**2 - y2**2 - z2**2 + x1**2 + y1**2 + z1**2
			b2 = r3**2 - r1**2 - x3**2 - y3**2 - z3**2 + x1**2 + y1**2 + z1**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY);
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5
			x[1] = a4*y[1] + a5
			z[0] = a6*y[0] + a7
			z[1] = a6*y[1] + a7

			realANS = [None]*3

			if z[0] < 0: 
				realANS = numpy.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = numpy.array([x[1],y[1],z[1]])
			else:
				showError = "Error: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		if method == 4:
			a11 = 2*(x2 - x1)
			a12 = 2*(y2 - y1)
			a13 = 2*(z2 - z1)

			a21 = 2*(x2 - x3)
			a22 = 2*(y2 - y3)
			a23 = 2*(z2 - z3)

			b1 = r1**2 - r2**2 - x1**2 - y1**2 - z1**2 + x2**2 + y2**2 + z2**2
			b2 = r3**2 - r2**2 - x3**2 - y3**2 - z3**2 + x2**2 + y2**2 + z2**2

			a1 = (a11/a13) - (a21/a23)
			a2 = (a12/a13) - (a22/a23)
			a3 = (b2/a23) - (b1/a13)

			a4 = -a2/a1
			a5 = -a3/a1
			a6 = (-a21*a4 - a22)/a23
			a7 = (b2 - a21*a5)/a23

			a = a4**2 + 1 + a6**2
			b = 2*a4*(a5 - x1) - 2*y1 + 2*a6*(a7 - z1)
			c = a5*(a5 - 2*x1) + a7*(a7 - 2*z1) + x1**2 + y1**2 + z1**2 - r1**2
			'''
			YY = Symbol('YY')
			sol = solve(a*YY**2 + b*YY + c,YY);
			y = sol
			'''
			y = [None]*2
			y[0] = (-b + math.sqrt(b**2 - 4*a*c)) / (2*a)  
			y[1] = (-b - math.sqrt(b**2 - 4*a*c)) / (2*a)

			x = [None]*2
			z = [None]*2

			x[0] = a4*y[0] + a5
			x[1] = a4*y[1] + a5
			z[0] = a6*y[0] + a7
			z[1] = a6*y[1] + a7

			realANS = [None]*3

			if z[0] < 0:
				realANS = numpy.array([x[0],y[0],z[0]])
			elif z[1] < 0:
				realANS = numpy.array([x[1],y[1],z[1]])
			else:
				showError = "Error: height z is zero"
				print(showError)

			#print("Method: %d" %method)
			#print("FWD_X:%f" %realANS[0])
			#print("FWD_Y:%f" %realANS[1])
			#print("FWD_Z:%f" %realANS[2])

		return realANS

	def DeltaINV(self,x,y,z):

		workingR = math.sqrt(x**2 + y**2)

		a = self.wb - self.up
		b = self.sp/2 - (self.root3/2)*self.wb
		c = self.wp - 0.5*self.wb

		E1 = 2*self.L*(y + a)
		F1 = 2*z*self.L
		G1 = x**2 + y**2 + z**2 + a**2 + self.L**2 + 2*y*a - self.l**2

		E2 = -self.L*(self.root3*(x+b) + y + c)
		F2 = 2*z*self.L
		G2 = x**2 + y**2 + z**2 + b**2 + c**2 + self.L**2 + 2*(x*b + y*c) - self.l**2

		E3 = self.L*(self.root3*(x-b) - y - c)
		F3 = 2*z*self.L
		G3 = x**2 + y**2 + z**2 + b**2 + c**2 + self.L**2 + 2*(-x*b + y*c) - self.l**2;

		t1 = [None]*2
		t2 = [None]*2
		t3 = [None]*2

		t1[0] = (-F1 + math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
		t1[1] = (-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1)
		t2[0] = (-F2 + math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
		t2[1] = (-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2)
		t3[0] = (-F3 + math.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)
		t3[1] = (-F3 - math.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3)

		theta1_1 = 2*math.atan(t1[1]);
		theta2_1 = 2*math.atan(t2[1]);
		theta3_1 = 2*math.atan(t3[1]);

		deg1 = theta1_1*self.rad2deg;
		deg2 = theta2_1*self.rad2deg;
		deg3 = theta3_1*self.rad2deg;


		if isinstance(theta1_1, complex) or isinstance(theta2_1, complex) or isinstance(theta3_1, complex):
			print("Error: Driving angle is complex number")
			return [None]

		if workingR <= self.maxR and z > self.maxStoke and z <= self.minStoke :
			#print("INV_deg1:%f" %deg1)
			#print("INV_deg2:%f" %deg2)
			#print("INV_deg3:%f" %deg3)
			return deg1, deg2, deg3
		else:
			PreAng = self.ReadAngle()
			deg1 = PreAng[0]
			deg2 = PreAng[1]
			deg3 = PreAng[2]
			#print("INV_deg1:%f" %deg1)
			#print("INV_deg2:%f" %deg2)
			#print("INV_deg3:%f" %deg3)
			print("...Out of working range!...")
			return deg1, deg2, deg3

	def XYZOutRange(self,x,y,z):

		workingR = math.sqrt(x**2 + y**2)
		if workingR > self.maxR or z < self.maxStoke or z > self.minStoke :

			WarningFlag = True

		else:
			WarningFlag = False

		return WarningFlag


	def RunServo(self,ServoDeg1,ServoDeg2,ServoDeg3):

		ServoDeg1 = 90 + ServoDeg1  # +90 to compensate the mechanical offset setting
		ServoDeg2 = 90 + ServoDeg2
		ServoDeg3 = 90 + ServoDeg3


		servo_ang1 = self.map(ServoDeg1, 0.0, 360.0, 0, 4095)
		servo_ang2 = self.map(ServoDeg2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(ServoDeg3, 0.0, 360.0, 0, 4095)


		dxl1_goal_position = int(servo_ang1)
		dxl2_goal_position = int(servo_ang2)
		dxl3_goal_position = int(servo_ang3)

		##### Delta Robot Safety Working Range #####
		if dxl1_goal_position and dxl2_goal_position and dxl3_goal_position > 568:
			dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
			dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
			dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, dxl3_goal_position)
		else:
			print("ERROR: servo angle is less than 40deg")

	def ReadAngle(self):

		dxl_present_position1, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position2, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
		dxl_present_position3, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)

		deg1 = self.map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
		deg2 = self.map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)
		deg3 = self.map(dxl_present_position3, 0.0, 4095.0, 0.0, 360.0)

		deg1 = deg1 - 90
		deg2 = deg2 - 90
		deg3 = deg3 - 90

		return deg1, deg2, deg3

	def GripperClose(self):
		servo_com4 = 3600
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		#print("Gripper closed")

	def GripperOpen(self):
		servo_com4 = 1500
		dxl4_goal_position = int(servo_com4)
		dxl_comm_result4, dxl_error4 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_GOAL_POSITION, dxl4_goal_position)
		#print("Gripper Opened")

	def TorqueOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is enable")

	def TorqueOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			print("Torque is disable")

	def TorqueGripperOn(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)

	def TorqueGripperOff(self):
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL4_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)

	def IsMoving1(self):
		Moving1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING)
		return Moving1

	def IsMoving2(self):
		Moving2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING)
		return Moving2

	def IsMoving3(self):
		Moving3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING)
		return Moving3

	def MovingStatus1(self):
		MovingStat1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat1 > 48:
			print("Motor1 is in Trapezodal Profile")
		elif MovingStat1 < 35 and MovingStat1 > 20:
			print("Motor1 is in Triangular Profile")
		elif MovingStat1 < 20 and MovingStat1 > 3:
			print("Motor1 is in Rectangular Profile")
		elif MovingStat1 < 3:
			print("Motor1 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat1

	def MovingStatus2(self):
		MovingStat2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat2 > 48:
			print("Motor2 is in Trapezodal Profile")
		elif MovingStat2 < 35 and MovingStat2 > 20:
			print("Motor2 is in Triangular Profile")
		elif MovingStat2 < 20 and MovingStat2 > 3:
			print("Motor2 is in Rectangular Profile")
		elif MovingStat2 < 3:
			print("Motor2 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat2

	def MovingStatus3(self):
		MovingStat3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING_STATUS)

		if MovingStat3 > 48:
			print("Motor3 is in Trapezodal Profile")
		elif MovingStat3 < 35 and MovingStat3 > 20:
			print("Motor3 is in Triangular Profile")
		elif MovingStat3 < 20 and MovingStat3 > 3:
			print("Motor3 is in Rectangular Profile")
		elif MovingStat3 < 3:
			print("Motor3 is in Step Mode (No Profile)")
		else:
			print("UNKNOWN Profile...")

		return MovingStat3 

	def DeltaPos(self,pre_pos,goal_pos):

		pre_pos_pul = self.map(pre_pos,0.0,360.0,0,4095)
		pre_pos_pul = int(pre_pos_pul)
		goal_pos_pul = self.map(goal_pos,0.0,360.0,0,4095)
		goal_pos_pul = int(goal_pos_pul)

		delta_pos = abs(goal_pos_pul - pre_pos_pul)

		return delta_pos

	def SetProfile1(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 1: %d" %set_V_PRFL)
		print("A PRFL 1: %d" %set_A_PRFL)
		print("--------------------------------")

	def SetProfile2(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 2: %d" %set_V_PRFL)
		print("A PRFL 2: %d" %set_A_PRFL)
		print("--------------------------------") 

	def SetProfile3(self,set_V_PRFL,set_A_PRFL):
		######################### Set Velocity / Acceleration Profile  ##############################
		set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
		set_V_Limit = 500       # 350 Default                  [0.229RPM]

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
		dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

		acceleration_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_ACCELERATION_LIMIT)
		velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_VELOCITY_LIMIT)

		print("V PRFL 3: %d" %set_V_PRFL)
		print("A PRFL 3: %d" %set_A_PRFL)
		print("--------------------------------")    

	def SetPID1(self,set_P_Gain,set_I_Gain,set_D_Gain):

		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 1: %d" %position_P_gain)
		print("Position I Gain 1: %d" %position_I_gain)
		print("Position D Gain 1: %d" %position_D_gain)
		print("------------------------------")

	def SetPID2(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 2: %d" %position_P_gain)
		print("Position I Gain 2: %d" %position_I_gain)
		print("Position D Gain 2: %d" %position_D_gain)
		print("------------------------------")

	def SetPID3(self,set_P_Gain,set_I_Gain,set_D_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

		position_D_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_D_GAIN)
		position_I_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_I_GAIN)
		position_P_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_POSITION_P_GAIN)

		print("Position P Gain 3: %d" %position_P_gain)
		print("Position I Gain 3: %d" %position_I_gain)
		print("Position D Gain 3: %d" %position_D_gain)
		print("------------------------------")

	def SetFFGain1(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 1: %d" %FF1_gain)
		print("Feedforward 2nd Gain 1: %d" %FF2_gain)
		print("------------------------------") 

	def SetFFGain2(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 2: %d" %FF1_gain)
		print("Feedforward 2nd Gain 2: %d" %FF2_gain)
		print("------------------------------")

	def SetFFGain3(self,set_FF1_Gain,set_FF2_Gain):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

		FF1_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_1st_GAIN)
		FF2_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_FEEDFORWARD_2nd_GAIN)

		print("Feedforward 1st Gain 3: %d" %FF1_gain)
		print("Feedforward 2nd Gain 3: %d" %FF2_gain)
		print("------------------------------")

	def TrajectoryGenerationDelta(self,Vstd,Astd,DelPos1,DelPos2,DelPos3):
		DelPos = [DelPos1, DelPos2, DelPos3]
		MIN = min(DelPos)
		MAX = max(DelPos)

		if DelPos1 == MAX:
			V1 = Vstd
			A1 = Astd
			print("Servo1 is the standard")
			t3_1 = 64.0*V1/A1 + 64.0*DelPos1/V1
			t1_std = 64*Vstd/Astd
			t3_tri = 2*t1_std
			t2_1 = 64.0*DelPos1/V1
			t3_std = t3_1
			t2_std = t2_1
			#print("t1_std: %f" %t1_std)
			#print("t3: %f" %t3_std)
			#print("t3_tri: %f" %t3_tri)
			#print("DelPos1: %f" %DelPos1)
			#print("DelPos2: %f" %DelPos2)
			#print("DelPos3: %f" %DelPos3)
			t3_2 = t3_std
			t3_3 = t3_std
			t2_2 = t2_std
			t2_3 = t2_std
			den_std = (t3_std - t2_std)
			V2 = 64.0*DelPos2/t2_2
			V3 = 64.0*DelPos3/t2_3
			A2 = 64*V2 / den_std
			A3 = 64*V3 / den_std

		elif DelPos2 == MAX:
			V2 = Vstd
			A2 = Astd
			print("Servo2 is the standard")
			t3_2 = 64.0*V2/A2 + 64.0*DelPos2/V2
			t1_std = 64*Vstd/Astd
			t3_tri = 2*t1_std
			t2_2 = 64.0*DelPos2/V2
			t3_std = t3_2
			t2_std = t2_2
			#print("t1_std: %f" %t1_std)
			#print("t3: %f" %t3_std)
			#print("t3_tri: %f" %t3_tri)
			#print("DelPos1: %f" %DelPos1)
			#print("DelPos2: %f" %DelPos2)
			#print("DelPos3: %f" %DelPos3)
			t3_1 = t3_std
			t3_3 = t3_std  
			t2_1 = t2_std
			t2_3 = t2_std
			den_std = (t3_std - t2_std)
			V1 = 64.0*DelPos1/t2_1
			V3 = 64.0*DelPos3/t2_3
			A1 = 64*V1 / den_std
			A3 = 64*V3 / den_std

		elif DelPos3 == MAX:
			V3 = Vstd
			A3 = Astd
			print("Servo3 is the standard")
			t3_3 = 64.0*V3/A3 + 64.0*DelPos3/V3
			t1_std = 64*Vstd/Astd
			t3_tri = 2*t1_std
			t2_3 = 64.0*DelPos3/V3
			t3_std = t3_3
			t2_std = t2_3
			#print("t1_std: %f" %t1_std)
			#print("t3: %f" %t3_std)
			#print("t3_tri: %f" %t3_tri)
			#print("DelPos1: %f" %DelPos1)
			#print("DelPos2: %f" %DelPos2)
			#print("DelPos3: %f" %DelPos3)
			t3_1 = t3_std
			t3_2 = t3_std
			t2_1 = t2_std
			t2_2 = t2_std
			den_std = (t3_std - t2_std)
			V1 = 64.0*DelPos1/t2_1
			V2 = 64.0*DelPos2/t2_2
			A1 = 64*V1 / den_std
			A2 = 64*V2 / den_std

		return V1, A1, V2, A2, V3, A3

	def DeltaGoHome(self):

		self.SetProfile1(30,5)
		self.SetProfile2(30,5)
		self.SetProfile3(30,5)

		pos1 = 90
		pos2 = 90
		pos3 = 90

		servo_ang1 = self.map(pos1, 0.0, 360.0, 0, 4095)
		servo_ang2 = self.map(pos2, 0.0, 360.0, 0, 4095)
		servo_ang3 = self.map(pos3, 0.0, 360.0, 0, 4095)

		dxl1_goal_position = servo_ang1
		dxl2_goal_position = servo_ang2
		dxl3_goal_position = servo_ang3

		dxl_comm_result1, dxl_error1 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
		dxl_comm_result2, dxl_error2 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))
		dxl_comm_result3, dxl_error3 = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, int(dxl3_goal_position))

		Moving1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING)
		Moving2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING)
		Moving3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING)
		time.sleep(0.5)

		while((Moving1==1) and (Moving2==1) and (Moving3==1)):
			Moving1, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL1_ID, self.ADDR_PRO_MOVING)
			Moving2, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL2_ID, self.ADDR_PRO_MOVING)
			Moving3, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL3_ID, self.ADDR_PRO_MOVING)

	def DeltaGoCircle(self,Radius,Height):

		zeta = [None]*360
		X = [None]*360
		Y1 = [None]*180
		Y2 = [None]*180
		Y = [None]*360
		DEG1 = [None]*360
		DEG2 = [None]*360
		DEG3 = [None]*360

		## Generate X,Y coordinate for circle
		for i in range(1,361):
		    zeta[i-1] = i*self.deg2rad
		    X[i-1] = Radius*math.cos(zeta[i-1])

		for i in range(1,(len(X)/2)+1):
		    Y1[i-1] = math.sqrt(Radius**2 - X[i-1]**2)

		j = 0
		for i in range((len(X)/2)+1,len(X)+1):
		    Y2[j] = -math.sqrt(Radius**2 - X[j]**2)
		    j = j+1

		Y = numpy.concatenate((Y1,Y2))

		## Generae servo angle data set according to X,Y coordinate
		i = 0
		increment = 5

		while(i<360):
			x = X[i]
			y = Y[i]
			z = Height   

			DEG = self.DeltaINV(x,y,z)
			DEG1[i] = DEG[0]
			DEG2[i] = DEG[1]
			DEG3[i] = DEG[2]


			i = i + 1

		## Run servo in 359 points left

		self.SetProfile1(50,20)
		self.SetProfile2(50,20)
		self.SetProfile3(50,20)
		K = 0
		while K < 360:

			self.RunServo(DEG1[K],DEG2[K],DEG3[K])

			# make a delay for starting point
			if K == 0:
				time.sleep(1.5)

			K = K + increment

		## Finish the whole circle
		self.RunServo(DEG1[359],DEG2[359],DEG3[359])

	def DeltaGoSquare(self, SideLength, Height):
		xLim = SideLength/2
		yLim = xLim

		step_divider = 20
		step = xLim/step_divider
		x = list()
		y = list()

		# first line quadrant 1,2
		i = xLim
		while i >= -xLim:
			x.append(i)
			y.append(yLim)
			# update
			i = i - step

		# second line quadrant 2,3
		j = yLim
		while j >= -yLim:
			x.append(-xLim)
			y.append(j)
			# update
			j = j - step

		# thrid line quadrant 3,4
		k = -xLim
		while k <= xLim:
			x.append(k)
			y.append(-yLim)
			# update
			k = k + step

		#fourth line quadrant 4,1
		l = -yLim
		while l <= yLim:
			x.append(xLim)
			y.append(l)
			# update
			l = l + step

		size = len(x)
		print size
		z = [Height]*size
		DEG1 = [None]*size
		DEG2 = [None]*size
		DEG3 = [None]*size
		m = 0
		# Pre calculate Inverse Kinematics
		for m in range(0,len(x)):
			DEG = self.DeltaINV(x[m],y[m],z[m])
			DEG1[m] = DEG[0]
			DEG2[m] = DEG[1]
			DEG3[m] = DEG[2]

		## Run servo 
		self.SetProfile1(30,10)
		self.SetProfile2(30,10)
		self.SetProfile3(30,10)

		for K in range(0,size):

			self.RunServo(DEG1[K],DEG2[K],DEG3[K])

			# make a delay for starting point
			if K == 0:
				time.sleep(1.5)

	def GetXYZ(self):
		ReadAng = self.ReadAngle()
		ReadAng1 = ReadAng[0]
		ReadAng2 = ReadAng[1]
		ReadAng3 = ReadAng[2]

		XYZ = self.DeltaFWD(ReadAng1,ReadAng2,ReadAng3)
		x = XYZ[0]
		y = XYZ[1]
		z = XYZ[2]

		return x,y,z, ReadAng1,ReadAng2, ReadAng3

	def DeltaGoPoint(self,x,y,z):

		WARN = self.XYZOutRange(x,y,z)
		if not WARN:
			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]

			GoalPos = self.DeltaINV(x,y,z)
			GoalPos1 = GoalPos[0]
			GoalPos2 = GoalPos[1]
			GoalPos3 = GoalPos[2]

			DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
			DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
			DelPos3 = self.DeltaPos(PreAng3,GoalPos3)

			VSTD = 30
			ASTD = 10

			TRAJ = self.TrajectoryGenerationDelta(VSTD,ASTD,DelPos1,DelPos2,DelPos3)

			V1 = TRAJ[0]
			A1 = TRAJ[1]
			V2 = TRAJ[2]
			A2 = TRAJ[3]
			V3 = TRAJ[4]
			A3 = TRAJ[5]

			self.SetProfile1(V1,A1)
			self.SetProfile2(V2,A2)
			self.SetProfile3(V3,A3)
			
			self.RunServo(GoalPos1,GoalPos2,GoalPos3)

			MovingFlag = True
			# This delay would make sure that the moving detection loop will not run too fast than actual motion
			time.sleep(0.1)
			while MovingFlag:
				Move1 = self.IsMoving1()
				Move2 = self.IsMoving2()
				Move3 = self.IsMoving3()

				if Move1 == 0 and Move2 == 0 and Move3 == 0:
					MovingFlag = False
		else:
			print("ERROR: Out of workspace")

	def DeltaKinematics(self):
		try:
			while True:
				XYZ = self.GetXYZ()
				print("===      Present XYZ from FWD      ===")
				print("X: %f" %XYZ[0])
				print("Y: %f" %XYZ[1])
				print("Z: %f" %XYZ[2])
				print("===          Present Angle         ===")
				print("Read Ang1: %f" %XYZ[3])
				print("Read Ang2: %f" %XYZ[4])
				print("Read Ang3: %f" %XYZ[5])
				X = XYZ[0]
				Y = XYZ[1]
				Z = XYZ[2]
				ReadAng1 = XYZ[3]
				ReadAng2 = XYZ[4]
				ReadAng3 = XYZ[5]
				INVDEG = self.DeltaINV(X,Y,Z)
				print("===     Present Angle from INV     ===")
				print("Deg1: %f" %INVDEG[0])
				print("Deg2: %f" %INVDEG[1])
				print("Deg3: %f" %INVDEG[2])
				print("===  Error Angle from calculation  ===")
				errorANG1 = ReadAng1 - INVDEG[0]
				errorANG2 = ReadAng2 - INVDEG[1]
				errorANG3 = ReadAng3 - INVDEG[2]
				print("Error Deg1: %f" %errorANG1)
				print("Error Deg2: %f" %errorANG2)
				print("Error Deg3: %f" %errorANG3)
				print("-------------------------------------")
				time.sleep(0.5)
		except(KeyboardInterrupt, SystemExit):
			print("End program...")


	def DeltaJogLinear(self):
		LinearIncrement = 50
		waitForStart = True
		print("----------------------------------------------------------------------------------")
		print("-----------------------Press Start Button to Jog Linear!--------------------------")
		print("----------------------------------------------------------------------------------")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				startJog = True
			time.sleep(0.1)

		time.sleep(1)
		print("Full Robot is ready to move")
		print("Delta Robot Jog Linear Started...")

		self.SetProfile1(60,15)
		self.SetProfile2(60,15)
		self.SetProfile3(60,15)

		while startJog:

			############ Receive Value From Joy Stick All The Time ###############
			Buttons = self.getButton()
			Z_Btn = Buttons[1] #B
			X_Btn = Buttons[2] #X
			Y_Btn = Buttons[3] #Y
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			Continue_Btn = Buttons[8] #Logiccool
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			########## Reset Jogging and Back to home position ##################

			if Continue_Btn == 1:
				self.DeltaGoHome()

			###################### Exit the program ###########################

			if Back_Btn == 1:
				self.DeltaGoHome()
				print("----------------------------------------------------------------------------------")
				print("-----------------------Exit the Delta Robot Linear Jog Mode-----------------------")
				print("----------------------------------------------------------------------------------")
				break

			if GripOpen_Btn == 1:
				self.GripperOpen()
				time.sleep(0.5)

			if GripClose_Btn == 1:
				self.GripperClose()
				time.sleep(0.5)


			################### Move in X direction #####################
			OnceTrig = True
			while X_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				X_Btn = Buttons[2] #X
				#JogDirX = Hats[0] # Normal = 0, LeftDir Pressed = -1, RightDir Pressed = 1
				Axes = self.getAxis()
				Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				#Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.DeltaFWD(PreDeg1,PreDeg2,PreDeg3)
					Yconst = PreReadXYZ[1]
					Zconst = PreReadXYZ[2]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn


				if abs(Ax0) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.DeltaFWD(Deg1,Deg2,Deg3)
					ReadX = ReadXYZ[0]
					#ReadY = ReadXYZ[1]
					#ReadZ = ReadXYZ[2]
					Xcom = ReadX + (Ax0*LinearIncrement)
					Ycom = Yconst
					Zcom = Zconst
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.DeltaINV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

			################### Move in Y direction #####################
			OnceTrig = True
			while Y_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				Y_Btn = Buttons[3] #Y
				#JogDirYZ = Hats[1] # Normal = 0, DowDir Pressed = -1, UpDir Pressed = 1
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1 

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.DeltaFWD(PreDeg1,PreDeg2,PreDeg3)
					Xconst = PreReadXYZ[0]
					Zconst = PreReadXYZ[2]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn

				if abs(Ax1) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.DeltaFWD(Deg1,Deg2,Deg3)
					#ReadX = ReadXYZ[0]
					ReadY = ReadXYZ[1]
					#ReadZ = ReadXYZ[2]
					Xcom = Xconst
					Ycom = ReadY - (Ax1*LinearIncrement)
					Zcom = Zconst
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.DeltaINV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

			################### Move in Z direction #####################
			OnceTrig = True
			while Z_Btn == 1:
				#Hats = getHat()
				Buttons = self.getButton()
				Z_Btn = Buttons[1] #B
				#JogDirYZ = Hats[1] # Normal = 0, DowDir Pressed = -1, UpDir Pressed = 1
				Axes = self.getAxis()
				#Ax0 = Axes[0]       #Analog left push right = +1,  Analog left push left = -1
				Ax1 = Axes[1]       #Analog left push down = +1,  Analog left push up = -1 

				if OnceTrig == True:
				# make a constant value of Y and Z before jogging because these value don't change anyway
					PreReadDeg = self.ReadAngle()
					PreDeg1 = PreReadDeg[0]
					PreDeg2 = PreReadDeg[1]
					PreDeg3 = PreReadDeg[2]
					PreReadXYZ = self.DeltaFWD(PreDeg1,PreDeg2,PreDeg3)
					Xconst = PreReadXYZ[0]
					Yconst = PreReadXYZ[1]
					OnceTrig = False # it would not come and read this if loop again until release X_Btn


				if abs(Ax1) > 0.0001:
					ReadDeg = self.ReadAngle()
					Deg1 = ReadDeg[0]
					Deg2 = ReadDeg[1]
					Deg3 = ReadDeg[2]
					ReadXYZ = self.DeltaFWD(Deg1,Deg2,Deg3)
					#ReadX = ReadXYZ[0]
					#ReadY = ReadXYZ[1]
					ReadZ = ReadXYZ[2]
					Xcom = Xconst
					Ycom = Yconst
					Zcom = ReadZ - (Ax1*LinearIncrement)
					WARN = self.XYZOutRange(Xcom,Ycom,Zcom)
					InputDeg = self.DeltaINV(Xcom,Ycom,Zcom)
					InputDeg1 = InputDeg[0]
					InputDeg2 = InputDeg[1]
					InputDeg3 = InputDeg[2]

					if not WARN:
						self.RunServo(InputDeg1,InputDeg2,InputDeg3)
						## If there is no warning from XYZOutrange(), so let's drive the sevo ##

	def DeltaTeachPoint(self):

		self.TorqueOff()
		self.TorqueGripperOn()
		time.sleep(0.1)
		self.GripperCheck()

		################## Go to stand by position before starting  ###########################

		waitForStart = True
		print("Press Start Button!")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				startTeach = True

			time.sleep(0.1)

		print("--> You can move robot freely by your hand")
		print("--> Press Logicool button to memorize the position")
		print("--> Press Analog left button for OpenGripper")
		print("--> Press Analog right button for CloseGripper")

		Mem_Ang1 = [None]*10000
		Mem_Ang2 = [None]*10000
		Mem_Ang3 = [None]*10000
		Mem_GripperStatus = [None]*10000
		GripperStatus = 1 # For open at first

		i = 0
		J = 0
		runTeach = False

		while startTeach:

			Buttons = self.getButton()
			Back_Btn = Buttons[6] #Back
			Start_Btn = Buttons[7] #Start
			Memo_Btn = Buttons[8] #Logiccool
			GripOpen_Btn = Buttons[9] # Analog Left Push
			GripClose_Btn = Buttons[10] #Analog Right Push

			ReadANG = self.ReadAngle()
			ANG1 = ReadANG[0]
			ANG2 = ReadANG[1]
			ANG3 = ReadANG[2]

			if GripOpen_Btn == 1:
				self.GripperOpen()
				GripperStatus = 1

			if GripClose_Btn == 1:
				self.GripperClose()
				GripperStatus = 0

			if Memo_Btn == 1:
				ReadANG = self.ReadAngle()
				Mem_Ang1[i] = ReadANG[0]
				Mem_Ang2[i] = ReadANG[1]
				Mem_Ang3[i] = ReadANG[2]
				Mem_GripperStatus[i] = GripperStatus

				print("------------------------------")
				print("------------------------------")
				print("Mem_Ang1: %f" %Mem_Ang1[i])
				print("Mem_Ang2: %f" %Mem_Ang2[i])
				print("Mem_Ang3: %f" %Mem_Ang3[i])
				if GripperStatus == 1:
					print("Gripper Open")
				elif GripperStatus == 0:
					print("Gripper Close")
				else:
					print("No Gripper status...")
				print("------------------------------")
				print("------------------------------")

				i = i + 1
				while Memo_Btn == 1:
					Buttons = self.getButton()
					Memo_Btn = Buttons[8] #Logiccool
					#print("Release Button!")
					#time.sleep(0.5)

				print("Teach Point: %d" %i)

			if Back_Btn == 1:
				for J in range(0,i):
					print("Point: %d" %J)
					print("Mem_Ang1 = %f" %Mem_Ang1[J])
					print("Mem_Ang2 = %f" %Mem_Ang2[J])
					print("Mem_Ang3 = %f" %Mem_Ang3[J])
					print("////////////////////////////")
				startTeach = False
				waitForStart = True

		print("Press Start Button to run Teaching Point")

		while waitForStart:

			Buttons = self.getButton()
			Start_Btn = Buttons[7] #Start

			if Start_Btn == 1:
				waitForStart = False
				runTeach = True

			time.sleep(0.1)

		while runTeach:

			self.TorqueOn()
			print("All Torque is ON")
			time.sleep(1)
			self.DeltaGoHome()
			time.sleep(3)

			PreAng = self.ReadAngle()
			PreAng1 = PreAng[0]
			PreAng2 = PreAng[1]
			PreAng3 = PreAng[2]

			for K in range(0,i):
				startTime = time.time()
				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back

				if Back_Btn == 1:
			 		break

				GoalPos1 = Mem_Ang1[K]
				GoalPos2 = Mem_Ang2[K]
				GoalPos3 = Mem_Ang3[K]

				DelPos1 = self.DeltaPos(PreAng1,GoalPos1)
				DelPos2 = self.DeltaPos(PreAng2,GoalPos2)
				DelPos3 = self.DeltaPos(PreAng3,GoalPos3)

				VSTD = 40
				ASTD = 8

				TRAJ = self.TrajectoryGenerationDelta(VSTD,ASTD,DelPos1,DelPos2,DelPos3)

				V1 = TRAJ[0]
				A1 = TRAJ[1]
				V2 = TRAJ[2]
				A2 = TRAJ[3]
				V3 = TRAJ[4]
				A3 = TRAJ[5]

				self.SetProfile1(V1,A1)
				self.SetProfile2(V2,A2)
				self.SetProfile3(V3,A3)

				self.RunServo(GoalPos1,GoalPos2,GoalPos3)
				startTime = time.time()

				print("Move to point %d" %K )

				MoveType1 = self.MovingStatus1()
				MoveType2 = self.MovingStatus2()
				MoveType3 = self.MovingStatus3()

				MovingFlag = True
				time.sleep(0.5)

				while MovingFlag:
					Move1 = self.IsMoving1()
					Move2 = self.IsMoving2()
					Move3 = self.IsMoving3()

					if Move1 == 0:
						endTime1 = time.time()
						period1 = endTime1 - startTime
					if Move2 == 0:
						endTime2 = time.time()
						period2 = endTime2 - startTime  
					if Move3 == 0:
						endTime3 = time.time()
						period3 = endTime3 - startTime
					if Move1 == 0 and Move2 == 0 and Move3 == 0:
						MovingFlag = False
						#endTime = time.time()
						#period = endTime - startTime
						print("Period1: %f" %period1)
						print("Period2: %f" %period2) 
						print("Period3: %f" %period3)                              
						print("Finished point %d" %K)

				if Mem_GripperStatus[K] == 1:
					self.GripperOpen()
					time.sleep(0.5)
					print("Open Gripper")
				elif Mem_GripperStatus[K] == 0:
					self.GripperClose()
					time.sleep(0.5)
					print("Close Gripper")

				PreAng1 = GoalPos1
				PreAng2 = GoalPos2
				PreAng3 = GoalPos3

			waitForStartAgain = True
			print("Press start for run again")
			print("Press back to exit")
			while waitForStartAgain:
				Buttons = self.getButton()
				Back_Btn = Buttons[6] #Back
				Start_Btn = Buttons[7] #Start
				if Back_Btn == 1:
					waitForStartAgain = False
					runTeach = False
					self.DeltaGoHome()
					#TorqueOff()

				if Start_Btn == 1:
					K = 0
					waitForStartAgain = False