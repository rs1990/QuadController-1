TODO
 - Create launch file
 - Create interfacing program
	- Read state
		- Nav Date
			- topic name: /ardrone/navdata
			- rosmsg: ardrone_autonomy/Navdata
		- IMU Data
			- topic name: /ardrone/imu
			- rosmsg: sensor_msgs/Imu
		- Mag Data
			- topic name: /ardrone/mag
			- romsg: geometry_msgs/Vector3Stamped
	- Reformat to Eigen Matrix
		State index map
			x_1: X
			x_2: Y
			x_3: Z
			x_4: Phi
			x_5: Theta
			x_6: Psi
			x_7: X dot
			x_8: Y dot
			x_9: Z dot
			x_10: Phi dot
			x_11: Theta dot
			x_12: Psi dot
			x_13: State dependent distrubances including aerodynamic nonlinearities for linear position[1]
			x_14: State dependent distrubances including aerodynamic nonlinearities for linear position[2]
			x_15: State dependent distrubances including aerodynamic nonlinearities for linear position[3]
			x_16: State dependent distrubances including aerodynamic nonlinearities for angular position[1]
			x_17: State dependent distrubances including aerodynamic nonlinearities for angular position[2]
			x_18: State dependent distrubances including aerodynamic nonlinearities for angular position[3]
			x_19: Sliding mode errors for linear position[1](Should be integration)
			x_20: Sliding mode errors for linear position[2]
			x_21: Sliding mode errors for linear position[3]
			x_22: Sliding mode errors for angular position[1]
			x_23: Sliding mode errors for angular position[2]
			x_24: Sliding mode errors for angular position[3]
			x_25-55: Weights for neural network in the form of a vector 
				for the first neural network. Converted into a matrix 
				later. The size is determined by the number of layers in
				the neural net. 
			x_55-85: Weights for neural network in the form of a vector 
				for the second neural network. Converted into a matrix 
				later. The size is determined by the number of layers in
				the neural net. 
		In the matlab code there are some state parameters that are used to
		build the actual state vector. As far as I can tell the actual 
		plant information comes from 4 different varaibles: zeta, eta, 
		zetadot, etadot.
			- Zeta:
				X: current x position
				Y: current y position
				Z: current z position
			- Eta: 
				Theta: current pitch 
				Phi: current roll
				Psi: current yaw
			- ZetaDot:
				X dot: current x velocity
				Y dot: current y veloctiy
				Z dot: current z velocity
			- EtaDot:
				Theta dot: Pitch angular velocity
				Phi dot: Roll angular velocity
				Psi dot: Yaw angular velocity
		These only account for 12 of the 23 other states. 11 states are unknown
	- process into controller
	- get output
	- send output as command velocity
	
DEMO
- Hold position: Move ardrone around and see if it comes back to it's original position. 


Plot
- Position
- Position error
- Angular Position
- Angular Position error
