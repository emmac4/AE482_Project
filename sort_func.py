import numpy as np
from scipy.linalg import expm
PI = np.pi

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 0.390],[0, 0, -1, 0.401],[1, 0 ,0, 0.2175],[0, 0, 0, 1]])
	w1 = np.array([[0,0,1]])
	w2 = np.array([[0,1,0]])
	w3 = np.array([[0,1,0]])
	w4 = np.array([[0,1,0]])
	w5 = np.array([[1,0,0]])
	w6 = np.array([[0,1,0]])

	q1 = np.array([[-0.150,0.150,0]])
	q2 = np.array([[-0.150,0.150,0.164]])
	q3 = np.array([[0.094,0,0.164]])
	q4 = np.array([[0.307, 0, 0.164]])
	q5 = np.array([[0,0.260,0.164]])
	q6 = np.array([[0.390,0,0.164]])

	v1 = np.cross(-w1, q1)
	v2 = np.cross(-w2, q2)
	v3 = np.cross(-w3, q3)
	v4 = np.cross(-w4, q4)
	v5 = np.cross(-w5, q5)
	v6 = np.cross(-w6, q6)
	
	W1=np.array([[0, -w1[0][2], w1[0][1]], [w1[0][2], 0, -w1[0][0]], [-w1[0][1], w1[0][0], 0]])
	W2=np.array([[0, -w2[0][2], w2[0][1]], [w2[0][2], 0, -w2[0][0]], [-w2[0][1], w2[0][0], 0]])
	W3=np.array([[0, -w3[0][2], w3[0][1]], [w3[0][2], 0, -w3[0][0]], [-w3[0][1], w3[0][0], 0]])
	W4=np.array([[0, -w4[0][2], w4[0][1]], [w4[0][2], 0, -w4[0][0]], [-w4[0][1], w4[0][0], 0]])
	W5=np.array([[0, -w5[0][2], w5[0][1]], [w5[0][2], 0, -w5[0][0]], [-w5[0][1], w5[0][0], 0]])
	W6=np.array([[0, -w6[0][2], w6[0][1]], [w6[0][2], 0, -w6[0][0]], [-w6[0][1], w6[0][0], 0]])

	bottom_row = np.array([[0,0,0,0]])

	S1 = np.vstack((np.hstack((W1, np.transpose(v1))),bottom_row))
	S2 = np.vstack((np.hstack((W2, np.transpose(v2))),bottom_row))
	S3 = np.vstack((np.hstack((W3, np.transpose(v3))),bottom_row))
	S4 = np.vstack((np.hstack((W4, np.transpose(v4))),bottom_row))
	S5 = np.vstack((np.hstack((W5, np.transpose(v5))),bottom_row))
	S6 = np.vstack((np.hstack((W6, np.transpose(v6))),bottom_row))


	S = [S1, S2, S3, S4, S5, S6]




	
	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	# theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

	thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
	M,S = Get_MS()

	T = np.eye(4)
	for i in range(len(thetas)):
		T = np.matmul(T, expm(S[i]*thetas[i]))

	T = np.matmul(T, M)

	# ==============================================================#
	print(str(np.round(T, 4)) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
    L1 = 0.152
    L2 = 0.120
    L3 = 0.244
    L4 = 0.093
    L5 = 0.213
    L6 = 0.083
    L7 = 0.083
    L8 = 0.082
    L9 = 0.0535
    L10 = 0.059
    OFF = 0.027
    
    Xgrip = xWgrip + 0.15
    Ygrip = yWgrip - 0.15
    Zgrip = zWgrip - 0.012
    yaw = yaw_WgripDegree * np.pi/180
    

    Xcen = Xgrip-L9*np.cos(yaw)
    Ycen = Ygrip-L9*np.sin(yaw)
    Zcen = Zgrip
    

    theta1=np.arctan2(Ycen, Xcen) - np.arcsin((L2-L4+L6)/np.sqrt(Xcen**2+Ycen**2))
    theta6=PI/2-yaw+theta1
        
    T = np.array([[np.cos(theta1), -1*np.sin(theta1), 0, Xcen],[np.sin(theta1), np.cos(theta1), 0, Ycen],[0,0,1,Zcen],[0,0,0,1]])
    v = np.array([-L7, -(L6+OFF), L8+L10, 1])
    
    Tv = np.matmul(T, v)
    X3end = Tv[0]
    Y3end = Tv[1]
    Z3end = Tv[2]
    
    D = np.sqrt(X3end**2 + Y3end**2 + (Z3end-L1)**2)
    alpha = np.arcsin((Z3end-L1)/D)
    
    theta2 = -1*(np.arccos((L5**2 - L3**2 - D**2)/(-2*L3*D))+alpha)
    theta4 = -1*(np.arccos((L3**2 - L5**2 - D**2)/(-2*L5*D)) - alpha)
    theta3 = np.abs(theta2) + np.abs(theta4)
    theta5 = -1*PI/2
    
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)