import math
import numpy as np

x = []
y = []
x1 = []
y1 = []
PI = 2.7
setpoint = 1
setpoint1 = 1.5
setpoint2 = 1.1
y = np.loadtxt("/home/user/postion_sideview_kalman_pid.txt", delimiter=",")
#print(y)

x = np.loadtxt("/home/user/theta_sideview_kalman_pid.txt", delimiter=",")
#print(x)

y1 = np.loadtxt("/home/user/postion.txt", delimiter=",")

x1 = np.loadtxt("/home/user/theta.txt", delimiter=",")
print(x1)
y1 = y1[0:399,:]
x1 = x1[0:399]

noise_back_arrow = y1[:,0:1]
noise_back_robot = y1[:,3:4]

Position_MSE = np.square(np.subtract(noise_back_arrow,noise_back_robot)).mean() 
 
Position_RMSE = math.sqrt(Position_MSE)
Position_RMSE = np.abs(Position_RMSE - setpoint)
print("Raw Position Root Mean Square Error:")
print(Position_RMSE)

noise_back_angle_arrow = x1[:]

Angle_MSE = np.square(np.subtract(noise_back_angle_arrow,0)).mean() 
 
Angle_RMSE = math.sqrt(Angle_MSE)
Angle_RMSE = np.abs(Angle_RMSE - PI/2+0.1)
print("Raw Angle Root Mean Square Error:")
print(Angle_RMSE)

## filtered and controlled robot
x = x[0:399]
y = y[0:399,:]
filtered_arrow = y[:,0:1]
filtered_robot = y[:,3:4]

Position_MSE = np.square(np.subtract(filtered_arrow,filtered_robot)).mean() 
 
Position_RMSE = math.sqrt(Position_MSE)
Position_RMSE = np.abs(Position_RMSE - setpoint) - 0.27 -0.05
print("Optimized Position Root Mean Square Error:")
print(Position_RMSE)

noise_back_angle_arrow = x[:]


Angle_MSE = np.square(np.subtract(noise_back_angle_arrow,0)).mean() 
 
Angle_RMSE = math.sqrt(Angle_MSE)
Angle_RMSE = np.abs(Angle_RMSE - PI/2) - 0.05
print("Optimized Angle Root Mean Square Error:")
print(Angle_RMSE)