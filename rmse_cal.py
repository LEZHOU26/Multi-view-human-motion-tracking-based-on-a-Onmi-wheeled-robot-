import math
import numpy as np

x = []
y = []
PI = 3.14
setpoint = 1
setpoint1 = 1.5
setpoint2 = 1.1
y = np.loadtxt("/home/user/postion_noise.txt", delimiter=",")
#print(y)

x = np.loadtxt("/home/user/postion_kalman_pid.txt", delimiter=",")
#print(x)

y = y[0:599,:]
x = x[0:599,:]

noise_back_arrow = y[:,0:1]
noise_back_robot = y[:,3:4]

Position_MSE = np.square(np.subtract(noise_back_arrow,noise_back_robot)).mean() 
 
Position_RMSE = math.sqrt(Position_MSE)
Position_RMSE = np.abs(Position_RMSE - setpoint1)
print("Position Root Mean Square Error:\n")
print(Position_RMSE)

noise_back_angle_arrow = y[:,2]
noise_back_angle_robot = y[:,5]

Angle_MSE = np.square(np.subtract(noise_back_angle_arrow,noise_back_angle_robot)).mean() 
 
Angle_RMSE = math.sqrt(Angle_MSE)
Angle_RMSE = np.abs(Angle_RMSE - PI)
print("Angle Root Mean Square Error:\n")
print(Angle_RMSE)

## filtered and controlled robot
filtered_arrow = x[:,0:1]
filtered_robot = x[:,3:4]

Position_MSE = np.square(np.subtract(filtered_arrow,filtered_robot)).mean() 
 
Position_RMSE = math.sqrt(Position_MSE)
Position_RMSE = np.abs(Position_RMSE - setpoint2)
print("Position Root Mean Square Error:\n")
print(Position_RMSE)

noise_back_angle_arrow = x[:,2]
noise_back_angle_robot = x[:,5]

Angle_MSE = np.square(np.subtract(noise_back_angle_arrow,noise_back_angle_robot)).mean() 
 
Angle_RMSE = math.sqrt(Angle_MSE)
Angle_RMSE = np.abs(Angle_RMSE - PI)
print("Angle Root Mean Square Error:\n")
print(Angle_RMSE)