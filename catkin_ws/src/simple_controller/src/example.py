

#vector x
x = [5,6,7,8,9]

for element in x:
    print (element) # this will stop at 9

stop = x.index(7)
i=0
goal_x_coordinates = [0.0, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
goal_y_coordinates = [-4.0, 1.0, 1.5,  1.0, -3.0, -4.0, 0.0]
while i<stop:
    print(x[i])
    