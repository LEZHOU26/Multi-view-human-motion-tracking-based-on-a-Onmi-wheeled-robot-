#! /usr/bin/env python
import rospy
from math import *
import numpy as np
import random
x = []
for i in range(100000):
    a = np.random.normal(0,0.1)
    tempa = str(a) + '\n'
    x.append(tempa)

with open('noise_x.txt', 'w') as f:
        for item in x:
            f.write(item)

x = []
with open('noise_y.txt', 'r') as f:
        data = f.readlines() #this returns data as a string of numbers
        for num_char in data:
            x.append(float(num_char))
print(x[2])

