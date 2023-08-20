# read output.txt
import numpy as np
import arm
import matplotlib.pyplot as plt
import cv2

from model import Model
import os

ARM = 0.1575
m = Model(ARM, ARM)

def read(name):
  with open(f'./{name}', 'r') as f:
    gcode = f.readlines()

  theta = []

  for line in gcode:
    line = line.strip()
    if line == '' or line[0] == '#':
      continue
    theta.append([float(v) for v in line.split(' ')])
  return theta

values = read("output.txt")


if False:
  for v in values:
    if np.random.rand(1)[0] > 0.01:
      continue
    image = m.plot(v[0], v[1])

    # if np.random.rand(1)[0] > 0.1:
    #   continue
    cv2.imshow("Code", image)
    cv2.waitKey(1)


  cv2.imshow("Finish", image)
  cv2.waitKey(0)

K = 200 * 32/ (2.0*np.pi)

values = np.array(values) 

p = np.array(values[:, 0:2]) # / K
v = np.array(values[:, 2:4]) 
a = np.array(values[:, 4:6])
e = np.array(values[:, 6:8]) 
tv = np.array(values[:, 8:10]) 

# e = np.array( values[:, 10:11] ) 

# plt.plot(p[:,0], label="x")
plt.plot(v[:,0], label="vx")
# plt.plot(a[:,0], label="ax")
plt.plot(e[:,0], label="ex")
plt.plot(tv[:,0], label="tvx")

# plt.plot(p[:,1], label="y")
plt.plot(v[:,1], label="vy")
# plt.plot(a[:,1], label="ay")
plt.plot(e[:,1], label="ey")
plt.plot(tv[:,1], label="tvy")

# plt.plot(e, label="e")

print(p.shape)
# print(e.sum() / e.shape[0])
plt.legend()
plt.show()