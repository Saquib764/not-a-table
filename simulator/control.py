import numpy as np
import arm
import matplotlib.pyplot as plt
import cv2

from model import Model
import os

ARM = 0.1575
m = Model(ARM, ARM)

keypoints = [
    [0, 0],
    [-1.5, 3],
    [0., 0.]
]
def read(name):
  with open(f'../test_designs/{name}', 'r') as f:
    gcode = f.readlines()

  theta = []

  for line in gcode:
    line = line.strip()
    if line == '' or line[0] == '#':
      continue
    theta1, theta2 = line.split(' ')
    theta.append([float(theta1), float(theta2)])
  return theta

keypoints = read("triangle.thr.txt")
count = 0
has_completed_path = False
def loop():
  global count, has_completed_path
  has_reached_target, has_completed_path = arm.follow_trajectory()
  if has_reached_target and count < len(keypoints):
    arm.add_point_to_trajectory(keypoints[count])
    count += 1
  pass

T = 400000.0
i = 0
image = None
while i < T:
  i += 1
  random = np.random.rand(1)
  if random < 0.01 or i % 5 == 0:
    loop()

  if not has_completed_path:
    cur_pos = arm.move()

    if np.random.rand(1)[0] < 0.01:
      image = m.plot(cur_pos[0], cur_pos[1])

      cv2.imshow("Code", image)
      cv2.waitKey(1)

cv2.imshow("Finish", image)
cv2.waitKey(0)

p = np.array(arm.p) / arm.N
v = np.array(arm.v) * 10/ arm.N
mv = np.array(arm.mv) * 10/ arm.N
a = np.array(arm.a) * 8/ arm.N

e = np.array(arm.e) * 0.01

plt.plot(p[:,0], label="x")
plt.plot(v[:,0], label="vx")
plt.plot(mv[:,0], label="mvx")
# plt.plot(a[:,0], label="ax")

plt.plot(p[:,1], label="y")
plt.plot(v[:,1], label="vy")
plt.plot(mv[:,1], label="mvy")
# plt.plot(a[:,1], label="ay")

plt.plot(e, label="e")

print(arm.total_error / T)
plt.legend()
plt.show()