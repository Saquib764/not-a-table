import cv2
import numpy as np
from gcodeparser import GcodeParser

from model import Model
import os

ARM = 0.1575
m = Model(ARM, ARM)



def read(name, save=False):
  with open(f'../test_thr/{name}', 'r') as f:
    gcode = f.readlines()

  theta_rho = []
  max_rho = 0

  for line in gcode:
    line = line.strip()
    if line == '' or line[0] == '#':
      continue
    theta, rho = line.split(' ')
    rho = float(rho)
    if rho > max_rho:
      max_rho = rho
    theta_rho.append([float(theta), rho])

  text = "# sss file"
  theta12 = []
  for theta, rho in theta_rho:
    # print(rho, theta * 180 /np.pi)
    theta1 = 0
    theta2 = 0
    normalised_rho = 0.95*(ARM + ARM) * rho / max_rho

    theta_dash = np.arccos( normalised_rho / (2.0 * ARM) )
    theta1 = theta - theta_dash
    theta2 = 2 * theta_dash

    text = f"{text}\n{'{0:.5f}'.format(theta1)} {'{0:.5f}'.format(theta2)}"
    theta12.append([theta1, theta2])

  if save:
    name_ = "_".join(name.split(' '))
    f = open(f"../output/{name_}.txt", "w")
    f.write(text)
    f.close()
  return theta12



def plot(theta12):
  theta1 = 0
  theta2 = 0
  d_theta = np.pi / (200*4)
  for target1, target2 in theta12:
    print(target1, target2)
    # print(rho, theta * 180 /np.pi)
    
    while abs(theta1 -target1) > 2*d_theta and abs(theta2 - target2) > 2*d_theta:
      n = max(abs(target1 - theta1), abs(target2 - theta2))/d_theta
      theta1 += (target1 - theta1)/n
      theta2 += (target2 - theta2)/n

      if np.random.rand() < 0.1:
        image = m.plot(theta1, theta2)

        cv2.imshow("Code", image)
        cv2.waitKey(1)
    # break


# cv2.waitKey(0)
# cv2.destroyAllWindows()


# Get file names that has .thr from folder ../designs
files = [f for f in os.listdir('../test_thr') if f.endswith('.thr')]
files = ['square.thr']
for file in files:
  print(file)
  theta12 = read(file)
  plot(theta12)



