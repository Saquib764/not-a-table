import cv2
import numpy as np

class Model:
  def __init__(self, ARM1=0.25, ARM2=0.25):
    self.ARM1 = ARM1
    self.ARM2 = ARM2
    self.R = 0.5
    self.scale = 1500.0
    self.cx = int(self.R * self.scale)
    self.cy = int(self.R * self.scale)

    self.arm1_color = (255, 0, 0)
    self.arm2_color = (0,0,255)
    self.drawing = np.zeros((2 * self.cx, 2 * self.cy, 3), np.uint8) * 255

  def plot(self, theta1, theta2):
    # angles in radians
    theta_dash = theta2/2.0
    theta = theta1 + theta_dash
    rho = self.ARM1 * np.cos(theta_dash) + self.ARM2 * np.cos(theta_dash)

    image = np.ones((2 * self.cx, 2 * self.cy, 3), np.uint8) * 255

    joint_pos = (self.ARM1 * np.cos(theta1), self.ARM1 * np.sin(theta1))

    joint_pos_pixel = (self.cx + self.scale * joint_pos[0], self.cy - self.scale * joint_pos[1])
    cv2.line(image, (int(self.cx), int(self.cy)), (int(joint_pos_pixel[0]), int(joint_pos_pixel[1])), self.arm1_color, 12)

    end_effector = (rho * np.cos(theta), rho * np.sin( theta))
    end_effector_pixel = (self.cx + self.scale * end_effector[0], self.cy - self.scale * end_effector[1])
    cv2.line(image, (int(joint_pos_pixel[0]), int(joint_pos_pixel[1])), (int(end_effector_pixel[0]), int(end_effector_pixel[1])), self.arm2_color, 12)

    cv2.circle(self.drawing, (int(end_effector_pixel[0]), int(end_effector_pixel[1])), 1, (255,5, 0), -1)

    image[np.where(self.drawing[:,:,1] > 0)] = self.drawing[np.where(self.drawing[:,:,1] > 0)]
    return image


