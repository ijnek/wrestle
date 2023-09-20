import cv2
import numpy as np
import math
from vision_msgs.msg import BoundingBox2D

# https://stackoverflow.com/a/58194879
def white_mask(img):
  img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  lower = np.array([0, 0, 0])
  upper = np.array([90,65,180])
  mask = cv2.inRange(img, lower, upper)
  return mask

def red_mask_low(img):
  img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  lower = np.array([0,70,135])
  upper = np.array([17,255,255])
  mask = cv2.inRange(img, lower, upper)
  return mask

def red_mask_high(img):
  img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  lower = np.array([140,60,50])
  upper = np.array([180,255,255])
  mask = cv2.inRange(img, lower, upper)
  return mask

def yellow_mask(img):
  img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  lower = np.array([16,137,0])
  upper = np.array([36,255,255])
  mask = cv2.inRange(img, lower, upper)
  return mask

def mask_inside_boundary(image, r_mask):
  # This returns an array of r and theta values
  lines = cv2.HoughLines(r_mask, 1, np.pi/180, 50)
  # cv2.imshow("lines", overlay_lines(image, lines))

  mask_outside = np.zeros(r_mask.shape[:2], dtype=np.uint8)

  left_top = [0, 0]
  right_top = [r_mask.shape[1], 0]

  if lines is not None:
    for line in lines:
      for rho,theta in line:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        poly = np.array([left_top, [x1, y1], [x2, y2], right_top])

        # Draw polygon on mask
        poly_mask = np.zeros(r_mask.shape[:2], dtype=np.uint8)
        cv2.fillPoly(poly_mask, pts=[poly], color=(255, 255, 255))
        # cv2.imshow("poly_mask", poly_mask)
        mask_outside = cv2.bitwise_or(mask_outside, poly_mask)

  # cv2.imshow("mask_outside", mask_outside)

  return cv2.bitwise_not(mask_outside)

def overlay_lines(img, lines):
  if lines is None:
    return img
  for line in lines:
    for rho,theta in line:
      a = np.cos(theta)
      b = np.sin(theta)
      x0 = a*rho
      y0 = b*rho
      x1 = int(x0 + 1000*(-b))
      y1 = int(y0 + 1000*(a))
      x2 = int(x0 - 1000*(-b))
      y2 = int(y0 - 1000*(a))

      cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
  return img

def get_largest_countour(mask):
  closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20)))
  contours, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

  contours = sorted(contours, key=cv2.contourArea, reverse=True)
  if len(contours) == 0:
      return None
  return contours[0]

def overlay_countour(img, contour):
  if contour is not None:
    # print("countour: ", contour)
    cv2.drawContours(img, [contour], 0, (0,255,0), 3)
  cv2.imshow("img", img)
  cv2.waitKey(1)

def get_bbox_2d(contour):
    """Get the bounding box of a contour."""
    x,y,w,h = cv2.boundingRect(contour)
    bb = BoundingBox2D()
    bb.center.position.x = float(x)
    bb.center.position.y = float(y)
    bb.size_x = float(w)
    bb.size_y = float(h)
    return bb

counter = 0
def locate_opponent(image):
    global counter
    cv2.imwrite(f"images/{counter}.png", image)
    counter += 1
    w_mask = white_mask(image)
    r_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask_low(image), red_mask_high(image)), yellow_mask(image))
    mask_bound = mask_inside_boundary(image, r_mask)
    final_mask = cv2.bitwise_and(w_mask, mask_bound)
    contour = get_largest_countour(final_mask)
    # overlay_countour(image, contour)
    bbox2d = get_bbox_2d(contour)
    if bbox2d.size_y > 10:
      return bbox2d
    else:
      return None

def locate_opponent_show(image):
  cv2.imshow("image", image)
  w_mask = white_mask(image)
  cv2.imshow("white_mask", w_mask)
  r_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask_low(image), red_mask_high(image)), yellow_mask(image))
  cv2.imshow("red_mask", r_mask)
  mask_bound = mask_inside_boundary(image, r_mask)
  cv2.imshow("mask_bound", mask_bound)
  final_mask = cv2.bitwise_and(w_mask, mask_bound)
  cv2.imshow("final_mask", final_mask)

  contour = get_largest_countour(final_mask)
  overlay_countour(image, contour)

if __name__ == '__main__':
  for i in range(0, 500):
    print(i)
    image = cv2.imread(f'images/{i}.png')
    cv2.waitKey(20)
  cv2.destroyAllWindows()
