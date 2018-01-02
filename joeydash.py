import cv2
import numpy as np


def set_point_to_displace(height_queue, points_to_displace_queue):
    cap = cv2.VideoCapture(0)
    while True:
        cam_constant = 557
        if height_queue.empty():
            pass
        else:
            cam_height = height_queue.get()
            print(cam_height)
            lower_colour = np.array([20, 45, 80])
            upper_colour = np.array([46, 130, 220])
            point_to_displace = np.zeros(2)
            _, frame = cap.read()
            height, width = frame.shape[:2]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            blur = cv2.GaussianBlur(hsv, (15, 15), 0)
            mask = cv2.inRange(blur, lower_colour, upper_colour)
            _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours):
                filter(lambda a: cv2.contourArea(a) > 500, contours)
                c = sorted(contours, key=cv2.contourArea)[::-1]
                for contour in c[0:1:]:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.circle(frame, ((2 * x + w) / 2, (2 * y + h) / 2), 2, (0, 0, 255), -1)
                    cv2.circle(frame, (width / 2, height / 2), 2, (0, 0, 255), -1)
                    cv2.imshow("frame", frame)
                    x_displacement_in_pixels = ((2 * x + w) / 2) - (width / 2)
                    y_displacement_in_pixels = (height / 2) - ((2 * y + h) / 2)
                    point_to_displace = [(x_displacement_in_pixels * cam_height) / cam_constant,
                                         (y_displacement_in_pixels * cam_height) / cam_constant]
            else:
                pass
            if points_to_displace_queue.full():
                points_to_displace_queue.clear()
                points_to_displace_queue.put(point_to_displace)
            else:
                points_to_displace_queue.put(point_to_displace)
