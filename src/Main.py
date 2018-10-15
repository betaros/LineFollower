# https://gist.github.com/flyboy74/2176de46d5777dafa6fcee891d230637

import time
import cv2
import numpy as np


def detect_line():
    camera = cv2.VideoCapture(0)

    while True:
        ret_val, image = camera.read()
        image = cv2.resize(image, (640, 360))
        image = cv2.flip(image, 1)

        blackline = cv2.inRange(image, (0, 0, 0), (60, 60, 60))
        kernel = np.ones((3, 3), np.uint8)
        blackline = cv2.erode(blackline, kernel, iterations=5)
        blackline = cv2.dilate(blackline, kernel, iterations=9)
        img_blk, contours_blk, hierarchy_blk = cv2.findContours(blackline.copy(), cv2.RETR_TREE,
                                                                cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, y_min), (w_min, h_min), ang = blackbox
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            setpoint = 320
            error = int(x_min - setpoint)
            ang = int(ang)
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
            cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

        cv2.imshow("orginal with line", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break


detect_line()
