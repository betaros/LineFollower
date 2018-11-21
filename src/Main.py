# https://gist.github.com/flyboy74/2176de46d5777dafa6fcee891d230637
import math

import cv2


def force_odd_number(num):
    if num % 2 == 0:
        return num + 1
    else:
        return num


def ceil(num):
    return int(math.ceil(num))

def detect_line():
    # parameters
    width = 300
    height = 200
    road_darkest_gray = 100
    road_edge_threshold = 10

    # calculated params
    width_half = int(width / 2)
    typical_road_width = width_half
    min_road_width = ceil(typical_road_width / 4)
    road_detect_step_size = ceil(height / 10)

    median_blur_kernel = force_odd_number(int(width / 10))
    gaussian_blur_kernel = force_odd_number(int(width / 30))

    # camera = cv2.VideoCapture(0)
    while True:
        # ret_val, image = camera.read()
        # image = cv2.imread("../images/sample-curve.jpg")
        image = cv2.imread("../images/sample-straight.jpg")
        image = cv2.resize(image, (width, height))
        out = image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # kernel = np.ones((3, 3), np.uint8)
        # image = cv2.erode(image, kernel, iterations=5)
        # image = cv2.dilate(image, kernel, iterations=8)
        image = cv2.medianBlur(image, median_blur_kernel, 0)
        image = cv2.inRange(image, 0, road_darkest_gray)
        # image = cv2.GaussianBlur(image, (gaussian_blur_kernel, gaussian_blur_kernel), 0)
        out = image

        # Line detection
        # image = cv2.Canny(image, 0, 0, apertureSize=3)

        # find road boundaries at bottom of image
        bottom = height-1
        top = height-1 - road_detect_step_size*3
        left_edge = right_edge = width_half
        # cv2.line(out, (left_edge, bottom), (right_edge, bottom), 100)

        old_y = bottom
        # for y in range(bottom, 0, -road_detect_step_size):
        for y in range(bottom, top, -road_detect_step_size):
            old_left_edge = left_edge
            old_right_edge = right_edge
            left_edge = detect_edge(image, road_edge_threshold, left_edge, y, -1)
            right_edge = detect_edge(image, road_edge_threshold, right_edge, y, +1)

            if right_edge - left_edge < min_road_width:
                break

            cv2.line(out, (old_left_edge-5, old_y), (left_edge-5, y), 100)
            cv2.line(out, (old_right_edge+5, old_y), (right_edge+5, y), 100)
            old_y = y

        middle = int((right_edge + left_edge) / 2)
        cv2.line(out, (middle, y), (width_half, y+5), 50)





        # out = cv2.resize(out, (width*10, height*10))
        cv2.imshow("original with line", out)

        # Exit when Escape is pressed
        if cv2.waitKey(1) == 27:
            break


def detect_edge(image, threshold, x, y, direction):
    width = len(image[0])

    if image[y, x] > threshold:
        while image[y, x] > threshold:
            x += direction
            if 0 > x or x > width:
                return x - direction
        return x

    else:
        while image[y, x] <= threshold:
            x -= direction
            if 0 > x or x > width:
                return x + direction
        return x + direction


detect_line()
