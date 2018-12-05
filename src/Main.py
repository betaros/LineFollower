# https://gist.github.com/flyboy74/2176de46d5777dafa6fcee891d230637
import math

import cv2
from cv2.cv2 import INTER_LINEAR


def force_odd_number(num):
    if num % 2 == 0:
        return num + 1
    else:
        return num


def ceil(num):
    return int(math.ceil(num))


# parameters
width = 300
height = 200
road_darkest_gray = 120
road_edge_threshold = 10
road_edge_threshold_is_upper_limit = True
road_detect_max_steps = 6

# calculated params
width_half = int(width / 2)
typical_road_width = width_half
min_road_width = ceil(typical_road_width / 4)
road_detect_step_size = ceil(height / 20)
threshold_multiplier = road_edge_threshold_is_upper_limit * 2 - 1

median_blur_kernel = force_odd_number(int(width / 20))
gaussian_blur_kernel = force_odd_number(int(width / 30))

# constants
TO_LEFT = -1
TO_RIGHT = +1


def detect_line():
    # camera = cv2.VideoCapture(0)
    # while True:
    # ret_val, image = camera.read()
    # image = cv2.rotate(image, ROTATE_180)

    # image = cv2.imread("../images/sample-straight.jpg")
    image = cv2.imread("../images/new-curve.jpg")
    image = cv2.resize(image, (width, height), interpolation=INTER_LINEAR)
    # out = image
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # kernel = np.ones((3, 3), np.uint8)
    # image = cv2.erode(image, kernel, iterations=5)
    # image = cv2.dilate(image, kernel, iterations=8)
    image = cv2.medianBlur(image, median_blur_kernel, 0)
    image = cv2.inRange(image, 0, road_darkest_gray)
    out = image
    # image = cv2.GaussianBlur(image, (gaussian_blur_kernel, gaussian_blur_kernel), 0)

    # Line detection
    # image = cv2.Canny(image, 0, 0, apertureSize=3)

    # find road boundaries at bottom of image
    bottom = height - 1
    top = 20  # height-1 - road_detect_step_size * road_detect_max_steps
    left_edge = right_edge = width_half
    # cv2.line(out, (left_edge, bottom), (right_edge, bottom), 100)

    old_y = bottom
    print("bottom=%d, top=%d, step=%d" % (bottom, top, road_detect_step_size))
    # for y in range(bottom, 0, road_detect_step_size):
    left_edge = right_edge = None

    for y in range(bottom, top, -road_detect_step_size):
        # print("checking y=%s, left=%s, right=%s" % (y, left_edge, right_edge))
        new_left_edge = detect_edge(image, left_edge or width_half, y, TO_LEFT)
        new_right_edge = detect_edge(image, right_edge or width_half, y, TO_RIGHT)

        if new_left_edge is None or new_right_edge is None:
            continue

        # if right_edge - left_edge < min_road_width:
        #     break

        middle = ceil((new_left_edge + new_right_edge) / 2)
        print("middle=%d; delta=%d" % (middle, middle - width_half))
        if left_edge is None and right_edge is None:
            # lowest road bounds
            # report delta
            pass
        else:
            old_middle = ceil((left_edge + right_edge) / 2)
            cv2.line(out, (left_edge - 5, old_y), (new_left_edge - 5, y), 100)
            cv2.line(out, (right_edge + 5, old_y), (new_right_edge + 5, y), 100)

            cv2.line(out, (old_middle, old_y), (middle, y), 100)
        old_y = y

        left_edge = new_left_edge
        right_edge = new_right_edge

    middle = int((right_edge + left_edge) / 2)
    cv2.line(out, (middle, old_y - 5), (width_half, old_y + 5), 50)

    # out = cv2.resize(out, (width*10, height*10))
    cv2.imshow("original with line", out)

    while True:
        # Exit when Escape is pressed
        if cv2.waitKey(1) == 27:
            break


def is_on_line(image, x, y):
    return image[y, x] * threshold_multiplier > road_edge_threshold * threshold_multiplier


def detect_edge(image, x, y, direction):
    if is_on_line(image, x, y):
        while is_on_line(image, x, y):
            # print("> %d / %d" % (x, width))
            x += direction
            if not (0 <= x < width):
                return None
        return x - direction
    else:
        while not is_on_line(image, x, y):
            # print("< %d / %d" % (x, width))
            x -= direction
            if not (0 <= x < width):
                return None
        return x + direction


detect_line()
