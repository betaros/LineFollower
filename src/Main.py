#!/usr/bin/env python

# https://gist.github.com/flyboy74/2176de46d5777dafa6fcee891d230637
import math
import sys

import cv2
import numpy as np
import roslib

import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32


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
road_edge_threshold = int(0xFF/2)
road_is_black = False
road_detect_max_steps = 6
output_precision = 180  # output is scaled from 0 (far left) to output_precision (far right)

# calculated params
width_half = int(width / 2)
typical_road_width = width_half
min_road_width = ceil(typical_road_width / 4)
road_detect_step_size = ceil(height / 20)

median_blur_kernel = force_odd_number(int(width / 20))
gaussian_blur_kernel = force_odd_number(int(width / 30))

# constants
TO_LEFT = -1
TO_RIGHT = +1

detection_publisher = image_publisher = None


def init():
    global detection_publisher, image_publisher

    roslib.load_manifest('lane_assist')
    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback)
    rospy.loginfo("Subscribed to /raspicam_node/image/compressed")

    detection_publisher = rospy.Publisher("/lane_assist/detected", Int32, queue_size=10)
    rospy.loginfo("Publishing /lane_assist/detected")

    image_publisher = rospy.Publisher("/traffic_sign/image/compressed", CompressedImage, queue_size=10)
    rospy.loginfo("Publishing /lane_assist/image/compressed")


def callback(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # OpenCV >= 3.0:

    detect_line(img)


def detect_line(image, is_test=False):
    # camera = cv2.VideoCapture(0)
    # while True:
    # ret_val, image = camera.read()

    # image = cv2.imread("../images/sample-straight.jpg")
    # image = cv2.imread("../images/new-curve.jpg")

    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)

    if not is_test:
        image = cv2.rotate(image, cv2.ROTATE_180)


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
    top = height-1 - road_detect_step_size * road_detect_max_steps
    left_edge = right_edge = width_half
    # print("bottom=%d, top=%d, step=%d" % (bottom, top, road_detect_step_size))
    old_y = bottom
    did_detect_road = False
    for y in range(bottom, top, -road_detect_step_size):
        new_left_edge = detect_edge(image, left_edge, y, TO_LEFT)
        new_right_edge = detect_edge(image, right_edge, y, TO_RIGHT)
        # print("checking y=%d, left=%s, right=%s  =>  left=%s, right=%s" % (y, left_edge, right_edge, new_left_edge, new_right_edge))

        if new_left_edge is None or new_right_edge is None:
            continue

        # if right_edge - left_edge < min_road_width:
        #     break

        middle = ceil((new_left_edge + new_right_edge) / 2)
        # print("middle=%d; delta=%d" % (middle, middle - width_half))
        if not did_detect_road:
            # lowest road bounds
            # report delta
            if not is_test:
                angle = int((middle - width_half) * output_precision / width)
                detection_publisher.publish(angle)

                # DEBUG LOG ANGLE
                scale = 1
                angle_degree = int(middle * 180 * scale / width)
                angle_mid = int(90 * scale)
                angle_end = int(180 * scale)

                # print("angle: %d" % (angle_degree))
                sys.stdout.write("(")
                for x in range(0, min(angle_degree, angle_mid)):
                    sys.stdout.write(" ")
                for x in range(angle_degree, angle_mid):
                    sys.stdout.write("<")
                sys.stdout.write("-")
                for x in range(angle_mid, angle_degree):
                    sys.stdout.write(">")
                for x in range(max(angle_degree, angle_mid), angle_end):
                    sys.stdout.write(" ")
                print(") %d\n" % int(angle_degree/scale))
            did_detect_road = True
            # break
        else:
            old_middle = ceil((left_edge + right_edge) / 2)
            cv2.line(out, (left_edge + 5, old_y), (new_left_edge + 5, y), 100)
            cv2.line(out, (right_edge - 5, old_y), (new_right_edge - 5, y), 100)

            cv2.line(out, (old_middle, old_y), (middle, y), 100)
        old_y = y

        left_edge = new_left_edge
        right_edge = new_right_edge

    # print("image[x=150, y=199] is_road = %s" % (is_pixel_on_road(image, width_half, bottom)))
    # print("image[x=20,  y=199] is_road = %s" % (is_pixel_on_road(image, +20, bottom)))
    # print("image[x=290, y=199] is_road = %s" % (is_pixel_on_road(image, width-10, bottom)))
    # for x in range(0, width - 1):
    #     if is_pixel_on_road(image, x, bottom):
    #         sys.stdout.write("X")
    #     else:
    #         sys.stdout.write("-")
    # sys.stdout.write("\n")


    if did_detect_road:
        # road was detected
        middle = int((right_edge + left_edge) / 2)
        cv2.line(out, (middle, old_y - 5), (width_half, old_y + 5), 50)

    else:
        # detected no road at all
        if not is_test:
            detection_publisher.publish(-1)
            print("/!\\ no road detected")

    if not is_test:
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', out)[1]).tostring()

        # Publish new image
        image_publisher.publish(msg)

    if is_test:
        # out = cv2.resize(out, (width*10, height*10))
        cv2.imshow("original with line", out)

        while True:
            # Exit when Escape is pressed
            if cv2.waitKey(1) == 27:
                break


def is_pixel_on_road(image, x, y):
    pixel_is_dark = (image[y, x] != 0)
    return pixel_is_dark == road_is_black


def detect_edge(image, x, y, direction):
    if is_pixel_on_road(image, x, y):
        # we're not on the road, move away until we hit the edge
        while is_pixel_on_road(image, x, y):
            # print("> %d / %d" % (x, width))
            x += direction
            if not (0 <= x < width):
                return None
        return x - direction
    else:
        # we're not on the road / on the edge marker, moving closer to center until reaching the road
        while not is_pixel_on_road(image, x, y):
            # print("< %d / %d" % (x, width))
            x -= direction
            if not (0 <= x < width):
                return None
        return x + direction


# image = cv2.imread("../images/sample-straight.jpg")
# image = cv2.imread("../images/new-curve.jpg")
# detect_line(image)


if __name__ == '__main__':
    init()
    rospy.init_node('lane_assist', log_level=rospy.DEBUG)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down lane_assist")
cv2.destroyAllWindows()
