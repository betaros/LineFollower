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

    # calculated params
    width_half = int(width / 2)
    typical_road_width = width_half
    min_road_width = ceil(typical_road_width / 4)
    road_detect_step_size = ceil(height / 20)

    median_blur_kernel = force_odd_number(int(width / 10))
    gaussian_blur_kernel = force_odd_number(int(width / 30))

    # camera = cv2.VideoCapture(0)
    while True:
        # ret_val, image = camera.read()
        image = cv2.imread("../images/sample-straight.jpg")
        image = cv2.resize(image, (width, height))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # out = image

        # kernel = np.ones((3, 3), np.uint8)
        # image = cv2.erode(image, kernel, iterations=5)
        # image = cv2.dilate(image, kernel, iterations=8)
        image = cv2.medianBlur(image, median_blur_kernel, 0)
        image = cv2.inRange(image, 0, road_darkest_gray)
        image = cv2.GaussianBlur(image, (gaussian_blur_kernel, gaussian_blur_kernel), 0)

        # Line detection
        out = image
        image = cv2.Canny(image, 0, 0, apertureSize=3)
        # out = np.zeros((width, height, 3), np.uint8)
        # lines = cv2.HoughLinesP(image, cv2.HOUGH_PROBABILISTIC, 1, 0, 5, 0)
        # for x in range(0, len(lines)):
        #     for x1, y1, x2, y2 in lines[x]:
        #         # cv2.line(inputImage,(x1,y1),(x2,y2),(0,128,0),2, cv2.LINE_AA)
        #         pts = np.array([[x1, y1], [x2, y2]], np.int32)
        #         cv2.polylines(out, [pts], True, (255, 100, 100))

        # img_blk, contours_blk, hierarchy_blk = cv2.findContours(
        #     image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        # )
        # for contour in contours_blk:
        #     blackbox = cv2.minAreaRect(contour)
        #     box = cv2.boxPoints(blackbox)
        #     box = np.int0(box)
        #     cv2.drawContours(out, [box], 0, (0, 0, 255), 3)
        # cv2.putText(out, str(len(contours_blk)), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        middle = width_half
        old_y = height - 1
        for y in range(height - 1, 0, -road_detect_step_size):
            left_edge = 0
            right_edge = width

            # find left edge
            for x in range(middle, 0, -1):
                if image[y, x] > 0:
                    left_edge = x
                    break

            # find right edge
            for x in range(middle, width - 1, +1):
                if image[y, x] > 0:
                    right_edge = x
                    break

            cv2.line(out, (left_edge, y), (right_edge, y), 100)

            old_middle = middle
            middle = int((left_edge + right_edge) / 2)
            cv2.line(out, (old_middle, old_y), (middle, y), 220)
            # image[y, middle] = 200

            old_y = y

            if right_edge - left_edge < min_road_width:
                break

        # out = cv2.resize(out, (width*10, height*10))
        cv2.imshow("original with line", out)

        # Exit when Escape is pressed
        if cv2.waitKey(1) == 27:
            break


detect_line()
