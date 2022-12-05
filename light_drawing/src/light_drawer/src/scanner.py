from skimage.filters import threshold_local
from feature_extractor import FeatureExtractor
import numpy as np
import argparse
import cv2
import imutils

def order_points(pts):
    # initialzie a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype = "float32")
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis = 1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis = 1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    # return the ordered coordinates
    return rect

def four_point_transform(image, pts):
    # obtain a consistent order of the points and unpack them
    # individually
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    # return the warped image
    return warped

if __name__=="__main__":
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    feature_extractor = FeatureExtractor()
    ap.add_argument("-i", "--image", required = False,
        help = "Path to the image to be scanned")
    args = vars(ap.parse_args())
    img_name = ""
    if args["image"] is None:
        print("STEP 0: WEBCAM")
        cv2.namedWindow("preview")
        vc = cv2.VideoCapture(0)
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False
        while rval:
            cv2.imshow("preview", frame)
            rval, frame = vc.read()
            key = cv2.waitKey(1)
            if key == 27: # exit on ESC
                break
            if key == 13:
                # Enter pressed
                img_name = "images/webcam.png"
                cv2.imwrite(img_name, frame)
                print("Image taken!")
                break
        vc.release()
        cv2.destroyWindow("preview")
    else:
        # load the image and compute the ratio of the old height
        # to the new height, clone it, and resize it
        img_name = args["image"]
    
    image = cv2.imread(img_name)
    ratio = image.shape[0] / 500.0
    orig = image.copy()
    image = imutils.resize(image, height = 500)
    # convert the image to grayscale, blur it, and find edges
    # in the image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 75, 200)
    # show the original image and the edge detected image
    print("STEP 1: Edge Detection")
    cv2.imshow("Image", image)
    cv2.imshow("Edged", edged)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # find the contours in the edged image, keeping only the
    # largest ones, and initialize the screen contour
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    colors = feature_extractor.extract_color(image, cnts)

    colored_image = np.zeros(image.shape)
    for i, contour in enumerate(cnts):
        print((colors[i][0], colors[i][1], colors[i][2]))
        cv2.drawContours(colored_image, [contour], -1, (colors[i][0], colors[i][1], colors[i][2]), 2)

    # print("DRAWING COLORED CONTOURS")
    # colored_image = np.zeros(image.shape)
    # for i, contour in enumerate(cnts):
    #     contour = contour.squeeze(1)
    #     # print((colors[i][0], colors[i][1], colors[i][2]))
    #     for point in contour:
    #         x, y = point[0], point[1]
    #         colored_image[y][x][0] = colors[i][0]
    #         colored_image[y][x][1] = colors[i][1]
    #         colored_image[y][x][2] = colors[i][2]
    #         # colored_image[y][x][0] = 255
    #         # colored_image[y][x][1] = 0
    #         # colored_image[y][x][2] = 0
    # print(colored_image.shape, colored_image[:, :, 0].max(),  colored_image[:, :, 0].min())
    cv2.imwrite("test.png", colored_image)

    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:5]
    # loop over the contours
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        # if our approximated contour has four points, then we
        # can assume that we have found our screen
        if len(approx) == 4:
            screenCnt = approx
            break
    # show the contour (outline) of the piece of paper
    print("STEP 2: Find contours of paper")
    cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 2)
    cv2.imshow("Outline", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # apply the four point transform to obtain a top-down
    # view of the original image
    warped = four_point_transform(orig, screenCnt.reshape(4, 2) * ratio)
    # convert the warped image to grayscale, then threshold it
    # to give it that 'black and white' paper effect
    
    # warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    # T = threshold_local(warped, 11, offset = 10, method = "gaussian")
    # warped = (warped > T).astype("uint8") * 255
    # show the original and scanned images
    
    print("STEP 3: Apply perspective transform")
    cv2.imshow("Original", imutils.resize(orig, height = 650))
    cv2.imshow("Scanned", imutils.resize(warped, height = 650))
    cv2.waitKey(0)
