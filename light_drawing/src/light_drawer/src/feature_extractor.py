import cv2
import numpy as np
import os

MIN_CONTOUR_LEN = 100
MAX_IMAGE_HEIGHT = 500
THRESHOLD_SAMPLE_SIZE = 201

def extract_contour(img):
    grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayscale = cv2.blur(grayscale, (5, 5))
    thresh_img = cv2.adaptiveThreshold(
        grayscale,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        THRESHOLD_SAMPLE_SIZE,
        0,
    )
    contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours, hierarchy

def filter_contours_by_len(contours):
    new_contours = []
    for contour in contours:
        if cv2.arcLength(contour, True) > MIN_CONTOUR_LEN:
            new_contours.append(contour)
    return tuple(new_contours)

def resize_img(img):
    x = img.shape[0]
    y = img.shape[1]
    if y < MAX_IMAGE_HEIGHT:
        return img
    else:
        scale = MAX_IMAGE_HEIGHT / y
        return cv2.resize(img, (int(y * scale), int(x * scale)))

def invert_black_white(img):
    return abs(img - 255)

def create_empty_img(x, y):
    return np.ones((x, y)) * 255

def draw_image(img, name):
    cv2.imshow(name, img)
    cv2.waitKey(0)

if __name__=="__main__":
    cwd = os.path.realpath(os.path.dirname(__file__))
    files = [filename for filename in os.listdir(cwd) if ".jpg" in filename]
    print("\n\nInput integer of image you want: \n")
    for i in range(len(files)):
        print(f"{i}: {files[i]}")
    selection = int(input("\n..."))
    selection = int(selection)
    img = cv2.imread(cwd + "/" + files[selection])
    img = resize_img(img)
    draw_image(img, "original")
    x = img.shape[0]
    y = img.shape[1]
    contour_img = create_empty_img(x, y)
    contours, hierarchy = extract_contour(img)
    contours = filter_contours_by_len(contours)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    draw_image(invert_black_white(contour_img), "contours")