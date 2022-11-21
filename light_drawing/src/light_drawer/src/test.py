import os
import cv2
from feature_extractor import FeatureExtractor
# from path_planner import PathPlanner


if __name__=="__main__":
    cwd = os.path.realpath(os.path.dirname(__file__))
    files = [filename for filename in os.listdir(cwd + "/images") if ".jpg" in filename]
    print("\n\nInput integer of image you want: \n")
    for i in range(len(files)):
        print(f"{i}: {files[i]}")
    selection = int(input("\n..."))
    selection = int(selection)
    img = cv2.imread(cwd + "/images/" + files[selection])
    feature_extractor = FeatureExtractor()
    img = feature_extractor.resize_img(img)
    feature_extractor.draw_image(img, "original")
    x = img.shape[0]
    y = img.shape[1]
    contour_img = feature_extractor.create_empty_img(x, y)
    contours, hierarchy = feature_extractor.extract_contour(img)
    contours = feature_extractor.filter_contours_by_len(contours)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    feature_extractor.draw_image(feature_extractor.invert_black_white(contour_img), "contours")

    