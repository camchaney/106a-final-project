import cv2

image = cv2.imread('images/luffy.jpg')
print("Size of image before pyrDown: ", image.shape)

image = cv2.pyrDown(image)
print("Size of image after pyrDown: ", image.shape)
cv2.imshow('DownSample', image)