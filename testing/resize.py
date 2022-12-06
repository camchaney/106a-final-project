import cv2
 
img = cv2.imread('images/golden_gate.jpg', cv2.IMREAD_UNCHANGED)
 
print('Original Dimensions : ',img.shape)
 
# scale = 0.6 # ratio of original size
des_width = 26
scale = des_width / img.shape[0]
#print(scale_percent)
width = int(img.shape[0] * scale)
height = int(img.shape[1] * scale)
dim = (width, height)
#print(width)

# resize image
resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

print(resized[:,0])
 
print('Resized Dimensions : ',resized.shape)
 
cv2.imshow("Resized image", resized)
cv2.waitKey(0)
cv2.destroyAllWindows()