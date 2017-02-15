import cv2, numpy

cv2.namedWindow('image', cv2.WINDOW_NORMAL)
img = cv2.imread("/home/martin/Pictures/mug.jpg", 0)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()


