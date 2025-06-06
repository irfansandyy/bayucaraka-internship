import cv2 as cv

img = cv.imread('D:/Code/python/magangbayucaraka/traffic.jpeg')
if img is None:
	print("Error: Could not load image.")
else:
	cv.imshow('Traffic Image', img)
	cv.waitKey(0)
	cv.destroyAllWindows()
print(img) #BGR bukan RGB

cam = cv.VideoCapture(0)

while(True):
	ret, frame = cam.read()
	if not ret: break
	cv.imshow('WebCam', frame)
	if cv.waitKey(1) == ord('q'): break
cam.release()
cv.destroyAllWindows()
