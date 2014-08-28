import cv
import sys
img = cv.LoadImage('gente.jpg')
hog = HOGDescriptor()
hog.setSVMDetector(HOGDescriptor.getDefaultPeopleDetector())
for r in hog.detectMultiScale(img, 0, Size(8,8), Size(24,16), 1.05, 2):
    r.x += round(r.width*0.1)
    r.y += round(r.height*0.1)
    r.width = round(r.width*0.8)
    r.height = round(r.height*0.8)
    rectangle(img, r.tl(), r.br(), Scalar(0,255,0), 1)

namedWindow("people detector", 1)
imshow("people detector", img)
waitKey(0)
