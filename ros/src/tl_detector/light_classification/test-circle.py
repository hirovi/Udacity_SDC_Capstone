import numpy as np 
import cv2


img = cv2.imread('/tmp/tt1.jpg',0)
circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1.2,100,
                            param1=50,param2=30,minRadius=0,maxRadius=0)

if circles is not None:
    print("Achou")
    
else:
    print("Nao achou") 