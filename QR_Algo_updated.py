import cv2
import numpy as np
import argparse
import sys

from operator import itemgetter
from glob import glob
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(
'udpsrc port=5600 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264"'
' ! rtph264depay'
' ! avdec_h264'
' ! videoconvert'
' ! appsink')
while True:
    success,img = cap.read()
    decoder = cv2.QRCodeDetector()
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    data, points, _ = decoder.detectAndDecode(img)
    pts5 = np.float32([[100, 100], [600, 100], [100, 600], [600, 600]])  # buyukk yakin ekran

    if points is not None:
        points3 = np.float32(
            [[points[0][0][0], points[0][0][1]], [points[0][1][0], points[0][1][1]], [points[0][3][0], points[0][3][1]],
             [points[0][2][0], points[0][2][1]]])
        matrix = cv2.getPerspectiveTransform(points3, pts5)
        result = cv2.warpPerspective(img, matrix, (800, 800))
        print("PERSPECTIVE TRANSFORMED QR DETECTION")
        data2, points2, _ = decoder.detectAndDecode(result)
        if data2 is not None:
            print('Decoded data: ' + data2)
        cv2.imshow('Detected QR code', result)
    if points is not None:
        print('Decoded data: ' + data)

        points = points[0]
    cv2.imshow('Detected QR code', img)
cap.release()
cv2.waitKey(0)
cv2.destroyAllWindows()