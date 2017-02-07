import cv2
import numpy as np
import time

start = time.time()

i = 1

while i < 100:

    im = cv2.imread("Blop.JPG", cv2.IMREAD_COLOR)
    
    height, width, depth = im.shape
    
    '''
    im1 = np.empty([height, width, depth])
    
    MinBlueRatio = 1.2
    MaxBlueRatio = 2.50
    
    MinRedRatio = 0.85
    MaxRedRatio = 1.8
    
    BlueRatio = 0.0
    RedRatio = 0.0
    
    print "Calculating..."
    
    for i in range(0,height,1):
        for j in range(0,width,1):
            BlueRatio = float(im[i,j,1]/im[i,j,0])
            RedRatio = float(im[i,j,1]/im[i,j,2])
            
            if (BlueRatio > MinBlueRatio) and (BlueRatio < MaxBlueRatio):
                if (RedRatio > MinRedRatio) and (RedRatio < MaxRedRatio):
                    for x in (0,2,1):
                        im1[i,j,x] = im[i,j,x]
                else:
                    for x in (0,2,1):
                        im1[i,j,x] = 0
            else:
                for x in (0,2,1):
                    im1[i,j,x] = 0
    
    print "done"
    '''
    
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([15,50,50])
    upper_yellow = np.array([26,255,255])
    
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    kernel = np.ones((10,10),np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    
    #cv2.imwrite("opening.jpeg", opening)
    
    
    
    #cv2.imwrite("closing.jpeg", closing)
    
    res = cv2.bitwise_and(im,im, mask= closing)
    
    im2 = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)
    '''
    cv2.imwrite("im2.jpeg", im2)
    
    cv2.imwrite("ColourPick.jpeg", res)
    '''
    
    
    #im2 = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
    #im2 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    
    params = cv2.SimpleBlobDetector_Params()
    
    params.minThreshold = 10
    params.maxThreshold = 256
    
    params.filterByColor = True
    params.blobColor = 255
    
    params.filterByArea = True
    params.minArea = 2000
    params.maxArea = 10000000000000
    
    params.filterByCircularity = False
    params.minCircularity = 0.01
    params.maxCircularity = 1
    
    params.filterByConvexity = False
    params.minConvexity = 0.01
    params.maxConvexity = 1
    
    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    params.maxInertiaRatio = 1
    
    
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)
        
    keypoints = detector.detect(closing)
    
    im_with_circle = np.copy(im)
    #print im_with_circle.shape
    
    #print "keypoints:"
    
    for keypoint in keypoints:
        '''
        print "x = "+str(keypoint.pt[0])
        print "y = "+str(keypoint.pt[1])
        print "size = "+str(keypoint.size)
        '''
        cv2.circle(im_with_circle,(int(keypoint.pt[0]),int(keypoint.pt[1])),5*int(keypoint.size),[0,0,255],20)
    
    im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #cv2.imwrite("BlobedImage.jpg", im_with_keypoints)
    cv2.imwrite("BlobedImage1.jpg", im_with_circle)

print str(time.time()-start)
