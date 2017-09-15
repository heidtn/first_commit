import cv2
import glob
import sys
import timeit
import numpy as np

imfiles = glob.glob('../temp_images/*.jpg')
imfiles = sorted(imfiles, key=lambda x: float(x.split('.jpg')[0].split('/')[-1]))
print imfiles
colors = []




def thin_image(img):
    size = np.size(img)
    skel = np.zeros(img.shape,np.uint8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    done = False
    while( not done):
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
     
        zeros = size - cv2.countNonZero(img)
        if zeros==size:
            done = True
     
    return skel

def remove_noise(img, size=200):
    im2, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    good_contours = []
    for c in contours:
        if cv2.contourArea(c) > size:
            good_contours.append(c)

    mask2 = np.zeros(img.shape[:2], np.uint8)
    cv2.drawContours(mask2, good_contours, -1, 255, -1)
    return mask2


def draw_circle(event,x,y,flags,param):
    global mouseX, mouseY, display
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(display, (x,y),6,(255,0,0),2)
        mouseX,mouseY = x,y
        colors.append(hsv[y, x])
        print "current color", hsv[y, x]

cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
i = 0

start_time = timeit.default_timer()

# f = open('corner_extraction.txt', 'w+')


for filename in imfiles:
    timestamp = filename.split('.jpg')[0].split('/')[-1]
    image = cv2.imread(filename)
    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

    f_x = 279.60814589461154
    f_y = 280.1693782018111
    c_x = 222.49106441516423
    c_y = 317.7691476613439

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    # Try segmenting the thing
    lower_orange = np.array([6,80,130])
    upper_orange = np.array([16,255,255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Dilate then erode the mask
    kernel = np.ones((3,3),np.uint8)
    dilation = cv2.dilate(mask, kernel)

    mask2 = remove_noise(dilation)

    thinned = thin_image(mask2)
    dilated_thin = cv2.dilate(thinned, kernel)
    cleaned_thin = remove_noise(dilated_thin, 100)
    corners = cv2.cornerHarris(cleaned_thin,16,1,0.04)

    img2 = np.zeros(mask.shape, dtype=np.uint8)

    corners2 = cv2.dilate(corners, kernel, iterations=1)
    print corners2.max()

    CORNERINESS = .04
    img2[corners2 > 0.5*corners2.max()] = [255]

    display = image

    # Find and draw corners
    res, contours, hierarchy = cv2.findContours(img2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    corner_list = []
    for c in contours:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(display, (cX, cY), 7, (0, 255, 0), -1)
        x = (cX - c_x) / f_x
        y = (cY - c_y) / f_y
        corner_list.append([x, y])


    # f.write(timestamp + ",")
    # numpoints = min(len(corner_list), 20)
    # f.write(str(numpoints))

    # for i in range(20):
    #     if i < len(corner_list):
    #         f.write("," + str(corner_list[i][0]) + "," + str(corner_list[i][1]))
    #     else:
    #         f.write(",0,0")

    # f.write('\n')

    i += 1
    print i

    while True:
        cv2.imshow('image', display)
        cv2.imshow('mask', mask2)
        cv2.imshow('dilatedthin', cleaned_thin)

        key = cv2.waitKey(20)
        if key == ord('q'):
            sys.exit()
        elif key == ord('n'):
            break

elapsed = timeit.default_timer() - start_time

print "elapsed time: ", elapsed
print "time per analysis", elapsed/float(i)