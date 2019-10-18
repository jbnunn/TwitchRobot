import os, sys
import cv2
import numpy as np 

"""
Get source video, then:

1. Convert to grayscale
2. Gaussian blur
3. Detect edges
3a. Detect edges of a certain color
4. Transform edges to "lanes"

"""
resized_width = 400
resized_height = 300

def resize(image):
    return cv2.resize(image, (resized_width,resized_height))

def process(video):
    
    # Step 0: Setup Windows and resize input    
    blank = np.zeros(shape=(resized_width,resized_height,3))

    cv2.imshow('resized',blank)
    cv2.imshow('grayscale',blank)
    cv2.imshow('blurred',blank)
    cv2.imshow('masked',blank)
    cv2.imshow('edges',blank)
    # cv2.imshow('line_image', blank)


    cv2.moveWindow('resized',0,0)
    cv2.moveWindow('grayscale',410,0)
    cv2.moveWindow('blurred',0,310)
    cv2.moveWindow('masked',410,310)
    cv2.moveWindow('edges',820,310)
    # cv2.moveWindow('line_image',820,0)

    # Step 1: Grab the source video
    cap = cv2.VideoCapture(video)

    total_lines = 0
    while(cap.isOpened()):
        _, original = cap.read()
        if original is not None:

            resized = resize(original)
            line_image = np.zeros_like(resized)


            # Convert to grayscale
            grayscale = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

            # Gaussian Blur
            blurred = cv2.GaussianBlur(grayscale, (7,7), sigmaX=0)

            # Detect only white edges
            low_white = np.array([120,120,120])
            high_white = np.array([200,200,200])
            
            mask = cv2.inRange(resized, low_white, high_white)

            # Detect Edges
            edges = cv2.Canny(mask, 75, 100)
            
            # Draw Lines
            min_line_length = 10
            max_line_gap = 50
            rho = 1
            theta = np.pi/180
            threshold = 50
            thickness = 5

            lines = cv2.HoughLinesP(edges, rho=rho, theta=theta, threshold=threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)
            if lines is not None:
                total_lines += len(lines)
                print(total_lines)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line.reshape(4)
                    cv2.line(resized,(x1,y1),(x2,y2),(0,0,255),2)

            cv2.imshow('resized', resized)
            cv2.imshow('grayscale', grayscale)
            cv2.imshow('blurred', blurred)
            cv2.imshow('edges', edges)
            cv2.imshow('masked', mask)
            # cv2.imshow('line_image', line_image)

            if cv2.waitKey(1) == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("Total lines found: ", total_lines)

def process_frame():
    image = cv2.imread('frame.png')
    edges = cv2.Canny(image, 75, 100)

    # Draw Lines
    min_line_length = 50
    max_line_gap = 50
    rho = 1
    theta = np.pi/180
    threshold = 50
    thickness=5

    lines = cv2.HoughLinesP(edges, rho, theta, threshold, min_line_length, max_line_gap)
    
    if lines is not None:
        print("Lines found: ", len(lines))
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)

    while True:
        cv2.imshow("image", image)
        
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    video = sys.argv[1]
    process(video)
    # process_frame()
