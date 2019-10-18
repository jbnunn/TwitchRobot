# Lane Detection with OpenCV

## Install

    conda create -n LaneDetection python=3.6 opencv numpy pillow matplotlib
    source activate LandDetection

## Steps

### 1. The Source Image

For this demo you'll want source video to be 800x600.

### 2. Image Conversion

All image conversion can be done with [OpenCV](https://opencv.org). Before we can identify lane lines, we need to:

1) Convert the image to grayscale to deal with 1 channel instead of 3 channels (BGR)
2) Apply Gaussian blur to the image to reduce artifacts. Note the difference in edge detection (step 3) before Gaussian blur and after Gaussian blur:
3) Detect edges with Canny Edge detector
4) Determine the Region of Interest: We want to focus on the road, specifically the road defined by the lanes. We'll create a polygon of this space, and mask out the rest of the image (essentially turning it into 0's -- black -- in the array of pixels that define the image)
5) Hough Transform: Detect any shape, if that shape can be represented in mathematical form. It can detect the shape even if it is broken or distorted a little bit. This is handy to define lane lines that are "broken" in edge detection

## Inspiration/Education/Resources

* [OpenCV Python Tutorial](https://www.youtube.com/watch?v=eLTLtUVuuy4)
* [Hough Lines](https://www.geeksforgeeks.org/line-detection-python-opencv-houghline-method/)
* [Udacity/CarND-LaneLines-P1](https://github.com/udacity/CarND-LaneLines-P1): Just found this, I'm sure it has something good but I haven't dug in yet.
* [Python Plays: GTA V](https://www.youtube.com/playlist?list=PLQVvvaa0QuDeETZEOy4VdocT7TOjfSA8a)