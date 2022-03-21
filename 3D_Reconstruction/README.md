# 3D_Reconstruction

The task was to perform 3D reconstruction of the scene based on [the data from this link](https://drive.google.com/file/d/1oQEbu9Bszh5ZQwIPoyMwidSI7AY1UHwg/view?usp=sharing), using a.o. OpenCV  in c++.

As an accesory, I've created GUI in Qt allowing the user to experiment with different parameters of the reconstructor. Though the project has educational, not practical purpose, The photos are stored after processing, what provides shorter time of calculation after reusing them with different parameters. The downside of this solution is a high memory consumption, but it greatly improves the comfort of using the gui.

# How to use:
- the user should store every files in one directory. Photos should have .JPG format, camera calibration info should be provided using OpenCV and stored in .xml file, camera exterior orientation should be stored in .txt file in the same way as in the file prowided in previously indicated link.
- There is no protection against entering incorrect data of the parameters
- ### ***parameters:***
- Gaussian Filter Settings are "ksize" and "sigmaX" of the [Gaussian Blur](https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1)
- desctiption of Orb Settings can be found in the following link: [ORB](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html)
- "Z min" and "Z max" are minimum and maximum distance from the camera in Z coordinate of the points of interest
- Max error (int) defines the degree of tolerance during matching. If the max error is too high, the matcher produces falce matches, if it's too low, it filters out good matches.
- ROI size is the size of the area in which the matcher is looking for a match point from one image to a point from another image. This value should be a divisor of both the height and width of the image

# required files:


 # warnings
