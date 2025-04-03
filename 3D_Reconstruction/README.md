# 3D_Reconstruction

The task was to perform 3D reconstruction of the scene based on [the data from this link](https://drive.google.com/file/d/1ncjbGmhIzO0QKvR86mLDwl0_yOQiMW4s/view?usp=sharing), using a.o. OpenCV in C++.

As an extra, I've created a GUI in Qt allowing the user to experiment with different parameters of the reconstructor. Though the project has educational, not practical purpose, the photos are stored after processing, what provides shorter time of calculation after reusing them with different parameters. The downside of this solution is a high memory consumption, but it greatly improves the comfort of using the GUI.

# How to use:
- the project currently is prepaired to be used in debug mode, so it should be opened with IDE
- the user should store every file in one directory. Photos should have JPG format, camera calibration info should be provided using OpenCV and stored in .xml file, camera exterior orientation should be stored in .txt file in the same way as in the file provided in previously indicated link.
- There is no protection against entering incorrect data of the parameters
- ### ***parameters:***
- Gaussian Filter Settings are "ksize" and "sigmaX" of the [Gaussian Blur](https://docs.opencv.org/3.4/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1)
- description of ORB Settings can be found in the following link: [ORB](https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html)
- "Z min" and "Z max" are minimum and maximum distance from the camera in Z coordinate of the points of interest
- Max error (int) defines the degree of tolerance during matching. If the max error is too high, the matcher produces false matches, if it's too low, it filters out good matches.
- ROI size is the size of the area in which the matcher is looking for a match point from one image to a point from another image. This value should be a divisor of both the height and width of the image

# Required:
- prebuilded [OpenCV](https://github.com/opencv/opencv) with [contrib](https://github.com/opencv/opencv_contrib) (important modules: highgui, calib3d, cudaimgproc, cudafilters, imgcodecs, features2d, core - all in debug mode)
 - [PCL](https://github.com/PointCloudLibrary/pcl/releases) with OpenNI2 (required All-in-One Installer)
 - [Qt](https://www.qt.io/download-thank-you?hsLang=en) (tested on 5.15.2 version)

# Enviromental variables to set:
1. name: PCL_ROOT
- [path_to_PCL]
2. name: OPENNI2_INCLUDE64
- [path_to_OpenNI]\Include\
3. name: OPENNI2_LIB64
- [path_to_OpenNI]\Lib\
4. name: OPENNI2_REDIST64
- [path_to_OpenNI]\Redist\
5. name: QTDIR
- [path_to_Qt]
6. name: OPENCV_DIR
- [path_to_prebuilded_OpenCV]
7. name: OpenCV_INCLUDE_DIRS
- path to OpenCV's include folder
8. name: Path
- [path_to_OpenNI]
- [path_to_OpenNI]\Tools
- %PCL_ROOT%\bin
- %PCL_ROOT%\3rdParty
- %PCL_ROOT%\3rdParty\VTK\bin
- %QTDIR%\lib
- %QTDIR%\bin
- %OPENCV_DIR%\bin\Debug

 # Warnings
- After generating project in CMake, C/C++ Preprocessor Definition "BOOST_ALL_NO_LIB-DBOOST_ALL_NO_LIB" should be changed to "BOOST_ALL_NO_LIB". It seems to be an error in one of PCL files.
- The program does not provide security against wrong input (char instead of int in recontruction parameters for instance)
- There was a problem with [R|t] matrixes. OpenCV interprets Euler angles as their negative values. In result any functions depending on them, works incorrectly.
- The PCLviewer's properties are not edited. Thus after generating a cloud, the user should change the view orientation to check the ressult
