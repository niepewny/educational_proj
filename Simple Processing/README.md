# Simple Image Processing

The project contains of two tasks: 
- Pr6: performing adaptive threshold, using geometric/arithmetic mean or median as the threshold value
- Mo1: performing opening and closing

Though there are two different tasks, there was a requirement to include it into one file, but they are still splitted into two classes.

# required files:
 ### [OpenCV](https://sourceforge.net/projects/opencvlibrary/files/4.5.5/opencv-4.5.5-vc14_vc15.exe/download)
 - it should be pasted in the project's directory.
 - the directory containing **.dll** file is still needed to be included to environment variables.

 # warnings
- The dilatation is performed in an inefficient way. For instance, using the mask would be the better way.