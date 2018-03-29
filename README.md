## DESCRIPTION **************************************
This C++ implemented algorithm is described in  
"Color Consistency Correction Based on Remapping Optimization for Image Stitching", ICCV Workshop 2017

Notice: This program is free for personal, non-profit and academic use.
All right reserved to CVRS: http://cvrs.whu.edu.cn/

If you have any question, please contact: menghanxyz@gmail.com (Menghan Xia)

## USAGE ********************************************
### Dependent Libarary [compulsory]:
OpenCV 2.4.9 is recommended.

### Project Configure:
This procedure is developed on Visual Studio 2010 under windows8.1 system environment,
where the source code is organized with CMakeLists. So, before opening it in Visual Studio,
you need to configure the project with the software named CMake.

### Running and Test:
There will be a "Data" folder in the decompressing files, where three created folders exist:
"Cache"  : creat a text file that describes the adjacent relationships of each image. [example contained]
"Images" : input your source images that are aligned geometrically. [example contained]
"Results": procedure will output the processed results in this position. [creat it by yourself]

Besides, to use the existing "Data" directory successfully, do not forget to modify the path variable 
"baseDir" to its absolute path of "Data" in the source file "CoColour/Source/Utils/util.h" [line 16]

So far, you can run the procedure and see the color correction results now.

### About Datasets [LUNCHROOM, CAMPUS, ZY-3, UAV]: 
There is some unclear problem in uploading datasets ZY-3 and UAV. If necessary, please contact us by email to get them.