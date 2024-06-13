# panorama_stitching
To run the program, follow these steps:

1. Install Visual Studio, specifically the 2022 community version.
2. Integrate Opencv into the C++ project by following the instructions provided at https://christianjmills.com/posts/opencv-visual-studio-getting-started-tutorial/windows/.
3. In the Release folder, ensure that the following items are present:
   - Three input images named 1.png, 2.png, and 3.png
   - DLL Opencv lib files (as described in the "Identifying the Necessary DLL" section of step 2)
4. Replace the content of the main cpp file (created by Visual Studio for the C++ Console project) with the content of "homography_test.cpp".
5. After compiling in release mode, navigate to the release directory and run the .exe file.

This program can also be easily run on Linux using a Cmake file already linked to Opencv.

Approach:

In order to solve homography problems, the program first identifies matching points from two images using feature detection and matching (in this case, SIFT is used). A RANSAC-based regression is then applied to find the homography matrix. The source image is then transformed to the coordinate system of the destination image using the homography matrix, resulting in a transformed image. Finally, the destination image is copied onto the transformed image, effectively stitching them together.

A couple of points to note:
1. While SIFT works well in this case, in other scenarios, ORB may work better for finding matched features.
2. The distance threshold used to decide if two features are matched should be relatively high to be more tolerant of a match. While some "mismatches" are considered matches, RANSAC can help remove outliers. This is specific to finding matched points by feature detection, and other regression methods for homography may be necessary in different cases.
3. To address the issue of pixels having negative coordinates after transformation, an offset matrix is multiplied by the homography, shifting the transformed image to the "positive side." In cases where pixels have high coordinates, the solution is to enlarge the image's size.
   
A demo is at: https://youtu.be/n2y3zm_ARYs and the final stitched image is "stitched_image.png"

*Note that both the YouTube demo link and this repository are private 

