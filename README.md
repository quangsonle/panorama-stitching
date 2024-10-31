# panorama_stitching in C++

Finding a panorama, or stitching images from different perspectives, is not a new problem in image processing. The solution involves a combination of (1) matching point discovery, which can be done by feature extractors like SIFT or ORB, or manually, (2) homography matrix estimation using OpenCV's regression, and (3) stitching the transformed image onto the reference within the reference's coordinate system.

1. The first issue is the direction of transformation: typically, the reference is selected as the image on the left, and then the image on the right is transformed and stitched to the left, with the new size being the sum of the left and right. This method helps to eliminate negative coordinates if the right image is chosen as the reference (since the left coordinates could be negative after transformation). However, the new size (sum of left and right) is not optimal, as a significant overlap can lead to a large redundant area (i.e., black pixels).
This solution proposes a compensation matrix that will be multiplied with the homography matrix to adjust for negative coordinate pixels, in case the right image is the reference. A maximum expanded size is also calculated to minimize redundant black pixels. Thus, the transformation can be performed from any direction, resulting in favorable outcomes.

2. The second issue concerns the reference point; the video demonstrates that if the reference is not centered but on one side, and there are more than two pictures, the furthest one from the reference can be distorted due to extensive coordinate expansion. To automate the panorama creation process, the center should be detected among all pictures; this issue will be addressed in future work.
3. The test dataset is taken from: https://github.com/sumeyye-agac/homography-and-image-stitching-from-scratch



The following result is when the center is the reference (right->center, left->(right-center))
![center_is_reference](https://github.com/quangsonle/panorama_stitching/assets/48610518/c35e4e45-04ae-43e3-bb4e-e697586c01cd)
The following result is when the right is the reference (center->right, left->center-right)): we can see the distortion when the left is stitched
![right_is_reference](https://github.com/quangsonle/panorama_stitching/assets/48610518/d8772231-f77f-46d2-ad29-624a0926f70e)

Please see the demonstration video at [https://youtu.be/BFVoZWYr1Ao](https://www.youtube.com/watch?v=BFVoZWYr1Ao)

To run the program, follow these steps:

Install Visual Studio, specifically the 2022 community version.

Integrate Opencv into the C++ project by following the instructions provided at https://christianjmills.com/posts/opencv-visual-studio-getting-started-tutorial/windows/.

In the Release folder, ensure that the following items are present:

Three input images named 1.jpg, 2.jpg, and 3.jpg in the dataset folder

DLL Opencv lib files (as described in the "Identifying the Necessary DLL" section of step 2)

Replace the content of the main cpp file (created by Visual Studio for the C++ Console project) with the content of "homography_test.cpp".

After compiling in release mode, navigate to the release directory and run the .exe file.

This program can also be easily run on Linux using a Cmake file already linked to Opencv.
