// homography_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include <algorithm>
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

using namespace std;
//#include "opencv2/xfeatures2d.hpp"
using namespace cv;
//using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
//read coordinates of the cursor, it is for debug purposes
void onMouse(int action, int x, int y, int flag, void* param) {//for debug the transformation
	if (action == cv::EVENT_LBUTTONDOWN) {
		cout << "x:" << x << "y:" << y << endl;
	}
}
class find_homography // OOP - just as the instruction prefers
{
public:
	//If the matrix is transformed resulting in negative coordinate elements, 
        //those elements would be omitted. 
        //This function is designed to compensate for this situation 
        //by moving those points with an offset to the positive side.
        // it aims to find the minimums after transformation
	void find_offset(Mat hm, int& Offset_x, int& Offset_y, int rows, int cols)
	{
	
		double offset_x=0.0 , offset_y=0.0;
		// find x offset
		if (hm.at<double>(0, 0) >= 0.0 && hm.at<double>(0, 1) >= 0.0 && hm.at<double>(0, 2) >= 0.0)
			offset_x = 0.0;
		else
		{
			if (hm.at<double>(0, 0) >= 0.0 && hm.at<double>(0, 1) >= 0.0 && hm.at<double>(0, 2) < 0.0)
				offset_x = abs(hm.at<double>(0, 2));
			else if (hm.at<double>(0, 0) >= 0.0 && hm.at<double>(0, 1) < 0.0)
			{
				float diffv = hm.at<double>(0, 2) + hm.at<double>(0, 1) * rows;
				offset_x = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
			else if (hm.at<double>(0, 1) >= 0.0 && hm.at<double>(0, 0) < 0.0)
			{
				float diffv = hm.at<double>(0, 2) + hm.at<double>(0, 0) * cols;
				offset_x = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
			else if (hm.at<double>(0, 1) < 0.0 && hm.at<double>(0, 0) < 0.0)
			{
				float diffv = hm.at<double>(0, 2) + hm.at<double>(0, 0) * cols + hm.at<double>(0, 1) * rows;
				offset_x = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
		}

		/// Now find y offset
		if (hm.at<double>(1, 0) >= 0.0 && hm.at<double>(1, 1) >= 0.0 && hm.at<double>(1, 2) >= 0.0)
			offset_y = 0.0;
		else
		{
			if (hm.at<double>(1, 0) >= 0.0 && hm.at<double>(1, 1) >= 0.0 && hm.at<double>(1, 2) < 0.0)
				offset_y = abs(hm.at<double>(1, 2));
			else if (hm.at<double>(1, 0) >= 0.0 && hm.at<double>(1, 1) < 0.0)
			{
				float diffv = hm.at<double>(1, 2) + hm.at<double>(1, 1) * rows;
				offset_y = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
			else if (hm.at<double>(1, 1) >= 0.0 && hm.at<double>(1, 0) < 0.0)
			{
				float diffv = hm.at<double>(1, 2) + hm.at<double>(1, 0) * cols;
				offset_y = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
			else if (hm.at<double>(1, 1) < 0.0 && hm.at<double>(1, 0) < 0.0)
			{
				float diffv = hm.at<double>(1, 2) + hm.at<double>(1, 0) * cols + hm.at<double>(1, 1) * rows;
				offset_y = diffv >= 0.0 ? 0.0 : abs(diffv);
			}
		}
		Offset_x = (int)offset_x;
		Offset_y = (int)offset_y;
	}
	//This function is designed to find the maximum values after the transformation new size. 
        //It is opposite to the offset above- which finds the minimum after transformation
	void find_new_size(Mat hm, int& New_sizex, int& New_sizey, int rows, int cols)
	{
		{
			vector<Point2f> srcp;
			vector<Point2f>dstp;
			float xp = (hm.at<double>(2, 1) * hm.at<double>(0, 2) - hm.at<double>(0, 1)) / (hm.at<double>(0, 1) * hm.at<double>(2, 0) - hm.at<double>(2, 1) * hm.at<double>(0, 0));
			float yp = (hm.at<double>(2, 0) * hm.at<double>(0, 2) - hm.at<double>(0, 0)) / (hm.at<double>(0, 0) * hm.at<double>(2, 1) - hm.at<double>(2, 0) * hm.at<double>(0, 1));
		//	printf("for x:%f,%f,....%f,%f\n", (hm.at<double>(2, 1) * hm.at<double>(0, 2) - hm.at<double>(0, 1)), (hm.at<double>(0, 1) * hm.at<double>(2, 0) - hm.at<double>(2, 1) * hm.at<double>(0, 0)), xp, yp);
			if (xp > cols || yp > rows)
			{
				xp = 0.0;
				yp = 0.0;
			}
			srcp.push_back(Point2f(xp, yp));
			srcp.push_back(Point2f(0, 0));
			srcp.push_back(Point2f(cols, rows));
			srcp.push_back(Point2f(0, rows));
			srcp.push_back(Point2f(cols, 0));
			perspectiveTransform(srcp, dstp, hm);
			New_sizex = (int)(std::max({ dstp[0].x,dstp[1].x,dstp[2].x,dstp[3].x,dstp[4].x }));

		}
		///now find y max
		{
			vector<Point2f> srcp;
			vector<Point2f>dstp;
			float xp = (hm.at<double>(2, 1) * hm.at<double>(1, 2) - hm.at<double>(1, 1)) / (hm.at<double>(1, 1) * hm.at<double>(2, 0) - hm.at<double>(2, 1) * hm.at<double>(1, 0));
			float yp = (hm.at<double>(2, 0) * hm.at<double>(1, 2) - hm.at<double>(1, 0)) / (hm.at<double>(1, 0) * hm.at<double>(2, 1) - hm.at<double>(2, 0) * hm.at<double>(1, 1));
		//	printf("for y:%f,%f,....%f,%f\n", (hm.at<double>(2, 1) * hm.at<double>(1, 2) - hm.at<double>(1, 1)), hm.at<double>(1, 1) * hm.at<double>(2, 0) - hm.at<double>(2, 1) * hm.at<double>(1, 0),xp, yp);
			if (xp > cols || yp > rows)
			{
				xp = 0.0;
				yp = 0.0;
			}
			
			srcp.push_back(Point2f(xp, yp));
			srcp.push_back(Point2f(0, 0));
			srcp.push_back(Point2f(cols, rows));
			srcp.push_back(Point2f(0, rows));
			srcp.push_back(Point2f(cols, 0));
			perspectiveTransform(srcp, dstp, hm);
		//	cout << "y max find:" << dstp << endl;


			New_sizey = (int)(std::max({ dstp[0].y,dstp[1].y,dstp[2].y,dstp[3].y,dstp[4].y }));
		}
	}
	Mat stitch_two_images(Mat img1, Mat img2)
	{

		int minHessian = 400;
		//Ptr<ORB> detector = ORB::create(minHessian); //try ORB but worse than SIFT
		Ptr<SIFT> detector = SIFT::create(minHessian);
		std::vector<KeyPoint> keypoints1, keypoints2;
		Mat descriptors1, descriptors2;
		detector->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
		detector->detectAndCompute(img2, noArray(), keypoints2, descriptors2);
		//-- Step 2: Matching descriptor vectors with a FLANN based matcher
		// Since SURF is a floating-point descriptor NORM_L2 is used
		//Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming"); //this is the matcher for ORB
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		std::vector< std::vector<DMatch> > knn_matches;
		matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
		//-- Filter matches using the Lowe's ratio test
		const float ratio_thresh = 0.7f; /// threshold to decide the match (i.e distance of KNN)
		std::vector<DMatch> good_matches;
		std::vector<Point2f> points1, points2;
	
		for (size_t i = 0; i < knn_matches.size(); i++)
		{
			if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
			{
				good_matches.push_back(knn_matches[i][0]);
				points1.push_back(keypoints1[knn_matches[i][0].queryIdx].pt);
				points2.push_back(keypoints2[knn_matches[i][0].trainIdx].pt);
			
			}
		}
	
		//-- Draw matches
		Mat img_matches;
		drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
			Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		//-- Show detected matches
	
		Mat  h = findHomography(points1, points2, RANSAC); /// RANSAC-based regression to find homography


		int offset_x, offset_y;
		find_offset(h, offset_x, offset_y, img1.rows, img1.cols);
		
	
		double data[9] = { 1.0,0.0,offset_x ,0.0,1.0,offset_y,0.0,0.0,1.0 };
		cv::Mat offset_mat = cv::Mat(3, 3, CV_64F, data); 



		//cout << "offset_mat=" << offset_mat << endl;
		cout << "homography matrix=" << h << endl; // to print the "raw" homography matrix
		Mat h_c = offset_mat * h; ///to make the offset work
		cout << "after  matrix=" << h_c << endl; // to print the "raw" homography matrix
		Mat im1Reg;

		cv::Mat result, cnp2;

		int newsize1x, newsize1y,newsize2x,newsize2y;
		find_new_size(h_c, newsize1x, newsize1y, img1.rows, img1.cols);
		//printf("size1: %d,%d, size 1 orignial:%d,%d \n", newsize1x, newsize1y, img1.rows, img1.cols);
		find_new_size(offset_mat, newsize2x, newsize2y, img2.rows, img2.cols);
		//printf("size2: %d,%d, size 2 orignial:%d,%d \n", newsize2x, newsize2y, img2.rows, img2.cols);
		int size_cols = newsize1x > newsize2x ? newsize1x : newsize2x;
		int size_rows = newsize1y > newsize2y ? newsize1y : newsize2y;
	
		warpPerspective(img1, result, h_c, cv::Size(size_cols, size_rows)); //move img1 to img2 reference 
	
		Mat img2t;
		warpPerspective(img2, img2t, offset_mat, cv::Size(size_cols, size_rows)); // move img2 with the offset transformation to fit the img1
	
		cv::Rect crop_box = cv::Rect(offset_x, offset_y, img2.cols , img2.rows);
		
	
		cv::Mat half(result, crop_box);
		
		img2t(crop_box).copyTo(half);
	//	imshow("img2t", img2t);
		

		return result;
	}
};
void main()
{
	Mat img1 = imread("3.jpg");
	Mat img2 = imread("2.jpg");
	Mat img3 = imread("1.jpg");
	find_homography find_homography_instance;
	cout << "for 1-2:" << endl;
	Mat m12 = find_homography_instance.stitch_two_images(img1, img2);
	Mat display12, final_display;
	resize(m12, display12, cv::Size(1080, 720), 0, 0, cv::INTER_LINEAR);
	imshow("stitched_image 1-2", display12);
	cv::waitKey(0);
	cv::destroyAllWindows();
	cout << "for 3-12:" << endl;
	Mat m123 = find_homography_instance.stitch_two_images(img3, m12);
	resize(m123, final_display, cv::Size(1080, 720), 0, 0, cv::INTER_LINEAR);
	imshow("final stitched_image", final_display);
	imwrite("stitched_image.png", final_display);
	cv::waitKey(0);
	
}
