// homography_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"


#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"

using namespace std;
//#include "opencv2/xfeatures2d.hpp"
using namespace cv;
//using namespace cv::xfeatures2d;
using std::cout;
using std::endl;


class find_homography // OOP - just as the instruction prefers
{
public:
	//In the event that the matrix is transformed resulting in negative coordinate elements, 
        //those elements would be omitted. 
        //This function is designed to compensate for this situation by moving those points with an offset to the positive side.
	void find_offset(Mat hm, double& offset_x, double& offset_y, int rows, int cols)
	{
		offset_x = 0.0;
		offset_y = 0.0;
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

		/// now find y offset
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
	//	imshow("Good Matches", img_matches);
		Mat  h = findHomography(points1, points2, RANSAC); /// RANSAC-based regression to find homography


		double offset_x, offset_y;
		find_offset(h, offset_x, offset_y, img1.rows, img1.cols);
		

		double data[9] = { 1.0,0.0,offset_x ,0.0,1.0,offset_y,0.0,0.0,1.0 };
		cv::Mat offset_mat = cv::Mat(3, 3, CV_64F, data); 



		//cout << "offset_mat=" << offset_mat << endl;
		cout << "homography matrix=" << h << endl; // to print the "raw" homography matrix
		Mat h_c = offset_mat * h; ///to make the offset work
	
		Mat im1Reg;

		cv::Mat result, cnp2;

		/// caculate size_cols and size_rows: 
		//If the offsets are for the negative coordinates, this one is for pixels at high coordinates that are out of the size of the original matrices.
		int size_cols = img1.cols >= img2.cols ? img1.cols : img2.cols;

		if (offset_x > 0.0)
		{
			size_cols += 2 * offset_x;
			
		}

		int size_rows = img1.rows >= img2.rows ? img1.rows : img2.rows;
		if (offset_y > 0.0)
		{
			size_rows += 2 * offset_y;

		}

		warpPerspective(img1, result, h_c, cv::Size(size_cols, size_rows)); //move img1 to img2 reference 

		Mat img2t;
		warpPerspective(img2, img2t, offset_mat, cv::Size(size_cols, size_rows)); // move img2 with the offset transformation to fit the img1

		cv::Rect crop_box = cv::Rect(offset_x, offset_y, img2t.cols - 2 * offset_x, img2t.rows - 2 * offset_y);
		cv::Mat half(result, crop_box);


		img2t(crop_box).copyTo(half);
		//imshow("final", result);

		//cv::waitKey(0);

		return result;
	}
};
void main()
{
	Mat img1 = imread("1.png");
	Mat img2 = imread("2.png");
	Mat img3 = imread("3.png");
	find_homography find_homography_instance;
	cout << "for 1-2:" << endl;
	Mat m12= find_homography_instance.stitch_two_images(img1, img2);
	imshow("stitched_image 1-2", m12);
	cv::waitKey(0);
	cv::destroyAllWindows();
	cout << "for 3-12:"<< endl;
	Mat m123 = find_homography_instance.stitch_two_images(img3, m12);
	imshow("final stitched_image", m123);
	imwrite("stitched_image.png", m123);
	cv::waitKey(0);
}