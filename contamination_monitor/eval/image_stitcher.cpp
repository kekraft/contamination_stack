#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching/stitcher.hpp"

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/package.h>




#ifdef _DEBUG  
#pragma comment(lib, "opencv_core246d.lib")   
#pragma comment(lib, "opencv_imgproc246d.lib")   //MAT processing  
#pragma comment(lib, "opencv_highgui246d.lib")  
#pragma comment(lib, "opencv_stitching246d.lib");

#else  
#pragma comment(lib, "opencv_core246.lib")  
#pragma comment(lib, "opencv_imgproc246.lib")  
#pragma comment(lib, "opencv_highgui246.lib")  
#pragma comment(lib, "opencv_stitching246.lib");
#endif  

using namespace cv;  
using namespace std;
using namespace ros;


int main()  
{
 std::cout << "Starting program" << std::endl;

 std::string pkg_path = package::getPath("contamination_monitor");
 std::cout << "Package Path: " << pkg_path << std::endl;
 std::string img_dir_path = pkg_path + "/eval/images";
 std::cout << "Image Path: " << img_dir_path << std::endl;

 vector< Mat > vImg;
 Mat rImg;

 Mat img_left = imread(img_dir_path + "/left.jpg");
 Mat img_right = imread(img_dir_path + "/center.jpg");
 // Mat img_left = imread(img_dir_path + "/left.jpg");
 // Mat img_right = imread(img_dir_path + "/center.jpg");
 //Mat img_right = imread(img_dir_path + "/left.jpg");  // Used to try stitcher on same image
 	// Mat img_left = imread(img_dir_path + "/panorama_image1.jpg");
 	// Mat img_right = imread(img_dir_path + "/panorama_image2.jpg");

 if (img_left.cols < 1) {
 	std::cout << "Error reading image" << std::endl;
 	return -1;
 } else {
 	std::cout << "Image read correctly" << std::endl;
 	// imshow("opencvtest_1",img_left);
 	// waitKey(0);
 	// imshow("opencvtest_2", img_right);
 	// waitKey(0);
 }

 vImg.push_back(img_left);
 vImg.push_back(img_right);
 // vImg.push_back( imread("./stitching_img/S4.jpg") );
 // vImg.push_back( imread("./stitching_img/S5.jpg") );
 // vImg.push_back( imread("./stitching_img/S6.jpg") );
  

 Stitcher stitcher = Stitcher::createDefault(true);
 stitcher.setWarper(new PlaneWarper());
 // stitcher.setFeaturesFinder(new detail::SurfFeaturesFinder(1000,3,4,3,4));
 stitcher.setRegistrationResol(0.8); /// 0.6
 stitcher.setSeamEstimationResol(0.5);   /// 0.1
 stitcher.setCompositingResol(-1);   //1
 stitcher.setPanoConfidenceThresh(-1);   //1
 stitcher.setWaveCorrection(true);
 stitcher.setWaveCorrectKind(detail::WAVE_CORRECT_HORIZ);
 stitcher.setFeaturesMatcher(new detail::BestOf2NearestMatcher(true,0.3));
 stitcher.setBundleAdjuster(new detail::BundleAdjusterRay());

 



 unsigned long AAtime=0, BBtime=0; //check processing time
 AAtime = getTickCount(); //check processing time

 Stitcher::Status status = stitcher.stitch(vImg, rImg);



 BBtime = getTickCount(); //check processing time 
 printf("%.2lf sec \n",  (BBtime - AAtime)/getTickFrequency() ); //check processing time

 std::cout << "Stitching status: " << status << std::endl;

 if (Stitcher::OK == status){
 	imwrite(img_dir_path + "/output2.jpg", rImg);
 	// imshow("Stitching Result",rImg);	
 	// waitKey(0);
 } else if (Stitcher::ERR_NEED_MORE_IMGS == status){
 	std::cout << "Need more images." << std::endl;
 	std::cout << "Stitching failed." << std::endl;
 }
  
  //else
  //printf("Stitching fail.");

 
 

 std::cout << "End program." << std::endl;

 return 0;

}  
