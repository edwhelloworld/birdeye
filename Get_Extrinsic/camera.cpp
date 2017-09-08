#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
using namespace cv;
int n_boards = 0;
int board_w;
int board_h;

void bird_eye();

int main(int argc, char* argv[]) {

	board_w = 9;//9//atoi(argv[1]);
	board_h = 6;//atoi(argv[2]);
	bird_eye();
	return 0;
}

void bird_eye() {
	int board_n = board_w * board_h;
	Size board_sz = Size(board_w, board_h);
	Size imageSize;
	Mat intrinsic, distortion;
	//FileStorage fs2("Intrinsics2.xml", FileStorage::READ);
	//fs2["Intrinsics"] >> intrinsic;
	//fs2.release();
	//FileStorage fs3("Distortion2.xml", FileStorage::READ);
	//fs3["Distortion"] >> distortion;
	//fs3.release();
	FileStorage fs1("cameraParam.xml", FileStorage::READ);
	fs1["camera_matrix"] >> intrinsic;
	fs1["distortion_coefficients"] >> distortion;
	Mat image = imread("./Resource/cross.jpg", 1);
	imshow("org_img", image);
	imageSize = image.size();
	Mat mapx, mapy;
	Mat gray_image;
	
	initUndistortRectifyMap(intrinsic, distortion, Mat(),
		getOptimalNewCameraMatrix(intrinsic, distortion, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, mapx, mapy);
	remap(image, gray_image, mapx, mapy, INTER_LINEAR);

	cvtColor(gray_image, gray_image, CV_BGR2GRAY);
	imshow("undistort_img", gray_image);
	for (int i = 0; i < 90; i += 5)
	{
		Mat map1, map2;
		float theta = i * 3.14f / 180.f;
		Mat R = (Mat_<float>(3, 3) <<1, 0, 0, 0, 6.8253325706286228e-01, 7.3045479133977231e-01, 0, -7.2855051949300931e-01, 6.8263323837651402e-01);
		//9.9809310526524031e-01, 5.7987011904622368e-02,-2.1157969476367177e-02, -2.4169212066805296e-02,6.8253325706286228e-01, 7.3045479133977231e-01, 5.6797908500749339e-02,-7.2855051949300931e-01, 6.8263323837651402e-01);
		//1, 0, 0, 0, cos(theta), sin(theta), 0, -sin(theta), cos(theta)) ;
		Mat R1 = Mat::eye(3, 3, CV_32F);
		const int newImgW = 240;
		const int newImgH = 480;
		Size newImagSize(newImgW, newImgH);
		//Mat newCam = (Mat_<float>(3, 3) << newImagSize.width / 2, 0, newImagSize.width / 2, 0, newImagSize.width / 2, newImagSize.height / 2, 0, 0, 1);
		Mat newCam = (Mat_<float>(3, 3) << newImagSize.height *0.072, 0, (newImagSize.width) / 2, 0, newImagSize.height *0.072,(newImagSize.height) / 2, 0, 0, 1);
		
		vector<Point3f> objVtrPts;
		vector<Point2f> imgPts;
		Mat objVtrPtsM;
		objVtrPts.push_back(Point3f(-1, -2.2, 0));    //三维坐标的单位是毫米
		objVtrPts.push_back(Point3f(1, -2.2, 0));
		objVtrPts.push_back(Point3f(-1, -0.2, 0));
		objVtrPts.push_back(Point3f(1,-0.2, 0));

		Mat t = (Mat_<float>(3, 1) << 0, 0.2,0.32);//point in world coordinate + t =point in cam coordinate
		Mat r;
		Rodrigues(R1.inv(), r);
		Mat d = (Mat_<float>(4, 1) << 0,0,0,0);
		projectPoints(objVtrPts, r, t, newCam, d, imgPts);
		
		Mat newImg, roiImg;
		initUndistortRectifyMap(intrinsic, distortion, R.inv(), newCam,
			newImagSize, CV_16SC2, map1, map2);
		Mat Nmap1, Nmap2;
		
		int width = 240;
		int height = 480;
		//int startCols;//260;
		//int startRows;//210;
		//startCols = (newImgW - width) / 2;
		//startRows = 0;//(newImgH - height) / 2;
		map1(cv::Rect(imgPts[0], imgPts[3])).copyTo(Nmap1);
		map2(cv::Rect(imgPts[0], imgPts[3])).copyTo(Nmap2);

		remap(image, roiImg, Nmap1, Nmap2, INTER_LINEAR);
		remap(image, newImg, map1, map2, INTER_LINEAR);
		for (int i = 0; i < 4; i++)
		{
			circle(newImg, imgPts[i], 3, Scalar(0, 0, 255), -1, 8);
			std::cout << "Image point: " << imgPts[i] << std::endl;
		}
		imshow("undistort View", newImg);
		imshow("roi View", roiImg);
		int c = waitKey();
	}

	
	int corner_count = 0;
	
	vector<Point2f> corners;
	bool found = findChessboardCorners(image, board_sz, corners,
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );//| CV_CALIB_CB_FAST_CHECK| CV_CALIB_CB_NORMALIZE_IMAGE
	
	if(!found){
		printf("couldn't aquire chessboard!\n");
		return;
	}
	
	cornerSubPix(gray_image, corners, Size(11, 11),
		Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

	
	//vector<Point2f> objPts(4);
	//vector<Point2f> imgPts(4);
	//objPts[0] = Point2f(0, 0);//objPts[0].x = 0;			objPts[0].y = 0;
	//objPts[1] = Point2f(board_w - 1, 0);//objPts[1].x = board_w - 1;	objPts[1].y = 0;
	//objPts[2] = Point2f(0, board_h - 1);//objPts[2].x = 0;			objPts[2].y = board_h - 1;
	//objPts[3] = Point2f(board_w - 1, board_h - 1);//objPts[3].x = board_w - 1;	objPts[3].y = board_h - 1;
	//imgPts[3]   = corners[0];
	//imgPts[2]	= corners[board_w - 1];
	//imgPts[1]	= corners[(board_h - 1) * board_w];
	//imgPts[0]	= corners[(board_h - 1) * board_w + board_w - 1];

	vector<Point3f> objVtrPts;
	Mat objVtrPtsM;
	//objVtrPts.push_back(Point3f(0, 0, 0));    //三维坐标的单位是毫米
	//objVtrPts.push_back(Point3f((board_w - 1), 0, 0));
	//objVtrPts.push_back(Point3f(0, (board_h - 1), 0));
	//objVtrPts.push_back(Point3f((board_w - 1), (board_h - 1), 0));
	for (int i=0; i < board_h*board_w; i++)
	{
		int x = i % 9;
		int y = i / 9;
		objVtrPts.push_back(Point3f(x, y, 0));
	}
	Mat(objVtrPts).convertTo(objVtrPtsM, CV_32F);

	vector<Point2f> imgVtrPts;
	//imgVtrPts[0]=(Point2f(corners[(board_h - 1) * board_w + board_w - 1]));
	//imgVtrPts[1]=(Point2f(corners[(board_h - 1) * board_w]));
	//imgVtrPts[2]=(Point2f(corners[board_w - 1]));
	//imgVtrPts[3]=(Point2f(corners[0]));//0
	for (int i = 0; i < board_h*board_w; i++)
	{
		imgVtrPts.push_back(Point2f(corners[board_h*board_w-1-i]));
	}
	Mat imgVtrPtsM;
	Mat(imgVtrPts).convertTo(imgVtrPtsM, CV_32F);
	

	if (found)  drawChessboardCorners(image, board_sz, Mat(corners), found);//

	vector<double> rv(3), tv(3);
	Mat rvec(rv), tvec(tv);
	double rm[9];
	Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rotM, rvec);
	solvePnP(objVtrPtsM,Mat(imgVtrPtsM), intrinsic, distortion, rvec, tvec);
	Rodrigues(rvec, rotM);

	cout << "rotation_matrix: " << endl << rotM << endl;
	cout << "translation_matrix: " << endl << tv[0] << " " << tv[1] << " " << tv[2] << endl;
	FileStorage fs("Extrinsics.xml", FileStorage::WRITE);
	fs << "rotation_matrix" << rotM;
	fs << "translation_matrix" << tv;
	fs.release();
	imshow("Chessboard", image);
	waitKey();
	
}

