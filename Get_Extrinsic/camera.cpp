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
#define   Start_Line          (85)         //220 85  232 72
#define   End_Line            (269)        //405 269 415 255
#define   Start_Col           (100)         //120
#define   End_Col             (540)         //520 640-Start_Col
#define   Width               (End_Col-Start_Col)
#define   Height              (End_Line-Start_Line)
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
	FileStorage fs1("cameraParam.xml", FileStorage::READ);
	fs1["camera_matrix"] >> intrinsic;
	fs1["distortion_coefficients"] >> distortion;
	Mat image = imread("./Resource/cross.png", 1);
	imshow("org_img", image);
	imageSize = image.size();
	Mat mapx, mapy;
	Mat gray_image, undistort_img;
	
	initUndistortRectifyMap(intrinsic, distortion, Mat(),
		getOptimalNewCameraMatrix(intrinsic, distortion, imageSize, 1, imageSize, 0),
		imageSize, CV_16SC2, mapx, mapy);
	remap(image, undistort_img, mapx, mapy, INTER_LINEAR);
	//image.copyTo(gray_image);
	cvtColor(image, gray_image, CV_BGR2GRAY);

	//imshow("undistort_img", undistort_img);
//============================================================================================================
	//Mat CompressImg, sectImg;
	float JumpLine;
	int i;
	JumpLine = (float)(End_Line - Start_Line) / 59;// (249 - 65) / 60
	int SamplingLine[60] = { 0 };
	for (i = 0; i<60; i++)
	{
		SamplingLine[i] = (int)(Start_Line + JumpLine*i + 0.5);
	}

	float JumpCol = (float)(End_Col - Start_Col) / 119;
	int SamplingCol[120] = { 0 };
	for (i = 0; i<120; i++)
	{
		SamplingCol[i] = (int)(Start_Col + JumpCol*i + 0.5);//+ 0.5);
	}

	Mat CompressImg(60, 120, gray_image.type(),Scalar(0,0));//= image.clone();
	for (i = 0; i < 60; i++)//行循环   i < rowNumber
	{
		const uchar* data = gray_image.ptr<uchar>(SamplingLine[i]);//获取第i行的首地址 
		uchar* cdata = CompressImg.ptr<uchar>(i);
		for (int j = 0; j < 120; j++)//列循环 j < colNumber
		{
			cdata[j] = data[SamplingCol[j]];
		}
	}
	imshow("cimg", CompressImg);
	//Mat sectImg(gray_image.rows, gray_image.cols, gray_image.type(), Scalar(0, 0));
	//for (i = 0; i < 60; i++)//行循环   i < rowNumber
	//{
	//	const uchar* data = CompressImg.ptr<uchar>(i);//获取第i行的首地址 
	//	uchar* sdata = sectImg.ptr<uchar>(SamplingLine[i]);
	//	for (int j = 0; j < 120; j++)//列循环 j < colNumber
	//	{
	//		sdata[SamplingCol[j]] = data[j];
	//	}
	//}
	//imshow("sect_img", sectImg);
	//Mat reszImg;
	//resize(CompressImg, reszImg, Size(Width, Height), (0, 0), INTER_LINEAR);
	//
	//Mat tempImg(gray_image.rows, gray_image.cols, gray_image.type(), Scalar(0, 0));
	//reszImg.copyTo(tempImg(Rect(Start_Line, Start_Col, Width, Height)));
	//imshow("reszimg", reszImg);
	//imshow("tempImg", tempImg);
	
//======================================================================================================
	//for (int i = 0; i < 90; i += 5)
	{
		Mat map1, map2;
		int i = 50;//cout << i << endl;
		float theta = i * 3.14f / 180.f;
		Mat R = (Mat_<float>(3, 3) <<//1, 0, 0, 0, 6.8253325706286228e-01, 7.3045479133977231e-01, 0, -7.2855051949300931e-01, 6.8263323837651402e-01);
		//9.9809310526524031e-01, 5.7987011904622368e-02,-2.1157969476367177e-02, -2.4169212066805296e-02,6.8253325706286228e-01, 7.3045479133977231e-01, 5.6797908500749339e-02,-7.2855051949300931e-01, 6.8263323837651402e-01);
		1, 0, 0, 0, cos(theta), sin(theta), 0, -sin(theta), cos(theta)) ;
		Mat R1 = Mat::eye(3, 3, CV_32F);
		const int newImgW = 240;//240;
		const int newImgH = 320;//480;
		Size newImagSize(newImgW, newImgH);
		//Mat newCam = (Mat_<float>(3, 3) << newImagSize.width / 2, 0, newImagSize.width / 2, 0, newImagSize.width / 2, newImagSize.height / 2, 0, 0, 1);
		Mat newCam = (Mat_<float>(3, 3) << newImagSize.height *0.072, 0, (newImagSize.width) / 2, 0, newImagSize.height *0.072,(newImagSize.height) / 2, 0, 0, 1);
		
		vector<Point3f> objVtrPts;
		vector<Point2f> imgPts;
		Mat objVtrPtsM;
		objVtrPts.push_back(Point3f(-1.0, -1.5, 0));    //三维坐标的单位是米
		objVtrPts.push_back(Point3f(1.0, -1.5, 0));
		objVtrPts.push_back(Point3f(-1.0, -0.3, 0));
		objVtrPts.push_back(Point3f(1.0,-0.3, 0));

		Mat t = (Mat_<float>(3, 1) << 0, 0.3,0.32);//point in world coordinate + t = point in cam coordinate
		Mat r;
		Rodrigues(R1.inv(), r);
		Mat d = (Mat_<float>(4, 1) << 0,0,0,0);
		projectPoints(objVtrPts, r, t, newCam, d, imgPts);
		
		Mat newImg, roiImg;
		initUndistortRectifyMap(intrinsic, distortion, R.inv(), newCam,
			newImagSize, CV_32FC1, map1, map2);//CV_16SC2
		Mat Nmap1, Nmap2;
		
		int width = 240;
		int height = 480;

		map1(cv::Rect(imgPts[0], imgPts[3])).copyTo(Nmap1);
		map2(cv::Rect(imgPts[0], imgPts[3])).copyTo(Nmap2);
		//	cout << Nmap1 << endl<<endl;
		//int gray_imageTP = CompressImg.type();//cout << gray_imageTP << endl;
		int Nwidth = Nmap1.cols;
		int Nheight= Nmap1.rows;
		Mat traversal_img(Nheight, Nwidth, gray_image.type(), Scalar(0, 0));// Scalar(0, 0)
		for(int i=0;i<Nheight;i++)//.at<>(row,col)
			for (int j = 0; j <Nwidth; j++)
			{
				int x = (Nmap1.at<float>(i, j) - 100-0.5) / ((float)JumpCol);//int x= (int)(Start_Col + JumpCol*map1.at<float>(j,i) + 0.5);
				int y= (Nmap2.at<float>(i, j) - 85-0.5)/ ((float)JumpLine);//int y= (int)(Start_Line + JumpLine*map2.at<float>(j, i) + 0.5);
				if(x<CompressImg.cols&&x>=0&&y<CompressImg.rows&&y>=0)
					traversal_img.at<uchar>(i, j) = CompressImg.at<uchar>(y, x);
			}
		imshow("traversal_img View", traversal_img);
		remap(gray_image, roiImg, Nmap1, Nmap2, INTER_NEAREST); //INTER_LINEAR);
		remap(gray_image, newImg, map1, map2, INTER_NEAREST);// INTER_LINEAR);
		for (int i = 0; i < 4; i++)
		{
			circle(newImg, imgPts[i], 3, Scalar(0, 0, 255), -1, 8);
			std::cout << "Image point: " << imgPts[i] << std::endl;
		}
		imshow("bird View", newImg);
		imshow("roi View", roiImg);
		int c = waitKey();
	}

//===================================================================================================================	
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

	
	vector<Point3f> objVtrPts;
	Mat objVtrPtsM;
	for (int i=0; i < board_h*board_w; i++)
	{
		int x = i % 9;
		int y = i / 9;
		objVtrPts.push_back(Point3f(x, y, 0));
	}
	Mat(objVtrPts).convertTo(objVtrPtsM, CV_32F);

	vector<Point2f> imgVtrPts;

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

