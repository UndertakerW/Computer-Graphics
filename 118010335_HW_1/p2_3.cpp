#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core_c.h>
//#include <opencv2/videoio/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;
 
int main(int argc, char** argv)
{
	if(argc != 2)
	{
		return -1;
	}
 
	Mat img;
    img = imread(argv[1], IMREAD_COLOR);    
                                                                                                    //如果想要载入最真实的图像，选择CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR。
	if(img.empty())
	{
		printf("Failed to load image!\n");
		return -1;
	}
    
    // problem 1
    
    cout << "[problem 1]" << endl;
    Mat gray;
 
	cvtColor(img, gray, CV_RGB2GRAY);

    imwrite("p2_3_1.png", gray);

    // namedWindow("gray scale", WINDOW_AUTOSIZE);
    // imshow("gray scale", gray);
    // waitKey(0);

    int size = 255;
    int row = gray.rows;
    int col = gray.cols;
    // Mat gray_norm(row, col, CV_32FC1);
    // gray_norm = gray / (float) size;

    MatrixXf B(row, col);
    cv2eigen(gray, B);

    MatrixXf A = B / size;
    cout << "A" << endl;
    cout << A << endl;

    // problem 2

    cout << "[problem 2]" << endl;
    JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
    MatrixXf U = svd.matrixU();
    MatrixXf V = svd.matrixV();
    MatrixXf S = U.inverse() * A * V.transpose().inverse();
    cout << "U" << endl;
    cout << U << endl;
    cout << "S" << endl;
    cout << S  << endl;
    cout << "V" << endl;
    cout << V << endl;
    cout << "USV\'" << endl;
    cout << U * S * V.transpose() << endl;

    // problem 3 
    cout << "[problem 3]" << endl;
    MatrixXf p3 = U * S.col(0) * V.col(0).transpose() * size;
    Mat p3_cv;
    eigen2cv(p3, p3_cv);
    imwrite("p2_3_3.png", p3_cv);

    // problem 4
    cout << "[problem 4]" << endl;
    MatrixXf p4 = U * S.leftCols(10) * V.leftCols(10).transpose() * size;
    Mat p4_cv;
    eigen2cv(p4, p4_cv);
    imwrite("p2_3_4.png", p4_cv);

    // problem 5
    cout << "[problem 5]" << endl;
    MatrixXf p5 = U * S.leftCols(50) * V.leftCols(50).transpose() * size;
    Mat p5_cv;
    eigen2cv(p5, p5_cv);
    imwrite("p2_3_5.png", p5_cv);

    return 0;
}