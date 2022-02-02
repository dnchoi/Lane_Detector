// /*****************************************************************************
// | Project Name: Advent-RP
// |
// |  General code: Version: 1.0
// |
// |  Description: Lane Detection & Keeping Module
// |
// |  Author(s): Joonwoo Son
// |
// |-----------------------------------------------------------------------------
// |               C O P Y R I G H T
// |-----------------------------------------------------------------------------
// | Copyright (c) 2016 HumanLAB (www.humanlab.kr)           All rights reserved.
// |****************************************************************************/

// typedef struct CvPolarPoint{
// 	float rho;
// 	float theta;
// } CvPolarPoint;

// #define MIN(a,b) ((a)>(b)?(b):(a))

// float rad2deg(float rad) { return float(rad * 180 / CV_PI); }
// float deg2rad(float deg) { return float(deg * CV_PI / 180); }

// CvPoint polar2cart(CvPolarPoint	Pt)
// {
// 	int X = (int)(Pt.rho * cos(Pt.theta));
// 	int	Y = (int)(Pt.rho * sin(Pt.theta));

// 	CvPoint res = cvPoint(X, Y);
// 	return res;
// }

// CvPoint add_i(CvPoint b, CvPoint a) { return cvPoint(b.x + a.x, b.y + a.y); }

// void drawHoughlines(CvSeq* lines, IplImage* frame)
// {

// 	for (int i = 0; i < MIN(lines->total, 100); i++)
// 	{
// 		float* line = (float*)cvGetSeqElem(lines, i);
// 		float rho = line[0];
// 		float theta = line[1];

// 		CvPoint pt1, pt2;

// 		double a = cos(theta), b = sin(theta);
// 		double x0 = a*rho, y0 = b*rho;
// 		pt1.x = cvRound(x0 + 1000 * (-b));
// 		pt1.y = cvRound(y0 + 1000 * (a));
// 		pt2.x = cvRound(x0 - 1000 * (-b));
// 		pt2.y = cvRound(y0 - 1000 * (a));
// 		cvLine(frame, pt1, pt2, CV_RGB(0, 255, 0), 1, CV_AA, 0);
// 	}
// }

// CvPolarPoint filtHoughlines(bool isRight, CvSeq* lines, int ang, IplImage* frame)
// {
// 	CvPoint pt1, pt2;
// 	CvPolarPoint avgPt;
// 	int max_ang = 50, cnt = 0;
// 	float tmpTheta = 0, sumTheta = 0, sumRho = 0, sumDeviation = 0;

// 	for (int i = 0; i < MIN(lines->total, 100); i++)
// 	{
// 		float* line = (float*)cvGetSeqElem(lines, i);
// 		float rho = line[0];
// 		float theta = line[1];

// 		if (isRight == true)
// 		{
// 			tmpTheta = CV_PI - theta;
// 		}
// 		else
// 		{
// 			tmpTheta = theta;
// 		}

// 		if (tmpTheta < deg2rad(max_ang-ang) && tmpTheta > deg2rad(ang))
// 		{
// 			//acculate theta & rho to calc average
// 			sumTheta += theta;
// 			sumRho += rho;
// 			cnt++;

// 			// Draw Houghlines
// 			double a = cos(theta), b = sin(theta);
// 			double x0 = a*rho, y0 = b*rho;
// 			pt1.x = cvRound(x0 + 1000 * (-b));
// 			pt1.y = cvRound(y0 + 1000 * (a));
// 			pt2.x = cvRound(x0 - 1000 * (-b));
// 			pt2.y = cvRound(y0 - 1000 * (a));
// 			//std::cout << "pt1 - x : " << pt1.x << "\tpt1 - y : " << pt1.y << "\tpt2 - x : " << pt2.x << "\tpt2 - y : " << pt2.y << std::endl;
// 			cvLine(frame, pt1, pt2, CV_RGB(0, 255, 0), 1, CV_AA, 0);
// 		}
// 	}

// 	float avgRho = sumRho / cnt;
// 	float avgTheta = sumTheta /cnt;
// 	avgPt.rho = avgRho;
// 	avgPt.theta = avgTheta;

// 	cvCircle(frame, polar2cart(avgPt), 10, CV_RGB(255, 0, 0), 2, 8, 0);

// 	return avgPt;
// }

// void drawPolarline(CvPolarPoint pt, CvPoint offset, IplImage* frame)
// {

// 	float rho = pt.rho;
// 	float theta = pt.theta;

// 	CvPoint pt1, pt2;

// 	double a = cos(theta), b = sin(theta);
// 	double x0 = a*rho, y0 = b*rho;
// 	pt1.x = cvRound(x0 + 1000 * (-b));
// 	pt1.y = cvRound(y0 + 1000 * (a));
// 	pt2.x = cvRound(x0 - 1000 * (-b));
// 	pt2.y = cvRound(y0 - 1000 * (a));
// 	cvLine(frame, add_i(pt1, offset), add_i(pt2, offset), CV_RGB(0, 0, 255), 1, CV_AA, 0);
// }

// CvPoint getIntersect(CvPoint pt1, float slope1, CvPoint pt2, float slope2)
// {
// 	CvPoint intersect_pt = { 0 };

// 	// x = (m1x1 - m2x2 + y2 - y1) / (m1 - m2)
// 	intersect_pt.x = ((slope1 * pt1.x) - (slope2 * pt2.x) + pt2.y - pt1.y) / (slope1 - slope2);
// 	// y = m1(x - x1) + y1
// 	intersect_pt.y = slope1 * (intersect_pt.x - pt1.x) + pt1.y;

// 	return intersect_pt;
// }
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp> // Include OpenCV API

using namespace std;
using namespace cv;

typedef struct CvPolarPoint{
        float rho;
        float theta;
} CvPolarPoint;

#define MIN(a,b) ((a)>(b)?(b):(a))

float rad2deg(float rad) { return float(rad * 180 / CV_PI); }
float deg2rad(float deg) { return float(deg * CV_PI / 180); }

cv::Point polar2cart(CvPolarPoint	Pt)
{
        int X = (int)(Pt.rho * cos(Pt.theta));
        int	Y = (int)(Pt.rho * sin(Pt.theta));

        cv::Point res = cv::Point(X, Y);
        return res;
}

cv::Point add_i(cv::Point b, cv::Point a) { return cv::Point((b.x + a.x), (b.y + a.y)); }

CvPolarPoint filtHoughlines(bool isRight, vector<Vec2f> lines, int ang, Mat frame)
{

    cv::Point pt1, pt2;
    CvPolarPoint avgPt;
    int max_ang = 90, cnt = 0;
    float tmpTheta = 0, sumTheta = 0, sumRho = 0, sumDeviation = 0;
    vector<Vec2f> get_lines = lines;
    for (int i = 0; i < MIN(lines.size(), 100); i++)
    {
        float rho = get_lines[i][0];
        float theta = get_lines[i][1];

        if (isRight == true)
        {
                tmpTheta = CV_PI - theta;
        }
        else
        {
                tmpTheta = theta;
        }

        if (tmpTheta < deg2rad(max_ang-ang) && tmpTheta > deg2rad(ang))
        {
                //acculate theta & rho to calc average
                sumTheta += theta;
                sumRho += rho;
                cnt++;

                // Draw Houghlines
                double a = cos(theta), b = sin(theta);
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(x0 + 1000 * (-b));
                pt1.y = cvRound(y0 + 1000 * (a));
                pt2.x = cvRound(x0 - 1000 * (-b));
                pt2.y = cvRound(y0 - 1000 * (a));
                //std::cout << "pt1 - x : " << pt1.x << "\tpt1 - y : " << pt1.y << "\tpt2 - x : " << pt2.x << "\tpt2 - y : " << pt2.y << std::endl;
                cv::line(frame, pt1, pt2, CV_RGB(0, 255, 0), 1);
        }
    }

    float avgRho = sumRho / cnt;
    float avgTheta = sumTheta /cnt;
    avgPt.rho = avgRho;
    avgPt.theta = avgTheta;

    circle(frame, polar2cart(avgPt), 10, CV_RGB(255, 0, 0), 2, 8, 0);

    return avgPt;
}

void drawPolarline(CvPolarPoint pt, cv::Point offset, Mat frame)
{

        float rho = pt.rho;
        float theta = pt.theta;

        cv::Point pt1, pt2;

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;

        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        cv::Point line_pt1 = add_i(pt1, offset);
        cv::Point line_pt2 = add_i(pt2, offset);
//        cout << line_pt1.x << "\t" << line_pt1.y << "\t" << line_pt2.x << "\t" << line_pt2.y << endl;
        cv::line(frame, line_pt1, line_pt2, CV_RGB(0, 0, 255), 2);
}

cv::Point getIntersect(cv::Point pt1, float slope1, cv::Point pt2, float slope2)
{
        cv::Point intersect_pt = { 0 };

        // x = (m1x1 - m2x2 + y2 - y1) / (m1 - m2)
        intersect_pt.x = ((slope1 * pt1.x) - (slope2 * pt2.x) + pt2.y - pt1.y) / (slope1 - slope2);
        // y = m1(x - x1) + y1
        intersect_pt.y = slope1 * (intersect_pt.x - pt1.x) + pt1.y;

        return intersect_pt;
}
