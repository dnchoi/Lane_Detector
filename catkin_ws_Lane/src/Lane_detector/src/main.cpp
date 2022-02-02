#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp> // Include OpenCV API

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "utils.h"
#include <pthread.h>
#include <gsl/gsl_fit.h>

boost::property_tree::ptree pt;
using namespace std;
using namespace cv;

/* ini file parameters */
int filtAng = 0;
int UpHorizLn = 0, LoHorizLn = 0;
int LRoiPt_x = 0, LRoiPt_y = 0;
int RRoiPt_x = 0, RRoiPt_y = 0;

int ROI_Lframe_w = 0, ROI_Lframe_h = 0;
int ROI_Rframe_w = 0, ROI_Rframe_h = 0;

int Threshold_min = 0, Threshold_max = 0;
std::string cam_video;

float steering_Kd, steering_Ki, steering_Kp = 0.0; //PID parameter
float speed_Kd, speed_Ki, speed_Kp = 0.0; //PID parameter

int Goal_Speed;
int monitor_flag;
int show_view;

////////////////////////////////////////////

float accelIntensity, brakeIntensity = 0.0; //PID parameter

float dt = 0.02; //PID parameter

float iErr_s = 0; //PID parameter
float iErr_w = 0; //PID parameter
float prevErr_s = 0; //PID parameter
float prevErr_w = 0; //PID parameter
float iErr_st = 0; //PID parameter
float iErr_wt = 0; //PID parameter
float prevErr_st = 0; //PID parameter
float prevErr_wt = 0; //PID parameter

float cur_speed = 2000; //PID parameter

/* CAN communication Parameters */
std::string send_msg = "9000,2000,0";

int receive_steering = 8100;
int receive_break_hold = 252;
int receive_break_open = 0;
int receive_speed = 2000;
int recevie_AGV_status = 0000000;
int receive_AGV_life = 0000000;

////////////////////////////////////////////

float speedControl(float target_speed, float now_speed, float Kp, float Ki, float Kd)
{
    float P = 0;
    float I = 0;
    float D = 0;
    float PID_out = 0;

    float currentSpeed = now_speed;
    float newTargetSpeed = target_speed;

    float err = newTargetSpeed - currentSpeed;

    P = Kp * err;
    iErr_st = iErr_st + err * dt;
    I = Ki * iErr_st;
    D = Kd * (err - prevErr_st) / dt;

    prevErr_st = err;

    PID_out = P + I + D;

    if (PID_out > 0)
    {
        accelIntensity = MIN(PID_out, 255);
        brakeIntensity = 0;
    }
    else if(PID_out < 0)
    {
        accelIntensity = 0;
        brakeIntensity = MIN(PID_out *(-1), 255);
    }

    if(monitor_flag == 1){
        std::cout << "@@SPEED@@ Kp : " << Kp << " Ki : " << Ki << " Kd : " << Kd << " err : " << err << " P : " << P << " I : " << I << " D : " << D << " PID : " << PID_out << " ACC inten : " << accelIntensity << " BRE inten : " << brakeIntensity << std::endl;
    }
    return PID_out;
}

float steeringControl(float TargetAngle, float Kp, float Ki, float Kd)
{
    float P = 0;
    float I = 0;
    float D = 0;
    float PID_out = 0;

    float err = TargetAngle;

    P = Kp * err;

    iErr_s = iErr_s + err * dt;
    I = Ki * iErr_s;

    D = Kd * (err - prevErr_s) / dt;

    prevErr_s = err;

    PID_out = P + I + D;
    if(monitor_flag == 1){
        std::cout << "@@STEER@@ Kp : " << Kp << " Ki : " << Ki << " Kd : " << Kd << " err : " << err << " P : " << P << " I : " << I << " D : " << D << " PID : " << PID_out << std::endl;
    }
    return PID_out;
}

void receive_callback(const std_msgs::String::ConstPtr& msg)
{
    std::string read_data;
    read_data = msg->data.c_str();

    std::vector<string> answer;
    std::stringstream ss(msg->data);
    std::string temp;

    while (getline(ss, temp, ',')) {
        answer.push_back(temp);
    }
    for (int i = 0; i < answer.size(); i++)
    {
        receive_steering = stoi(answer[0]);
        receive_break_hold = stoi(answer[1]);
        receive_break_open = stoi(answer[2]);
        receive_speed = stoi(answer[3]);
        recevie_AGV_status = stoi(answer[4]);
        receive_AGV_life = stoi(answer[5]);
    }
    if(monitor_flag == 1){
        std::cout << "@@RECEIVE@@ : " << receive_steering << " - " << receive_break_hold << " - " << receive_break_open << " - " << receive_speed << " - " << recevie_AGV_status << " - " << receive_AGV_life << std::endl;
    }
}

cv::Mat region_of_interest(cv::Mat img_edges, cv::Point *points)
{
	cv::Mat img_mask = cv::Mat::zeros(img_edges.rows, img_edges.cols, CV_8UC3);


	cv::Scalar ignore_mask_color = Scalar(255, 255, 255);
	const cv::Point* ppt[1] = { points };
	int npt[] = { 4 };


	//filling pixels inside the polygon defined by "vertices" with the fill color
	cv::fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);


	//returning the image only where mask pixels are nonzero
	cv::Mat img_masked;
	cv::bitwise_and(img_edges, img_mask, img_masked);

	return img_masked;
}

void filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	cv::Scalar lower_white = Scalar(0, 0, 0);
	cv::Scalar upper_white = Scalar(80,30,80);
	cv::Scalar lower_yellow = Scalar(10, 100, 100);
	cv::Scalar upper_yellow = Scalar(40, 255, 255);

	cv::UMat img_bgr;
	_img_bgr.copyTo(img_bgr);
	cv::UMat img_hsv, img_combine;
	cv::UMat white_mask, white_image;
	cv::UMat yellow_mask, yellow_image;

	cv::inRange(img_bgr, lower_white, upper_white, white_mask);
	cv::bitwise_and(img_bgr, img_bgr, white_image, white_mask);

	cv::inRange(img_bgr, lower_white, upper_white, white_mask);
	cv::bitwise_and(img_bgr, img_bgr, yellow_image, white_mask);

	cv::Mat red_img = Mat::zeros(480, 640, CV_8UC3);
	red_img.setTo(cv::Scalar(0,0,255));

	//Combine the two above images
	cv::addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);

	img_combine.copyTo(img_filtered);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_detector");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("CAN_send", 1000);
	ros::Subscriber sub = n.subscribe("CAN_RECEIVE", 1000, receive_callback);
    try
    {
        boost::property_tree::ini_parser::read_ini("config.ini", pt);
    }
    catch(boost::property_tree::ini_parser_error& err)
    {
        std::cout << err.message() << std::endl;
       	return 0;
    }
    cam_video = pt.get<std::string>("cam_video.value");
    filtAng = pt.get<int>("filtAng.value");
    UpHorizLn = pt.get<int>("UpHorizLn.value");
    LoHorizLn = pt.get<int>("LoHorizLn.value");
    LRoiPt_x = pt.get<int>("LRoiPt_x.value");
    LRoiPt_y = pt.get<int>("LRoiPt_y.value");
    RRoiPt_x = pt.get<int>("RRoiPt_x.value");
    RRoiPt_y = pt.get<int>("RRoiPt_y.value");
    ROI_Lframe_w = pt.get<int>("ROI_Lframe_w.value");
    ROI_Lframe_h = pt.get<int>("ROI_Lframe_h.value");
    ROI_Rframe_w = pt.get<int>("ROI_Rframe_w.value");
    ROI_Rframe_h = pt.get<int>("ROI_Rframe_h.value");
    Threshold_min = pt.get<int>("Threshold_min.value");
    Threshold_max = pt.get<int>("Threshold_max.value");

    steering_Kp = pt.get<float>("steering_Kp.value");
    steering_Ki = pt.get<float>("steering_Ki.value");
    steering_Kd = pt.get<float>("steering_Kd.value");

    speed_Kp = pt.get<float>("speed_Kp.value");
    speed_Ki = pt.get<float>("speed_Ki.value");
    speed_Kd = pt.get<float>("speed_Kd.value");

    Goal_Speed = pt.get<int>("goal.value");
    monitor_flag = pt.get<int>("monitor.value");
    show_view = pt.get<int>("show.value");
	cout << "cam_video : " << cam_video << "\tfiltAng : " << filtAng << "\tfiltAng : " << filtAng << std::endl;
	cout << "UpHorizLn : " << UpHorizLn << "\tLoHorizLn : " << LoHorizLn << std::endl;
	cout << "LRoiPt_x : " << LRoiPt_x << "\tLRoiPt_y : " << LRoiPt_y << "\tRRoiPt_x : " << RRoiPt_x << "\tRRoiPt_y : " << RRoiPt_x << std::endl;
	cout << "ROI_Lframe_w : " << ROI_Lframe_w << "\tROI_Lframe_h : " << ROI_Lframe_h << "\tROI_Rframe_w : " << ROI_Rframe_w << "\tROI_Rframe_h : " << ROI_Rframe_h << std::endl;

    ///////////////////////////// Variables //////////////////////////////////
    float alpha = 0.1;
    float angle_ratio = 1;
    cv::Point LRoiPt(LRoiPt_x, LRoiPt_y);
    cv::Point RRoiPt(RRoiPt_x, RRoiPt_y);
    cv::Point HeadingPt(320, UpHorizLn + LRoiPt.y);
    ///////////////////////////// Variables //////////////////////////////////
    char keyIn = 0;
    char is_Working = 0;
    double tick_st, tick_end, time_elap;
    CvPolarPoint LLnMovAvg = { 0 }, RLnMovAvg = { 0 }, LLnPt, RLnPt;
    cv::VideoCapture capture;

	if(cam_video.length() > 2)
	{
		capture.open(cam_video);
	}
	else
	{
		capture.open(atoi(cam_video.c_str()));
	}
	if(!capture.isOpened())
	{
		CV_Assert("@@@@@@@@@@@@@@@@@@CAM or Video is not Open!! checked config.ini file@@@@@@@@@@@@@@@@@@");
		return 0;
	}
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);
    int handle;
    int j = 0;
    int heading_flag = 1;
    cv::Mat PID_graph;
	cv::Rect rect_L_ROI(LRoiPt_x, LRoiPt_y, ROI_Lframe_w, ROI_Lframe_h);
	cv::Rect rect_R_ROI(RRoiPt_x, RRoiPt_y, ROI_Rframe_w, ROI_Rframe_h);
	cv::Rect rect_Center_ROI(LRoiPt_x, 0, ROI_Lframe_w + ROI_Rframe_w, 480);

    cv::Point PID_array[640];
	cv::Mat PID_graph_2 = Mat::zeros(480, 640, CV_8UC3);;

	while (keyIn != 'q' || is_Working == 'w')
	{
		if(j > 640)
		{
			j =0;
			PID_graph_2 = Mat::zeros(480, 640, CV_8UC3);
		}
		cv::line(PID_graph_2, cv::Point(0, 240), cv::Point(640, 240), cv::Scalar(0, 255, 0), 1, LINE_AA, 0);
		std_msgs::String msg;
		std::stringstream ss;
		// tick_st = cv::getTickCount();
		tick_st = static_cast<double>(cv::getTickCount());

		cv::Mat frame1, frame;
		capture.read(frame1);
		if (!frame1.empty()) 
		{
			cv::resize(frame1, frame1, cv::Size(640, 480));
			frame1.copyTo(frame);
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
			cv::rectangle(frame, cv::Point(LRoiPt_x, LRoiPt_y), cv::Point(LRoiPt_x + ROI_Lframe_w, LRoiPt_y + ROI_Lframe_h), Scalar(0, 255, 255), 1);
			cv::rectangle(frame, cv::Point(RRoiPt_x, RRoiPt_y), cv::Point(RRoiPt_x + ROI_Rframe_w, RRoiPt_y + ROI_Rframe_h), Scalar(0, 255, 255), 1);
			cv::rectangle(frame, rect_Center_ROI, Scalar(255, 255, 255), 1);
			cv::Mat tmp_ROI_Lframe, ROI_Lframe, edge_Lframe;
			cv::Mat tmp_ROI_Rframe, ROI_Rframe, edge_Rframe;
			cv::Mat Center_ROI;

			tmp_ROI_Lframe = frame1(rect_L_ROI);
			tmp_ROI_Rframe = frame1(rect_R_ROI);
			// tmp_Center_ROI = frame1(rect_Center_ROI);
			tmp_ROI_Lframe.copyTo(ROI_Lframe);
			tmp_ROI_Rframe.copyTo(ROI_Rframe);
			// tmp_Center_ROI.copyTo(Center_ROI);
			float trap_bottom_width = 0.80;  
			float trap_top_width = 0.40;     
			float trap_height = 0.9;         
			Point points[4];
			points[0] = Point((640 * (1 - trap_bottom_width)) / 2, 480);
			points[1] = Point((640 * (1 - trap_top_width)) / 2, 480 - 480 * trap_height);
			points[2] = Point(640 - (640 * (1 - trap_top_width)) / 2, 480 - 480 * trap_height);
			points[3] = Point(640 - (640 * (1 - trap_bottom_width)) / 2, 480);

			Center_ROI = region_of_interest(frame1, points);
			Center_ROI = Center_ROI - Scalar(50, 100, 50);
			// ROI_Lframe = tmp_ROI_Lframe.clone();
			// ROI_Rframe = tmp_ROI_Rframe.clone();
			cv::Mat filtered_image;
			filter_colors(Center_ROI, filtered_image);
			addWeighted(frame, 0.8, filtered_image, 1.0, 0.0, frame);

			medianBlur(ROI_Lframe, ROI_Lframe, 7);
			medianBlur(ROI_Rframe, ROI_Rframe, 7);
			cv::Canny(ROI_Lframe, edge_Lframe, Threshold_min, Threshold_max, 3);
			cv::Canny(ROI_Rframe, edge_Rframe, Threshold_min, Threshold_max, 3);

			vector<Vec2f> L_lines;
			HoughLines(edge_Lframe, L_lines, 1, CV_PI / 180, 20, 0, 0);
			vector<Vec2f> R_lines;
			HoughLines(edge_Rframe, R_lines, 1, CV_PI / 180, 20, 0, 0);

			LLnPt = filtHoughlines(0, L_lines, filtAng, ROI_Lframe);
			RLnPt = filtHoughlines(1, R_lines, filtAng, ROI_Rframe);

			if (fabs(LLnPt.rho) <= 200)
			{
				LLnMovAvg.rho = alpha * LLnPt.rho + (1 - alpha) * LLnMovAvg.rho;
				LLnMovAvg.theta = alpha * LLnPt.theta + (1 - alpha) * LLnMovAvg.theta;
			}
			if (fabs(RLnPt.rho) <= 200)
			{
				RLnMovAvg.rho = alpha * RLnPt.rho + (1 - alpha) * RLnMovAvg.rho;
				RLnMovAvg.theta = alpha * RLnPt.theta + (1 - alpha) * RLnMovAvg.theta;
			}

			drawPolarline(LLnMovAvg, cv::Point(LRoiPt_x, LRoiPt_y), frame);
			drawPolarline(RLnMovAvg, cv::Point(RRoiPt_x, RRoiPt_y), frame);

			cv::Point UpLfLnPt = add_i(getIntersect(polar2cart(LLnMovAvg), (-1 / tanf(LLnMovAvg.theta)), cv::Point(0, UpHorizLn), 0), LRoiPt);
			cv::Point UpRtLnPt = add_i(getIntersect(polar2cart(RLnMovAvg), (-1 / tanf(RLnMovAvg.theta)), cv::Point(0, UpHorizLn), 0), RRoiPt);
			cv::Point LoLfLnPt = add_i(getIntersect(polar2cart(LLnMovAvg), (-1 / tanf(LLnMovAvg.theta)), cv::Point(0, LoHorizLn), 0), LRoiPt);
			cv::Point LoRtLnPt = add_i(getIntersect(polar2cart(RLnMovAvg), (-1 / tanf(RLnMovAvg.theta)), cv::Point(0, LoHorizLn), 0), RRoiPt);
			cv::line(frame, UpLfLnPt, UpRtLnPt, CV_RGB(255, 0, 0), 1, 8, 0);
			cv::line(frame, LoLfLnPt, LoRtLnPt, CV_RGB(255, 0, 0), 1, 8, 0);
	// 		cout << UpLfLnPt.x << "\t" << UpLfLnPt.y << "\t" << UpRtLnPt.x << "\t" << UpRtLnPt.y << "\t" << UpLfLnPt.x - UpRtLnPt.x << "\t" << UpLfLnPt.y - UpRtLnPt.y << endl;
			cv::Point UpCenterPt = cv::Point((UpLfLnPt.x + UpRtLnPt.x) / 2, UpHorizLn + LRoiPt.y);
			cv::Point LoCenterPt = cv::Point((LoLfLnPt.x + LoRtLnPt.x) / 2, LoHorizLn + LRoiPt.y);
			cv::circle(frame, UpCenterPt, 2, CV_RGB(0, 255, 0), 3, 8, 0);
			cv::circle(frame, LoCenterPt, 2, CV_RGB(0, 255, 0), 3, 8, 0);
			if(std::abs(UpLfLnPt.x - UpRtLnPt.x) < 200 && j > 30)
			{
				cout << "Vision OUT!" << endl;
				// string sav = "Vision_dead_frame : " + to_string(j) + "_FrontLine Length : " + to_string(std::abs(UpLfLnPt.x - UpRtLnPt.x)) + ".jpg";
				// const char * save_name = sav.c_str();
				// cvSaveImage(save_name, frame);
			}
			if (keyIn == 'a')
			{
				HeadingPt.x -= 2;
			}
			else if (keyIn == 's')
			{
				HeadingPt.x += 2;
			}
			cv::circle(frame, HeadingPt, 3, CV_RGB(255, 0, 0), 3, 8, 0);

			float lane_slope = (float)(UpCenterPt.x - HeadingPt.x) / (float)(LoHorizLn - UpHorizLn);
			float TargetAngle = (rad2deg(atan(lane_slope))) * angle_ratio;

			receive_speed = cur_speed;
			float speed_PID = speedControl(Goal_Speed, receive_speed, speed_Kp, speed_Ki, speed_Kd);
			cur_speed = (cur_speed + speed_PID);
			float steering_PID = steeringControl(TargetAngle, steering_Kp, steering_Ki, steering_Kd);
			float current_Angle = (std::abs((int)(9000+steering_PID)));

			for(int i = 0; i < 640; i++)
			{
				PID_array[i+1] = PID_array[i];
			}

			PID_array[0] = cv::Point(j, (240 - ((int)steering_PID/100)));

			for(int i = 0; i < 640; i++)
			{
				cv::circle(PID_graph_2, PID_array[i], 1, CV_RGB(255,255,255), 1, 1, 0);
			}

			std::string sending_can;
			sending_can = to_string((int)current_Angle) + "," + to_string((int)cur_speed) + "," + "0";

			send_msg = sending_can;
			if(monitor_flag == 1){
				std::cout << "\nTarget steering : " << to_string(std::abs((int)(current_Angle))) << "\tTarget speed : " << to_string((int)cur_speed) << std::endl;
				std::cout << "\nsend_msg : " << send_msg << std::endl;
			}
			ss << send_msg;

			msg.data = ss.str();
			chatter_pub.publish(msg);

			tick_end = static_cast<double>(cv::getTickCount())-tick_st;
			time_elap = tick_end / cv::getTickFrequency();             

			char tmpStr[100];
			sprintf(tmpStr, "Processing Time = %2.3fms Target Angle = %2.2f Current Speed = %5d", time_elap, (current_Angle), (int)cur_speed);
			cv::putText(frame, tmpStr, cv::Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 255, 0), 1, 1);
			
			if(show_view == 1)
			{
				namedWindow("frame");
				namedWindow("ROI_L");
				namedWindow("ROI_R");
				namedWindow("Canny_L");
				namedWindow("Canny_R");
				namedWindow("Segmentation");
				namedWindow("PID");
				namedWindow("filtered_image");

				moveWindow("frame", 0,0);
				moveWindow("ROI_L", frame.size().width, 0);
				moveWindow("ROI_R", frame.size().width + ROI_Lframe.size().width, 0);
				moveWindow("Canny_L", frame.size().width, ROI_Lframe.size().height+50);
				moveWindow("Canny_R", frame.size().width + edge_Lframe.size().width, ROI_Rframe.size().height+50);
				moveWindow("Segmentation", frame.size().width + ROI_Lframe.size().width + ROI_Rframe.size().width, 0);
				moveWindow("PID", 0, frame.size().height+100);
				moveWindow("filtered_image", frame.size().width + ROI_Lframe.size().width + ROI_Rframe.size().width, Center_ROI.size().height+100);
				
				cv::imshow("frame", frame);
				cv::imshow("ROI_L", ROI_Lframe);
				cv::imshow("ROI_R", ROI_Rframe);
				cv::imshow("Canny_L", edge_Lframe);
				cv::imshow("Canny_R", edge_Rframe);
				cv::imshow("Segmentation", Center_ROI);
				cv::imshow("PID", PID_graph_2);
				cv::imshow("filtered_image", filtered_image);
			}

			keyIn = cv::waitKey(5);
			if(show_view != 1)
			{
				// char break_key = 'c';
				// scanf("%c", &break_key);
				if(is_Working == 'w')
				{
					break;
				}
			}

			ros::spinOnce();
			j++;
			// loop_rate.sleep();
	
		}
	}
	capture.release();

}
