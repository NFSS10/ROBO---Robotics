#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <algorithm>
#include "geometry_msgs/Twist.h"


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

bool ipmLeftDone;
bool ipmRightDone;
int ROBOTPOS_R_X = 36;
int ROBOTPOS_R_Y = 104;
int ROBOTPOS_L_X = 137;
int ROBOTPOS_L_Y = 107;

double leftDistToLine;// meters
double leftAngle; // rad
double rightDistToLine;// meters
double rightAngle; // rad
double rightGreenPercentage; //green percentage in the right camera

/// Intrisic Camera parameters
/*
    [   alpha_x ,   beta    ,   x0
        0       ,   alpha_Y ,   y0
        0       ,   0       ,   1   ]
    alpha_x, alpha_y -> focal length
    x0, y0           -> principal point
*/
//------------------------Camera right parameters----------------------------------------------
cv::Mat cameraRight_intrinsic = (cv::Mat_<double>(3,3) << \
                                 130.81305          , 0                     , 79.81980 ,\
                                 0                  , 131.22421             , 58.91209,\
                                 0                  , 0                     , 1);

cv::Mat cameraRight_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                     -1,    0,  0,  0.612,\
                                     0,   -1,  0,  0.0,\
                                     0,    0,  1,  -0.004,\
                                     0,    0,  0,  1);

cv::Mat cameraRight_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                   0.330007,      0.918154,       0.219294,     -0.540327012,\
                                   0.562501,     -0.004706,      -0.826783,     -0.018409465,\
                                   -0.758082,      0.396197,      -0.518016,      1.038223574,\
                                   0       ,      0       ,       0       ,      1);
// Distortion coeficients
cv::Mat cameraRight_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);

//------------------------Camera left parameters----------------------------------------------

cv::Mat cameraLeft_intrinsic = (cv::Mat_<double>(3,3) << \
                                132.31872          , 0                     , 74.70743 ,\
                                0                  , 132.17822             , 52.77469,\
                                0                  , 0                     , 1);

cv::Mat cameraLeft_T_chess_robot = (cv::Mat_<double>(4,4) <<  \
                                    -1,    0,  0,  0.633,\
                                    0,   -1,  0,  0.72,\
                                    0,    0,  1,  -0.004,\
                                    0,    0,  0,  1);

cv::Mat cameraLeft_T_cam_chess = (cv::Mat_<double>(4,4) << \
                                  -0.436125,      0.850786,      -0.293187,     0.081835848,\
                                  0.553047,     -0.003607,      -0.833142,     0.014687224,\
                                  -0.709883,     -0.525501,      -0.468951,     1.363192844,\
                                  0       ,      0       ,       0       ,     1);
// Distortion coeficients
cv::Mat cameraLeftt_dist_coef = (cv::Mat_<double>(1,4) << -0.275678598507515 , 0.045106260288961 ,
                                 0.004883645512607 , 0.001092737340199);
//--------------------------------------------------------------------------------------------

ros::Publisher dist_angle_pub;
ros::Publisher crossWalk_pub;

///--------------------------------------------------------------------------------------------------
void imageMergeAndTrack()
{
    std::cout << "[Used line - Left] Distance: " << leftDistToLine << " Angle: " << leftAngle << std::endl;
    std::cout << "[Used line - Right] Distance: " << rightDistToLine << " Angle: " << rightAngle << std::endl;
    std::cout << "[Right Camera Green Percentage] " << rightGreenPercentage << std::endl << std::endl;

	/// Publish Distance and Angle Info
	std_msgs::Float64MultiArray array;
	array.data.clear();
	array.data.push_back(leftDistToLine);
	array.data.push_back(leftAngle);
	array.data.push_back(rightDistToLine);
	array.data.push_back(rightAngle);
    array.data.push_back(rightGreenPercentage);    


    //clean values
    leftDistToLine = -123456789;
    leftAngle = -123456789;
    rightDistToLine = -123456789;
    rightAngle = -123456789;

	ros::Rate loop_rate(200);
	dist_angle_pub.publish(array);
	ros::spinOnce();
	loop_rate.sleep();

}

void HoughLinesProcessLeftImg(cv::Mat img_rgb)
{
    //Process image
    cv::Mat src, dst, color_dst, img_rgb_blured;
    cv::Mat img_bw;
    cv::GaussianBlur( img_rgb, img_rgb_blured, cv::Size(7,7), 0, 0, cv::BORDER_DEFAULT );
    cv::cvtColor(img_rgb_blured, img_bw, cv::COLOR_BGR2GRAY);
    cv::threshold(img_bw, img_bw, 127, 255,0);
    Canny( img_bw, dst, 50, 200, 3);

    int cropWith = 65;
    int cropHeight = 45;
    //crop left image
    cv::Rect rightROI(cropWith, cropHeight, //start point of roi
                     dst.size().width-cropWith, dst.size().height - cropHeight); //roi size
    dst = dst(rightROI);

    cvtColor(dst, color_dst, CV_GRAY2BGR );

    //Get hough lines
    std::vector<cv::Vec2f> lines;
    HoughLines( dst, lines, 1, (CV_PI/2)/180, 20);

    //Calculate line to use as guide
    int minDistance = 999999999;
    cv::Point usedLinePt1;
    cv::Point usedLinePt2;
    float usedLineAngle;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        cv::Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        cv::Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        
        double a2 = pt1.y - pt2.y;
        double b2 = pt2.x - pt1.x;
        double c = (pt1.x-pt2.x)*pt1.y + (pt2.y-pt1.y)*pt1.x;
        int x = cvRound((((-b2)*(ROBOTPOS_L_Y-cropHeight) - c)/a2));
        
        if((ROBOTPOS_L_X - x - cropWith) < minDistance)
        {
            minDistance = ROBOTPOS_L_X - x - cropWith;
            usedLinePt1 = pt1;
            usedLinePt2 = pt2;
            usedLineAngle = lines[i][1];;
        }
        
    }
    //Draw line
    line(color_dst, usedLinePt1, usedLinePt2, cv::Scalar(0,0,255), 5, 1 );

    //Visualize whats going on
    imshow( "Detected Lines Left", color_dst);

    leftDistToLine =  minDistance;
    leftAngle = usedLineAngle;
}


void HoughLinesProcessRightImg(cv::Mat img_rgb)
{
    //Process image
    cv::Mat src, dst, color_dst, img_rgb_blured;
    cv::Mat img_bw;
    cv::GaussianBlur( img_rgb, img_rgb_blured, cv::Size(7,7), 0, 0, cv::BORDER_DEFAULT );
    cv::cvtColor(img_rgb_blured, img_bw, cv::COLOR_BGR2GRAY);
    cv::threshold(img_bw, img_bw, 127, 255,0);
    Canny( img_bw, dst, 50, 200, 3);

    //crop right image
    int cropWith = 110;
    int cropHeight = 40;
    cv::Rect rightROI(0, cropHeight, //start point of roi
                        cropWith, dst.size().height - cropHeight); //roi size
    dst = dst(rightROI);

    cvtColor(dst, color_dst, CV_GRAY2BGR );
    

    //Get hough lines
    std::vector<cv::Vec2f> lines;
    HoughLines( dst, lines, 1, (CV_PI/2)/180, 25 );

    //Calculate line to use as guide
    int minDistance = 999999999;
    cv::Point usedLinePt1;
    cv::Point usedLinePt2;
    float usedLineAngle;

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        cv::Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        cv::Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        
        double a2 = pt1.y - pt2.y;
        double b2 = pt2.x - pt1.x;
        double c = (pt1.x-pt2.x)*pt1.y + (pt2.y-pt1.y)*pt1.x;

        int x = cvRound((((-b2)*(ROBOTPOS_R_Y-cropHeight) - c)/a2));
        if((x - ROBOTPOS_R_X) < minDistance)
        {
            minDistance = x - ROBOTPOS_R_X;
            usedLinePt1 = pt1;
            usedLinePt2 = pt2;
            usedLineAngle = lines[i][1];
        }

    }
    //Draw line
    line(color_dst, usedLinePt1, usedLinePt2, cv::Scalar(0,0,255), 5, 1 );

    //Calculate green percentage
    cv::Scalar sums;
    sums = cv::sum(img_rgb);

    double totalSum = sums[0] + sums[1] + sums[2];
    float G = roundf((sums[1]/totalSum) * 100) / 100; ;

    //Visualize whats going on
    imshow( "Detected Lines Right", color_dst);

    //set values
    rightDistToLine =  minDistance;
    rightAngle = usedLineAngle;
    rightGreenPercentage = G;

}




///------------------------------------------------LEFT--------------------------------------------------
void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        HoughLinesProcessLeftImg(img_rgb);

        cv::imshow("left rgb", img_rgb);
        
        ipmLeftDone = true;
        
        uint8_t k = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }

}
///----------------------------------------------RIGHT-----------------------------------------------

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        HoughLinesProcessRightImg(img_rgb);
       

        cv::imshow("right rgb", img_rgb);

        ipmRightDone = true; 
        uint8_t k = cv::waitKey(1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    if(ipmRightDone && ipmLeftDone)
    {
        ipmLeftDone = false;
        ipmRightDone = false;
        imageMergeAndTrack();
    }
    return;
}

///--------------------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    std::cout << "\nStarting conde tracking\n" << std::endl;

    /// init variables
    ros::init(argc, argv, "conde_tracking_node");
    ros::NodeHandle nh("~");
    cv::namedWindow("right rgb");
    cv::namedWindow("left rgb");

    cv::namedWindow( "Detected Lines Left");
    cv::namedWindow( "Detected Lines Right");

    std::string cameraRightTopic;
    std::string cameraLeftTopic;

    if(!(nh.getParam("camera_right", cameraRightTopic))){
        std::cerr << "Parameter (camera_right) not found" << std::endl;
        return 0;
    }

    std::cout << "Parameter camera_right: " << cameraRightTopic <<  std::endl;

    if(!(nh.getParam("camera_left", cameraLeftTopic))){
        std::cout << "Parameter (camera_left) not found" << std::endl;
        return 0;
    }
    std::cout << "Parameter camera_left: " << cameraLeftTopic <<  std::endl;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subRight = it.subscribe(cameraRightTopic, 1, imageRightCallback);
    image_transport::Subscriber subLeft = it.subscribe(cameraLeftTopic, 1, imageLeftCallback);

    dist_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("/conde_dist_angle", 1);
    crossWalk_pub = nh.advertise<std_msgs::Bool>("/crossWalk", 1);

    ros::spin();
    //    ros::Rate r(100);
    //    while(ros::ok())
    //    {
    //        ros::spinOnce();
    //        r.sleep();
    //    }
    cv::destroyWindow("view");
}

