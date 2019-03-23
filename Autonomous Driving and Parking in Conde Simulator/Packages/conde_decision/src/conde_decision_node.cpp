#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <mutex>
#include <thread>
#include <chrono>
#include <utility>

using m_clock = std::chrono::system_clock;
using ms = std::chrono::milliseconds;

//To count time
auto afterCrosswalk_StartTimeCount = m_clock::now();
auto afterFirstCase_StartTimeCount = m_clock::now();
auto startMoviments_StartTimeCount = m_clock::now();


ros::Publisher dist_angle_pub;

#define CV_PI   3.1415926535897932384626433832795
std::mutex m;

enum stateEnum
{
    MOVING_NORMAL,
    CORRECTING_POSITION_TO_LEFT,
    CORRECTING_POSITION_TO_RIGHT,
    FIRST_CURVE_FINISH_CASE,
    PASSING_CROSSWALK,
};

//Robot state
stateEnum state;


enum parkStateEnum
{ 
  WAITING,
  REVERSING_TO_PARKING_SPACE_1,
  IN_POSITION_TO_PARKING_SPACE_1,
  REVERSING_TO_PARKING_SPACE_2,
  IN_POSITION_TO_PARKING_SPACE_2,
  FINISHING_PARKING,
  PARKED,
};
//Robot parking state
parkStateEnum parkingState;


//Bools that have priority over the states
bool canMove;
bool isParking;
bool ignoreAfterCrosswalkCameraError;



//Used for the cases
double caseVel;
double caseAngle;

//Previous values sent to conde_control node
double lastVelSent;
double lastAngleSent;

void finishParking();
void reverseFirstSpace();
void reverseSecondSpace();
void processSign(std::string panelSign);
void crosswalkCrossCase();
std::string getStringFromStateEnum(stateEnum e);
void firstCurveFinishCase();
bool isCrosswalkAhead(double leftDistance, double leftAngle, double rightDistance, double rightAngle);
bool isSpecialCase(double angleGap);
std::pair<double,double> robotLogic(double leftDistance, double leftAngle, double rightDistance, double rightAngle, double rightGreenPercentage);
std::pair<double,double> processCamerasImageVals(double leftDistance, double leftAngle, double rightDistance, double rightAngle);
std::pair<double,double> processParking(double rightGreenPercentage);


void crossWalkCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", msg->data);
}

//Callback from the conde_signalling_panel node
void semaphoreCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string panelSign = msg->data.c_str();
  
  processSign(panelSign);
}

//Callback from the conde_tracking node
void velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
  double leftDistance = array->data[0]; //Distance to line left
  double leftAngle = array->data[1];    //Left line angle
  double rightDistance = array->data[2]; //Distance to line right
  double rightAngle = array->data[3];    //Right line angle
  double rightGreenPercentage = array->data[4]; //Green percentage in right camera
  
  
  //Process what to do
  std::pair<double, double> velAnglePair = robotLogic(leftDistance, leftAngle, rightDistance, rightAngle, rightGreenPercentage);

  lastVelSent = velAnglePair.first;
  lastAngleSent = velAnglePair.second;

  /// Publish to conde_control_node
  std_msgs::Float64MultiArray array_out;
  array_out.data.clear();
  array_out.data.push_back(velAnglePair.first);
  array_out.data.push_back(velAnglePair.second);          
  ros::Rate loop_rate(200);
  dist_angle_pub.publish(array_out);
  ros::spinOnce();
  loop_rate.sleep();
}




//Main
int main(int argc, char **argv)
{
  state = MOVING_NORMAL;
  parkingState = WAITING;
  canMove = false;
  isParking = false;

  caseVel = caseAngle = -123456;
  lastVelSent = lastAngleSent = 0.0;

  ignoreAfterCrosswalkCameraError = false;

  std::cout << "\nStarting conde decision\n" << std::endl;
  ros::init(argc, argv, "conde_decision");

  ros::NodeHandle n;


  ros::Subscriber sub1 = n.subscribe("/crossWalk", 5, crossWalkCallback);
  ros::Subscriber sub2 = n.subscribe("/conde_dist_angle", 100, velocityCallback);
  ros::Subscriber sub3 = n.subscribe("/conde_signalling_panel_info", 100, semaphoreCallback);

  dist_angle_pub = n.advertise<std_msgs::Float64MultiArray>("/conde_msg", 1);

  ros::spin();

  return 0;
}





//Core of robot, process the data and gets the final velocity ad angle to send to control
//returns pair: first = velocity, second = angle
std::pair<double,double> robotLogic(double leftDistance, double leftAngle, double rightDistance, double rightAngle, double rightGreenPercentage)
{
  std::pair <double,double> velAnglePair(0.0, 0.0);

  #pragma region PRIORITY_STATES
  if(!canMove)
    return velAnglePair;
  
  if(isParking)
    return processParking(rightGreenPercentage);


  if(ignoreAfterCrosswalkCameraError)
  {
      auto timePassedAfterCrosswalk = std::chrono::duration_cast<ms>(m_clock::now() - afterCrosswalk_StartTimeCount);
      if(timePassedAfterCrosswalk.count() > 20000)
        ignoreAfterCrosswalkCameraError = false;
  }
  #pragma endregion PRIORITY_STATES


  switch(state)
  {
    case FIRST_CURVE_FINISH_CASE:
    case PASSING_CROSSWALK:
      velAnglePair.first = caseVel;
      velAnglePair.second = caseAngle;
      break;
    case CORRECTING_POSITION_TO_LEFT:  
    case CORRECTING_POSITION_TO_RIGHT:
    case MOVING_NORMAL:
      velAnglePair = processCamerasImageVals(leftDistance, leftAngle, rightDistance, rightAngle);
      break;
    default:
      std::cout << "\033[1;31m" << "\n\nERROR WITH STATE\n\n" << "\033[0m" << std::endl;
      break;
  }

  #pragma region PRINT_ACTIVITY
  if(state != PASSING_CROSSWALK && state != FIRST_CURVE_FINISH_CASE)
  {
    double distanceGap = leftDistance - rightDistance;
    double angleGap = leftAngle - (CV_PI-rightAngle);

    std::cout << "[Reicived from Tracking]"
              << "\nDistance to left line: " << leftDistance << " - Left line Angle: " << leftAngle
              << "\nDistance to right line: " << rightDistance << " - Right line Angle: " << rightAngle
              << std::endl;
    
    if(angleGap < -1.5)
      std::cout << "[Gaps]" << "\nDistance gap: " <<  distanceGap << " - Angle gap: " << "\033[1;31m" << angleGap << "\033[0m" << std::endl;
    else if(angleGap > 1.5)
      std::cout << "[Gaps]" << "\nDistance gap: " <<  distanceGap << " - Angle gap: " << "\033[1;33m" << angleGap << "\033[0m" << std::endl;
    else
      std::cout << "[Gaps]" << "\nDistance gap: " <<  distanceGap << " - Angle gap: " <<  angleGap << std::endl;

    std::cout << "[Last Values Sent] vel: " << lastVelSent << " - angle: " << lastAngleSent << std::endl;

    std::cout << "[Sending Now] vel: " << velAnglePair.first << " - angle: " << velAnglePair.second << std::endl;
              
    std::cout << "[STATE] " << getStringFromStateEnum(state) << std::endl;

    if(ignoreAfterCrosswalkCameraError)
      std::cout << "\033[1;33m" << "--- IGNORING CAMERA ERROR VALUES ---" << "\033[0m" << std::endl << std::endl;  //YELLOW
    else std::cout << std::endl;
  }
  #pragma endregion PRINT_ACTIVITY

  return velAnglePair;
}



//Core of normal behaviours, process the data and detects changes to be made to the robot state
//returns pair: first = velocity, second = angle
std::pair<double,double> processCamerasImageVals(double leftDistance, double leftAngle, double rightDistance, double rightAngle)
{
  std::pair <double,double> velAnglePair(0.0, 0.0);

  double distanceGap = leftDistance - rightDistance;
  double angleGap = leftAngle - (CV_PI-rightAngle);

  #pragma region SPECIAL_CASES
  if(isCrosswalkAhead(leftDistance, leftAngle, rightDistance, rightAngle))
  {
    std::thread t1(crosswalkCrossCase);
    t1.detach();
    velAnglePair.first = 0.3;
    velAnglePair.second = 0.0;
    return velAnglePair;
  }
  else if(isSpecialCase(angleGap))
  {
    
    if(ignoreAfterCrosswalkCameraError)
    {
      //trying to compensate the camera error
      velAnglePair.first = 0.1;
      velAnglePair.second = lastAngleSent;
      return velAnglePair;
    }
    else
    {
      std::thread t1(firstCurveFinishCase);
      t1.detach();
      velAnglePair.first = 0.3;
      velAnglePair.second = 0.0;
      return velAnglePair;
    }
  }
  #pragma endregion SPECIAL_CASES
  

  #pragma region NORMAL_BEHAVIOUR
  if(distanceGap < -25)
    state = CORRECTING_POSITION_TO_RIGHT;
  else if(distanceGap > 45)
    state = CORRECTING_POSITION_TO_LEFT;

  if (state == CORRECTING_POSITION_TO_RIGHT && distanceGap < -5)
  {
    velAnglePair.first = 0.3;
    velAnglePair.second = -0.11;
  }
  else if(state == CORRECTING_POSITION_TO_LEFT && distanceGap > 35)
  {
    velAnglePair.first = 0.3;
    velAnglePair.second = 0.11;
  }
  else
  {
    state = MOVING_NORMAL;
    velAnglePair.first = 0.3;
    velAnglePair.second = 2.83616 - rightAngle;;
  }
  #pragma endregion NORMAL_BEHAVIOUR

  return velAnglePair;
}


//Receives sign information from the third camera, and processes the data and changes the priority booleans
void processSign(std::string panelSign)
{
  if(isParking) //if is parking, ignore this
    return;

  std::cout << "\033[1;35m" << "--- Detected Sign: " << "\033[0m";  //YELLOW
  if(panelSign == "ParkSignal")
  {
    isParking = true;
    canMove = true;
    std::cout << "\033[1;33m" << panelSign << "\033[0m" << std::endl;  //YELLOW
  }
  else if(panelSign == "StopSignal")
  {
    isParking = false;
    canMove = false;
    std::cout << "\033[1;31m" << panelSign << "\033[0m" << std::endl;  //RED
  }
  else if(panelSign == "MoveSignal")
  {
      canMove = true;
      isParking = false;
      std::cout << "\033[1;32m" << panelSign << "\033[0m" << std::endl;
  }
  else
  {
    std::cout << "\033[1;31m" << "BAD SIGN: " << panelSign << "\033[0m" << std::endl; //
  }
}


std::pair<double,double> processParking(double rightGreenPercentage)
{
  std::pair <double,double> velAnglePair(0.0, 0.0);

  switch(parkingState)
  {
    case WAITING:
    {
      std::thread t1(reverseFirstSpace);
      t1.detach();
      break;
    }
    case IN_POSITION_TO_PARKING_SPACE_1:
    {
      if(rightGreenPercentage > 0.35)
      {
        std::cout << "\033[1;32m" << "\nPark Space is Occupied" << "\033[0m" << std::endl;
        std::thread t1(reverseSecondSpace);
        t1.detach();
      }
      else
      {
        std::cout << "\033[1;32m" << "\nPark space is Clear" << "\033[0m" << std::endl;
        std::thread t1(finishParking);
        t1.detach();
      }
      break;
    }
    case IN_POSITION_TO_PARKING_SPACE_2:
      if(rightGreenPercentage > 0.48)
      {
        std::cout << "\033[1;32m" << "\nPark Space is Occupied" << "\033[0m" << std::endl;
        std::cout << "\033[1;31m" << "\nNo Parking Spaces Available" << "\033[0m" << std::endl;
        parkingState = PARKED;
      }
      else
      {
        std::cout << "\033[1;32m" << "\nPark space is Clear" << "\033[0m" << std::endl;
        std::thread t1(finishParking);
        t1.detach();
      }
      break;
    case REVERSING_TO_PARKING_SPACE_1:
    case REVERSING_TO_PARKING_SPACE_2:
    case FINISHING_PARKING:
      velAnglePair.first = caseVel;
      velAnglePair.second = caseAngle;
      break;
    case PARKED:
      isParking = false;
      canMove = false;
      break;
    default:
      std::cout << "\033[1;31m" << "\n\nERROR WITH PARKING STATE\n\n" << "\033[0m" << std::endl;
      break;
  }

  return velAnglePair;
}


//if crosswalk is ahead, returns true, else returns false
bool isCrosswalkAhead(double leftDistance, double leftAngle, double rightDistance, double rightAngle)
{
  if(leftDistance < -2000 && leftAngle > 1.4 && leftAngle < 1.6)
    return true;
  else return false;
}

//if is case where camera starts to output wild values (the case of the end of the first curve)
//    returns true, else returns false
bool isSpecialCase(double angleGap)
{
  if(angleGap < -1.5)
    return true;
  else return false;
}



//Handles the moviments after the first curve, ignoring wild camera values and locking others behaviours in the process
void firstCurveFinishCase()
{
  m.lock();
  state = FIRST_CURVE_FINISH_CASE;
  std::cout << "\033[1;32m" << "\nSTARTING SPECIAL CASE" << "\033[0m" << std::endl;
  caseVel = 0.3;
  caseAngle = 0.0;
  m.unlock();
  ros::Duration(2.0).sleep();
  //Quando a func. chega aqui -> ajeitou-se um pouco para a frente

  m.lock();
  std::cout << "\033[1;32m" << "\nSPECIAL CASE ROTATING" << "\033[0m" << std::endl;
  caseVel = 0.3;
  caseAngle = -0.22;
  m.unlock();
  ros::Duration(5.0).sleep();
  //Quando a func. chega aqui -> rodou para a direita e esta na passadeira
  
  m.lock();
  std::cout << "\033[1;32m" << "\nSPECIAL CASE MOVING FORWARD" << "\033[0m" << std::endl;
  caseVel = 0.3;
  caseAngle = 0.0;
  m.unlock();
  ros::Duration(5.5).sleep();
  //Quando a func. chega aqui -> avancou em linha reta e esta pronto para curvar

  m.lock();
  caseVel = 0.3;
  caseAngle = -0.21;
  std::cout << "\033[1;32m" << "\nSPECIAL CASE ROTATING AGAIN" << "\033[0m" << std::endl;
  m.unlock();
  ros::Duration(4.5).sleep();
  //Quando a func. chega aqui -> curvou e esta na entrada para a proxima curva

  m.lock();
  caseVel = 0.3;
  caseAngle = 0.0;
  std::cout << "\033[1;32m" << "\nSPECIAL CASE FORWARD A LITTLE" << "\033[0m" << std::endl;
  m.unlock();
  ros::Duration(2.0).sleep();
  //Quando a func. chega aqui -> ajeitou-se um pouco para a frente

  m.lock();
  //state = STOPPED;
  state = MOVING_NORMAL;
  std::cout << "\033[1;32m" << "\n---------- SPECIAL CASE FINISHED ----------" << "\033[0m" << std::endl;
  m.unlock();
  //Quando a func. chega aqui -> mudou o estado para STOPPED
}

//Handles the moviments to pass the crosswalk, ignoring wild camera values and locking others behaviours in the process
void crosswalkCrossCase()
{
  m.lock();
  state = PASSING_CROSSWALK;
  std::cout << "\033[1;32m" << "\nSTARTING PASSING CROSSWALK" << "\033[0m" << std::endl;
  caseVel = 0.3;
  caseAngle = 0.0;
  m.unlock();
  ros::Duration(4).sleep();
  //Quando a func. chega aqui -> ja passou pela passadeira

  m.lock();
  //state = STOPPED;
  state = MOVING_NORMAL;
  std::cout << "\033[1;32m" << "\n---------- PASSED BY THE CROSSWALK ! ----------" << "\033[0m" << std::endl;
  afterCrosswalk_StartTimeCount = m_clock::now();
  ignoreAfterCrosswalkCameraError = true;
  m.unlock();
}


//Reverse to first space
void reverseFirstSpace()
{
  m.lock();
  state = MOVING_NORMAL;
  parkingState = REVERSING_TO_PARKING_SPACE_1;
  std::cout << "\033[1;33m" << "\nReversing to the first parking space..." << "\033[0m" << std::endl;
  caseVel = -0.2;
  caseAngle = 0.0;
  m.unlock();
  ros::Duration(4.5).sleep();

  parkingState = IN_POSITION_TO_PARKING_SPACE_1;
}

//Reverse to second space
void reverseSecondSpace()
{
  m.lock();
  state = MOVING_NORMAL;
  parkingState = REVERSING_TO_PARKING_SPACE_2;
  std::cout << "\033[1;33m" << "\nReversing to the second parking space..." << "\033[0m" << std::endl;
  caseVel = -0.2;
  caseAngle = 0.01;
  m.unlock();
  ros::Duration(3.5).sleep();
  
  parkingState = IN_POSITION_TO_PARKING_SPACE_2;
}

//Rotates and parks the robot
void finishParking()
{
  m.lock();
  state = MOVING_NORMAL;
  parkingState = FINISHING_PARKING;
  std::cout << "\033[1;33m" << "\nAligning to park..." << "\033[0m" << std::endl;
  caseVel = 0.0;
  caseAngle = -0.2;
  m.unlock();
  ros::Duration(5.5).sleep();

  m.lock();
  state = MOVING_NORMAL;
  parkingState = FINISHING_PARKING;
  std::cout << "\033[1;33m" << "\nParking..." << "\033[0m" << std::endl;
  caseVel = 0.3;
  caseAngle = 0.0;
  m.unlock();
  ros::Duration(3.5).sleep();
  
  std::cout << "\033[1;32m" << "\nFinished parking !" << "\033[0m" << std::endl;
  parkingState = PARKED;
}


//Gets string representation of the state
std::string getStringFromStateEnum(stateEnum e)
{
  switch(e)
  {
    case MOVING_NORMAL: return "MOVING_NORMAL"; //white
    case CORRECTING_POSITION_TO_LEFT: return "\033[1;34mCORRECTING_POSITION_TO_LEFT\033[0m"; //blue
    case CORRECTING_POSITION_TO_RIGHT: return "\033[1;36mCORRECTING_POSITION_TO_RIGHT\033[0m"; //cyan
    case FIRST_CURVE_FINISH_CASE: return "\033[1;35mFIRST_CURVE_FINISH_CASE\033[0m"; //magenta
    case PASSING_CROSSWALK: return "\033[1;32mPASSING_CROSSWALK\033[0m"; //green
    default: return "\033[1;31mERROR WITH STATE\033[0m"; //red
  }
}