#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <visitgpsclient/VisitPositionsRobotGPSAction.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandHome.h>
#include <std_srvs/Trigger.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <offset_gps_mapping_kang/FindGPSPath.h>

#include <offset_control_kang/offset_odometry_robot.h>
#include <offset_control_kang/offset_mavros_robot.h>
//#include <offset_control/offset_c2_commands.h>
#include <offset_control_kang/offset_ground_robot_position_control.h>
#include <offset_control_kang/offset_mavros_battery_level_detector.h>

#define DEGREES_TO_RADIANS(a) (a * (M_PI / 180.0))

double starting_point_latitude = 0;
double starting_point_longitude = 0;

bool global_initial_pose_set_enable = false;
bool global_initial_pose_set = false;

class VisitPositionsRobotGPSAction
{
public:

  VisitPositionsRobotGPSAction(std::string name) :
    as_(nodeHandle_, name, boost::bind(&VisitPositionsRobotGPSAction::executeCB, this, _1), false),
    action_name_(name)
  {
    // Grab name of robot using ROS_NAMESPACE
    //robotName ="";// ros::this_node::getNamespace();
    robotName = ros::this_node::getNamespace();

    if(robotName =="/")
    {
       robotName ="";
    }

    // Initialize variables from config file (.yaml)
    FORMATION_POSITION_ID_ROBOT = -1;
    FORMATION_PARAM_FILE_MAX_FRONT_SIDE_FORMATION_POSITION_ID = -1;
    FORMATION_PARAM_FILE_INTERVAL_METER_DEFAULT = 1.0;
    FORMATION_PARAM_FILE_OFFSET_ANGLE_RAD = 0.0;
    NO_TRAVELING_DISTANCE_METER_DEFAULT = 0.25;
    NO_TRAVELING_ROTATION_RADIAN_DEFAULT = 0.25;
    MIN_POINT_TURN_ANGLE_RAD_DEFAULT = M_PI/6.0;
    MAX_SCALAR_SPEED_DEFAULT = 0.2;
    MAX_ROTATIONAL_RATE_RAD_PER_SEC_DEFAULT = M_PI/6.0;
    STEER_POWER_DEFAULT = 0.5;
    CAUTIOUS_TIME_SEC_DEFAULT = 2.0;
    IS_DEBUG_VELOCITY_CONTROL_DEFAULT = true;//false;   /*Kangneoung add*/

    robotInterdistanceSpacing = FORMATION_PARAM_FILE_INTERVAL_METER_DEFAULT;
    noTravelingDistanceMeter = NO_TRAVELING_DISTANCE_METER_DEFAULT;
    noTravelingRotationRadian = NO_TRAVELING_ROTATION_RADIAN_DEFAULT;
    minPointTurnAngleRad = MIN_POINT_TURN_ANGLE_RAD_DEFAULT;
    maxScalarSpeedMeterPerSec = MAX_SCALAR_SPEED_DEFAULT;
    maxRotationalRateRadPerSec = MAX_ROTATIONAL_RATE_RAD_PER_SEC_DEFAULT;
    steerPower = STEER_POWER_DEFAULT;
    cautiousTimeSec = CAUTIOUS_TIME_SEC_DEFAULT;
    isDebugVelocityControl = IS_DEBUG_VELOCITY_CONTROL_DEFAULT;

    // Update specified variables that are defined in the ROS param server
    nodeHandle_.getParam(robotName+"/OFFSET/LEADER_FOLLOWER_GROUND_ROBOT_SPACING_M", robotInterdistanceSpacing);
    nodeHandle_.getParam(robotName+"/OFFSET/NO_TRAVELING_DISTANCE_METER", noTravelingDistanceMeter);
    nodeHandle_.getParam(robotName+"/OFFSET/MIN_POINT_TURN_ANGLE_RAD", minPointTurnAngleRad);
    nodeHandle_.getParam(robotName+"/OFFSET/MAX_SCALAR_SPEED_METER_PER_SEC", maxScalarSpeedMeterPerSec);
    nodeHandle_.getParam(robotName+"/OFFSET/MAX_ROTATIONAL_RATE_RAD_PER_SEC", maxRotationalRateRadPerSec);
    nodeHandle_.getParam(robotName+"/OFFSET/STEER_POWER", steerPower);
    nodeHandle_.getParam(robotName+"/OFFSET/CAUTIOUS_TIME_SEC", cautiousTimeSec);
    nodeHandle_.getParam(robotName+"/OFFSET/IS_DEBUG_VELOCITY_CONTROL", isDebugVelocityControl);
	
	ros::NodeHandle private_nh("~");
	
	private_nh.getParam("starting_point_latitude", starting_point_latitude);
	private_nh.getParam("starting_point_longitude", starting_point_longitude);
	
	ROS_INFO("manual starting point latitude : %lf  manual starting point longitude : %lf ",starting_point_latitude,starting_point_longitude);
	
	private_nh.getParam("global_initial_pose_set_enable", global_initial_pose_set_enable);
	
	global_initial_pose_set = false;
	
	ROS_INFO("global_initial_pose_set_enable : %d  global_initial_pose_set : %d ",global_initial_pose_set_enable,global_initial_pose_set);
	
	

    // Create subscriber to odometry (local/global)
    suffix = "/mavros/local_position/pose";   
    topicName = robotName + suffix;
    //robot_odom_local_sub = nodeHandle_.subscribe(topicName, 10, &OFFSET_IAI_SOURCE_CODE_KANG::OdometryRobot::poseLocalCallback, &odomListenerRobot);  /*delete for bbn 091521*/
        topicName ="odom";
        private_nh.getParam("odom_topic",topicName);
        //suffix = "/odom";     /*add for bbn 091521*/
        //suffix = "/odom_virtual";  /*add for bbn gps navigation 101821*/
	//topicName = robotName + suffix; /*add for bbn 091521*/
	robot_odom_local_sub = nodeHandle_.subscribe(topicName, 10, &OFFSET_IAI_SOURCE_CODE_KANG::OdometryRobot::odomLocalCallback, &odomListenerRobot); /*add for bbn 091521*/
    
    topicName ="mavros/global_position/global";
    private_nh.getParam("global_position_topic",topicName);
    //suffix = "/mavros/global_position/global";
    //topicName = robotName + suffix;
    robot_odom_global_sub = nodeHandle_.subscribe(topicName, 10, &OFFSET_IAI_SOURCE_CODE_KANG::OdometryRobot::odomGlobalCallback, &odomListenerRobot);

    // Create subscriber to mavros state commands
    suffix = "/mavros/state";
    topicName = robotName + suffix;
    robot_state_sub = nodeHandle_.subscribe(topicName, 10, &OFFSET_IAI_SOURCE_CODE_KANG::MavrosRobot::stateCallback, &mavrosListenerRobot);

    // Create subscriber to C2 command - return to home base
    //c2_command_return_home_base_sub = nodeHandle_.subscribe("/OFFSET/c2_commands/return_home_base", 10, &OFFSET_IAI_SOURCE_CODE_KANG::C2_Commands::returnHomeBaseMessageCallback, &c2CommandListener);

    // Create subscriber to mavros battery level - emergency return to home base
    mavrosBatteryLevelListener.passReferenceNodeHandle(&nodeHandle_);
    suffix = "/mavros/battery";
    topicName = robotName + suffix;
    mavros_battery_level_return_home_base_sub = nodeHandle_.subscribe(topicName, 10, &OFFSET_IAI_SOURCE_CODE_KANG::Mavros_Battery_Level_Detector::mavrosBatteryMessageCallback, &mavrosBatteryLevelListener);

    // Create service client for finding gps path
    suffix = "/find_gps_path";
    topicName = robotName + suffix;
    findGPSPathClient = nodeHandle_.serviceClient<offset_gps_mapping_kang::FindGPSPath>(topicName);

    // Start action server
    as_.start();
  }

  ~VisitPositionsRobotGPSAction(void)
  {
  }

  void executeCB(const visitgpsclient::VisitPositionsRobotGPSGoalConstPtr &goal)
  {
    // Create publisher to send position/velocity commands
    suffix = "/mavros/setpoint_position/local";
    topicName = robotName + suffix;
    ROS_INFO("%s, robot name",robotName.c_str());
    ros::Publisher robot_position_command_pub = nodeHandle_.advertise<geometry_msgs::PoseStamped>(topicName, 10);

    // Create a simple action client to move_base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient("move_base", true); // spin a thread by default

    // Create a publishers for cancelling move_base action
    suffix = "/move_base/cancel";
    topicName = robotName + suffix;
    ros::Publisher robot_cancel_move_base_pub_ = nodeHandle_.advertise<actionlib_msgs::GoalID>(topicName, 10);    

    // Create service client for arming, takeoff/land, and set mode for UGV
    std::string suffixArm = "/mavros/cmd/arming";
    std::string suffixTakeoff = "/mavros/cmd/takeoff";
    std::string suffixLand = "/mavros/cmd/land";
    std::string suffixSetHome = "/mavros/cmd/set_home";
    std::string suffixSetMode = "/mavros/set_mode";
    std::string suffixHomeReqUpdate = "/mavros/home_position/req_update";

    std::string topicNameArm = robotName + suffixArm;
    std::string topicNameTakeoff = robotName + suffixTakeoff;
    std::string topicNameLand = robotName + suffixLand;
    std::string topicNameSetHome = robotName + suffixSetHome;
    std::string topicNameSetMode = robotName + suffixSetMode;
    std::string topicNameHomeReqUpdate = robotName + suffixHomeReqUpdate;

    ros::ServiceClient robotArmingClient = nodeHandle_.serviceClient<mavros_msgs::CommandBool>(topicNameArm);
    ros::ServiceClient robotSetHomeClient = nodeHandle_.serviceClient<mavros_msgs::CommandHome>(topicNameSetHome);
    ros::ServiceClient robotSetModeClient = nodeHandle_.serviceClient<mavros_msgs::SetMode>(topicNameSetMode);
    ros::ServiceClient robotHomeReqUpdateClient = nodeHandle_.serviceClient<std_srvs::Trigger>(topicNameHomeReqUpdate);

    // Initialize variables
    ros::Rate loop_rate(1);
    ros::Rate loop_rate_2(100);  // Setpoint publishing rate MUST be faster than 2 Hz

    // Mean radius of the Earth.
    const int RADIUS_EARTH = 6371008;

    ros::Time lastRequestRobot;
    bool flagStartedTimer = false;

    bool actionServerSuccess = true;
    bool flagRobotConnected = false;
    bool flagRobotReady = false;
    bool flagRobotArrivedTargetPoint = false;
    bool flagRobotOrientedForHeading = false;
    bool flagRobotWaitedAtTargetPoint = false;
    bool flagRobotArrivedGoalPoint = false;

    bool flagC2RequestReturnHomeBase = false;
    bool flagBatteryEmergencyReturnHomeBase = false;

    int robotTargetPointNumber = 0;
    geometry_msgs::Point goalPointRobot;
    std::vector<geometry_msgs::Point> goalPointsRobot;

    sensor_msgs::NavSatFix goalPointRobotGlobal;
    std::vector<sensor_msgs::NavSatFix> goalPointsRobotGlobal;

    sensor_msgs::NavSatFix goalPointRobotGlobalLocalPose;
    std::vector<sensor_msgs::NavSatFix> goalPointsRobotGlobalLocalPose;

    float headingInputWaypointDegrees = 0.0;
    std::vector<float> headingInputsWaypointsDegrees;
    std::vector<float> headingInputsWaypointsRadians;

    geometry_msgs::Quaternion orientationHeadingInput;
    std::vector<geometry_msgs::Quaternion> orientationsHeadingInput;

    float waitTimeAfterPosition = 0.0;
    std::vector<float> waitTimesAfterPosition;

    geometry_msgs::PoseStamped currentRobotPose;
    sensor_msgs::NavSatFix currentRobotPoseGlobal;
    geometry_msgs::PoseStamped robotStartingPose;

    sensor_msgs::NavSatFix currentRobotGPSPose;

    geometry_msgs::PoseStamped positionCommandRobot;

    mavros_msgs::CommandBool robotArmCommand;
    mavros_msgs::CommandHome robotSetHomeCommand;
    mavros_msgs::SetMode robotSetModeCommand;
    std_srvs::Trigger robotHomeReqUpdateCommand;

    bool flagUseRoadNetwork = false;
    bool flagRobotArrivedRoadNetworkTargetPoint = false;
    bool flagRobotArrivedRoadNetworkGoalPoint = false;
    int roadNetworkTargetPointNumber = 0;
    offset_gps_mapping_kang::FindGPSPath findGPSPathCommand;
    geometry_msgs::Quaternion quatOrienationRobot;
    std::vector<sensor_msgs::NavSatFix> waypointsToApproachTargetArea;
    geometry_msgs::Point roadNetworkGoalPointRobot;
    std::vector<geometry_msgs::Point> roadNetworkGoalPointsRobot;
    sensor_msgs::NavSatFix robotWaypointGpsLocationLocalPose;
    std::vector<sensor_msgs::NavSatFix> robotWaypointGpsLocationsLocalPose;

    bool moveBaseCommandedOnce = false;
    move_base_msgs::MoveBaseGoal moveBaseGoal;
    actionlib_msgs::GoalID cancelMoveBaseAction;
    actionlib::SimpleClientGoalState moveBaseGoalState(actionlib::SimpleClientGoalState::PENDING);

    // Reset action server feedback
    feedback_.currentPositionNumber = 0;
    feedback_.totalNumberPositionsToVisit = 0;
    feedback_.currentPoseRobot.header.seq = 0;
    feedback_.currentPoseRobot.header.stamp = ros::Time::now();
    feedback_.currentPoseRobot.header.frame_id = "world";
    feedback_.currentPoseRobot.pose = geometry_msgs::Pose();


    // Share robot starting GPS position with others actions servers running on same robot by using ROS param server
    ros::Duration(5.0).sleep();
    nodeHandle_.setParam(robotName+"/ROBOT_STARTING_GPS_POSITION/LAT", odomListenerRobot.getRobotGlobalPosition().latitude);
    nodeHandle_.setParam(robotName+"/ROBOT_STARTING_GPS_POSITION/LONG", odomListenerRobot.getRobotGlobalPosition().longitude);

    // Update initial position of robots
    robotStartingPose.pose.position.x = odomListenerRobot.getRobotPose().pose.position.x;
    robotStartingPose.pose.position.y = odomListenerRobot.getRobotPose().pose.position.y;
    robotStartingPose.pose.position.z = odomListenerRobot.getRobotPose().pose.position.z;



    // Use positions provided by user for visiting GPS positions
    ROS_INFO("%s: Using GPS positions provided by user for approaching target area.", action_name_.c_str());

    if (goal->positions.size() > 0)
    {
      for(int i=0; i<goal->positions.size(); i++)
      {
        goalPointRobotGlobal.latitude = goal->positions[i].latitude;
        goalPointRobotGlobal.longitude = goal->positions[i].longitude;

        goalPointsRobotGlobal.push_back(goalPointRobotGlobal);
      }
    }
    else
    {
      actionServerSuccess = false;
      ROS_ERROR("No GPS positions were found.");
      ROS_ERROR("Shutting down.");
    }

    // Ensure heading input is in the correct form
    if ( (goal->heading_input_deg.size() > 0) && (goal->heading_input_deg.size() == goal->positions.size()) )
    {
      for (int i=0; i<goal->heading_input_deg.size(); i++)
      {
        headingInputWaypointDegrees = goal->heading_input_deg[i];

        headingInputsWaypointsDegrees.push_back(headingInputWaypointDegrees);
        headingInputsWaypointsRadians.push_back(headingInputWaypointDegrees * M_PI/180.0);
      }
    }
    else
    {
      if ( (goal->heading_input_deg.size() > 0) && (goal->heading_input_deg.size() != goal->positions.size()) )
      {
        ROS_ERROR("Heading input did not align with GPS positions, defaulting to 0 degree heading for each position.");
      }
      else
      {
        ROS_ERROR("No heading inputs were found, defaulting to 0 degree heading for each position.");
      }

      for (int i=0; i<goal->positions.size(); i++)
      {
        headingInputWaypointDegrees = 0.0;

        headingInputsWaypointsDegrees.push_back(headingInputWaypointDegrees);
        headingInputsWaypointsRadians.push_back(headingInputWaypointDegrees * M_PI/180.0);
      }
    }

    // Convert heading input (radians) to quaternion
    for (int i=0; i<headingInputsWaypointsRadians.size(); i++)
    {
      tf2::Quaternion q_heading;
      q_heading.setRPY(0.0, 0.0, headingInputsWaypointsRadians[i]);

      orientationHeadingInput = tf2::toMsg(q_heading);

      orientationsHeadingInput.push_back(orientationHeadingInput);
    }

    // Confirm times to stay at positions is in the correct form
    if ( (goal->times_to_stay_at_positions.size() > 0) && (goal->times_to_stay_at_positions.size() == goal->positions.size()) )
    {
      for (int i=0; i<goal->times_to_stay_at_positions.size(); i++)
      {
        waitTimeAfterPosition = goal->times_to_stay_at_positions[i];

        waitTimesAfterPosition.push_back(waitTimeAfterPosition);
      }
    }
    else
    {
      if ( (goal->times_to_stay_at_positions.size() > 0) && (goal->times_to_stay_at_positions.size() != goal->positions.size()) )
      {
        ROS_ERROR("Wait times did not align with GPS positions, defaulting to no wait time for each position.");
      }
      else
      {
        ROS_ERROR("No wait times were found, defaulting to no wait time for each position.");
      }

      for (int i=0; i<goal->positions.size(); i++)
      {
        waitTimeAfterPosition = 0.0;

        waitTimesAfterPosition.push_back(waitTimeAfterPosition);
      }
    }



    // Variables for haversine formula.
    double lat1;
    double lat2;
    double lon1;
    double lon2;
    double dLat;
    double dLon;
    //double halfChordSq;
    //double angDist;
    //double gcDist;
    //double theta;
    //double offsetX_old;
    //double offsetY_old;

    double offsetX;
    double offsetY;
    double lat_sign;
    double long_sign;

    // Obtain leader robot GPS pose
	if(global_initial_pose_set_enable == false)
	{
		currentRobotGPSPose = odomListenerRobot.getRobotGlobalPosition();
		ROS_INFO("GPS global starting point set up");
	}
	else  /*global_initial_pose_set_enable == true*/
	{
		if(global_initial_pose_set == false)
		{
			currentRobotGPSPose.latitude  = starting_point_latitude;
			currentRobotGPSPose.longitude = starting_point_longitude;
			
			global_initial_pose_set = true;
			
			 ROS_INFO("Manual global starting point set up");
		}
		else /* global_initial_pose_set == true */
		{
			currentRobotGPSPose = odomListenerRobot.getRobotGlobalPosition();
			ROS_INFO("GPS global starting point set up");
		}
	}

    // Convert lat/lon to local coordinates.
    for(int i = 0; i < goalPointsRobotGlobal.size(); i ++)
    {
      goalPointRobotGlobal = goalPointsRobotGlobal[i];

      // Convert latitude/longitude to radians.
      lat1 = DEGREES_TO_RADIANS(currentRobotGPSPose.latitude);
      lat2 = DEGREES_TO_RADIANS(goalPointRobotGlobal.latitude);
      lon1 = DEGREES_TO_RADIANS(currentRobotGPSPose.longitude);
      lon2 = DEGREES_TO_RADIANS(goalPointRobotGlobal.longitude);
      dLat = lat2 - lat1;
      dLon = lon2 - lon1;

      // Use haversine formula to find distance between points.
      //halfChordSq = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
      //angDist = 2 * atan2(sqrt(halfChordSq), sqrt(1 - halfChordSq));
      //gcDist = RADIUS_EARTH * angDist;

      // Calculate initial bearing. This may lead to error over many miles since heading naturally varies
      // along a great-circle path, but works very well for small distances like those used by OFFSET.
      //theta = atan2(sin(dLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon));
        
      // Find the offset in meters from starting position.
      //offsetX_old = sin(theta) * gcDist;  /* mod for bbn gps nav 101821*/
      //offsetY_old = cos(theta) * gcDist;  /* mod for bbn gps nav 101821*/

      if(dLat>=0)
      {
        lat_sign = 1;
      }
      else
      {
        lat_sign = -1;
      }

      if(dLon>=0)
      {
        long_sign = 1;
      }
      else
      {
        long_sign = -1;
      }
      //East : X, North : Y
      offsetX = long_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLon/2.0)*sin(dLon/2.0)*cos(lat1)*cos(lat1)));
      offsetY = lat_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLat/2.0)*sin(dLat/2.0)));

      // Add offset to initial position to get goal position.
      // For some reason the existing code uses X for latitude and Y for longitude - NED?
      goalPointRobotGlobalLocalPose.latitude = robotStartingPose.pose.position.x + offsetX;
      goalPointRobotGlobalLocalPose.longitude = robotStartingPose.pose.position.y + offsetY;

      goalPointsRobotGlobalLocalPose.push_back(goalPointRobotGlobalLocalPose);
    }

    ROS_INFO("%s: Robot '%s' visiting GPS positions:", action_name_.c_str(), robotName.c_str());
    for (int i=0; i<goalPointsRobotGlobal.size() ; i++)
    {
      ROS_INFO("  %d) [%.10f, %.10f] m", i+1, goalPointsRobotGlobal[i].latitude, goalPointsRobotGlobal[i].longitude);
      ROS_INFO("       --> Wait time: %.3f seconds.", waitTimesAfterPosition[i]);
    }

    // Transform coordinates from world frame to robot frame
    for (int i=0; i<goalPointsRobotGlobalLocalPose.size() ; i++)
    {
      goalPointRobot.x = goalPointsRobotGlobalLocalPose[i].latitude;
      goalPointRobot.y = goalPointsRobotGlobalLocalPose[i].longitude;

      goalPointsRobot.push_back(goalPointRobot);
    }



    // Create GroundRobotPositionControl class instances
    OFFSET_IAI_SOURCE_CODE_KANG::GroundRobotPositionControl robotPositionControl(FORMATION_POSITION_ID_ROBOT,
                                                                            FORMATION_PARAM_FILE_MAX_FRONT_SIDE_FORMATION_POSITION_ID,
                                                                            FORMATION_PARAM_FILE_OFFSET_ANGLE_RAD,
                                                                            robotInterdistanceSpacing,
                                                                            noTravelingDistanceMeter,
                                                                            minPointTurnAngleRad,
                                                                            maxScalarSpeedMeterPerSec,
                                                                            maxRotationalRateRadPerSec);


    // Set first goal point for robot
    if (goalPointsRobot.size() > 0)
    {
      roadNetworkTargetPointNumber = 0;
      roadNetworkGoalPointsRobot.clear();
      robotWaypointGpsLocationsLocalPose.clear();
      
      goalPointRobot = goalPointsRobot[robotTargetPointNumber];
      goalPointRobotGlobal = goalPointsRobotGlobal[robotTargetPointNumber];
      orientationHeadingInput = orientationsHeadingInput[robotTargetPointNumber];

      findGPSPathCommand.request.start_position = currentRobotGPSPose;//odomListenerRobot.getRobotGlobalPosition();
      findGPSPathCommand.request.end_position = goalPointRobotGlobal;
      if ( findGPSPathClient.call(findGPSPathCommand) && (findGPSPathCommand.response.gps_coordinates.size() > 0) )
      {
        ROS_INFO("GPS waypoints to safely traverse between GPS positions were found, size of %ld, path length of %f m.", findGPSPathCommand.response.gps_coordinates.size(), findGPSPathCommand.response.path_length);

        if ( findGPSPathCommand.response.gps_coordinates.size() > 2 )
        {
          ROS_INFO(" - Road network will be used to get to GPS position #%d.", robotTargetPointNumber+1);

          flagUseRoadNetwork = true;
          waypointsToApproachTargetArea = findGPSPathCommand.response.gps_coordinates;

          // Convert road network GPS positions to local coordinates
          for(int i = 0; i < waypointsToApproachTargetArea.size(); i ++)
          {
            // Convert latitude/longitude to radians.
            lat1 = DEGREES_TO_RADIANS(currentRobotGPSPose.latitude);
            lat2 = DEGREES_TO_RADIANS(waypointsToApproachTargetArea[i].latitude);
            lon1 = DEGREES_TO_RADIANS(currentRobotGPSPose.longitude);
            lon2 = DEGREES_TO_RADIANS(waypointsToApproachTargetArea[i].longitude);
            dLat = lat2 - lat1;
            dLon = lon2 - lon1;

            // Use haversine formula to find distance between points.
            //halfChordSq = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
            //angDist = 2 * atan2(sqrt(halfChordSq), sqrt(1 - halfChordSq));
            //gcDist = RADIUS_EARTH * angDist;

            // Calculate initial bearing. This may lead to error over many miles since heading naturally varies
            // along a great-circle path, but works very well for small distances like those used by OFFSET.
            //theta = atan2(sin(dLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon));

            // Find the offset in meters from starting position.
            //offsetX_old = sin(theta) * gcDist;  /* mod for bbn gps nav 101821 */
            //offsetY_old = cos(theta) * gcDist;  /* mod for bbn gps nav 101821 */

            if(dLat>=0)
            {
              lat_sign = 1;
            }
            else
            {
              lat_sign = -1;
            }

            if(dLon>=0)
            {
              long_sign = 1;
            }
            else
            {
              long_sign = -1;
            }

            //East : X, North : Y
            offsetX = long_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLon/2.0)*sin(dLon/2.0)*cos(lat1)*cos(lat1)));
            offsetY = lat_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLat/2.0)*sin(dLat/2.0)));

            // Add offset to initial position to get goal position.
            // For some reason the existing code uses X for latitude and Y for longitude - NED?
            robotWaypointGpsLocationLocalPose.latitude = robotStartingPose.pose.position.x + offsetX;
            robotWaypointGpsLocationLocalPose.longitude = robotStartingPose.pose.position.y + offsetY;

            robotWaypointGpsLocationsLocalPose.push_back(robotWaypointGpsLocationLocalPose);
          }

          // Transform coordinates from world frame to robot frame
          for (int i=0; i<robotWaypointGpsLocationsLocalPose.size() ; i++)
          {
            roadNetworkGoalPointRobot.x = robotWaypointGpsLocationsLocalPose[i].latitude;
            roadNetworkGoalPointRobot.y = robotWaypointGpsLocationsLocalPose[i].longitude;

            roadNetworkGoalPointsRobot.push_back(roadNetworkGoalPointRobot);
          }

          // Set first road network goal point for robot
          if (roadNetworkGoalPointsRobot.size() > 0)
          {
            roadNetworkGoalPointRobot = roadNetworkGoalPointsRobot[roadNetworkTargetPointNumber];

            // Get current robot pose.
            currentRobotPose = odomListenerRobot.getRobotPose();

            tf::Vector3 axis(0, 0, 1);
            float angleRad = atan2( (roadNetworkGoalPointRobot.y - currentRobotPose.pose.position.y), (roadNetworkGoalPointRobot.x - currentRobotPose.pose.position.x) );
            tf::Quaternion quatTF(axis, angleRad);

            tf::quaternionTFToMsg(quatTF, quatOrienationRobot);

            if ( quatOrienationRobot.x == 0 && quatOrienationRobot.y == 0 && quatOrienationRobot.z == 0 && quatOrienationRobot.w == 0)
            {
              quatOrienationRobot.w = 1.0;
            }

            ROS_INFO("Robot is moving to GPS position using road network.");
            ROS_INFO(" - Set new GPS position for robot: <%.10f, %.10f>.", waypointsToApproachTargetArea[roadNetworkTargetPointNumber].latitude, waypointsToApproachTargetArea[roadNetworkTargetPointNumber].longitude);
          }
          else
          {
            actionServerSuccess = false;
            ROS_ERROR("No goal points were found in ROS param server.");
            ROS_ERROR("Shutting down.");
          }
        }
        else
        {
          ROS_INFO(" - Robot is moving straight to GPS position #%d.", robotTargetPointNumber+1);

          flagUseRoadNetwork = false;
        }        
      }
      else
      {
        actionServerSuccess = false;
        ROS_INFO("GPS waypoints to safely traverse between GPS positions were not able to be found.");          
      }

      ROS_INFO("Robot is moving to goal point.");
      ROS_INFO(" - Set new GPS position for robot: <%.10f, %.10f>.", goalPointsRobotGlobal[robotTargetPointNumber].latitude, goalPointsRobotGlobal[robotTargetPointNumber].longitude);
    }
    else
    {
      actionServerSuccess = false;
      ROS_ERROR("No goal points were provided.");
      ROS_ERROR("Shutting down.");
    }



    // Wait for move_base action server to come up
    ROS_INFO("Robot '%s' waiting for the move_base action server to come up.", robotName.c_str());
    if ( !moveBaseActionClient.waitForServer(ros::Duration(20, 0)) )
    {
      ROS_ERROR("The move_base action server did not start.");
      ROS_ERROR("Aborting.");
      actionServerSuccess = false;
    }



    // Wait for FCU connection
    while ( ros::ok() && actionServerSuccess && !flagRobotConnected )
    {
      if ( mavrosListenerRobot.isConnected() )
      {
        ROS_INFO("Robot is connected.");
        flagRobotConnected = true;
      }
      else
      {
        flagRobotConnected =true;// false; /*kangneoung mod*/
	ROS_INFO("Robot is not connected with FCU");
      }

      ros::spinOnce();
      loop_rate.sleep();
    }



    // Control loop - arm, switch to GUIDED mode, allowing position commands to be sent
    lastRequestRobot = ros::Time::now();
    while (0/*ros::ok()*/ && actionServerSuccess && !flagRobotReady) /*kangneoung mod*/
    {
      // Obtain robot pose
      currentRobotPose = odomListenerRobot.getRobotPose();

      // Arm robot
      if ( !mavrosListenerRobot.isArmed() && (ros::Time::now() - lastRequestRobot > ros::Duration(1.0)) )
      {
        // Set home position
        robotSetHomeCommand.request.current_gps = false;
        robotSetHomeCommand.request.latitude = odomListenerRobot.getRobotGlobalPosition().latitude;
        robotSetHomeCommand.request.longitude = odomListenerRobot.getRobotGlobalPosition().longitude;
        robotSetHomeCommand.request.altitude = odomListenerRobot.getRobotGlobalPosition().altitude;
        if ( robotSetHomeClient.call(robotSetHomeCommand) && robotSetHomeCommand.response.success )
        {
          ROS_INFO("Robot set home position: <%f, %f, %f> lat/long/alt.", robotSetHomeCommand.request.latitude, robotSetHomeCommand.request.longitude, robotSetHomeCommand.request.altitude);
        }
        else
        {
          ROS_ERROR("Robot failed to set home position: <%f, %f, %f> lat/long/alt.", robotSetHomeCommand.request.latitude, robotSetHomeCommand.request.longitude, robotSetHomeCommand.request.altitude);
          continue;
        }

        // Request update for home position
        if ( robotHomeReqUpdateClient.call(robotHomeReqUpdateCommand) && robotHomeReqUpdateCommand.response.success )
        {
          ROS_INFO("Robot requested home position update.");
        }
        else
        {
          ROS_ERROR("Robot failed to request home position update.");
          continue;
        }

        // Arm robot
        robotArmCommand.request.value = true;
        if ( robotArmingClient.call(robotArmCommand) && robotArmCommand.response.success )
        {
          ROS_INFO("Robot armed.");
        }
        else
        {
          ROS_ERROR("Robot failed to be armed.");
          continue;
        }

        flagRobotReady = false;

        lastRequestRobot = ros::Time::now();
      }
      // Once armed, switch to GUIDED mode, allowing position commands to be sent
      else if ( mavrosListenerRobot.isArmed() && mavrosListenerRobot.getMode() != "GUIDED" )
      {
        // // Flight controller needs a stream of setpoint messages before the commander accepts guided mode
        // for (int i=0; i<5; i++)
        // {
        //   positionCommandRobot = currentRobotPose;
        //   robot_position_command_pub.publish(positionCommandRobot);

        //   ros::spinOnce();
        //   loop_rate_2.sleep();
        // }

        robotSetModeCommand.request.custom_mode = "GUIDED";
        if ( !robotSetModeClient.call(robotSetModeCommand) && robotSetModeCommand.response.mode_sent )
        {
          ROS_ERROR("Failed to call set mode client for robot.");
        }

        flagRobotReady = false;

        lastRequestRobot = ros::Time::now();
      }
      else if ( mavrosListenerRobot.isArmed() && mavrosListenerRobot.getMode() == "GUIDED" )
      {
        ROS_INFO("Guided mode enabled for robot.");
        flagRobotReady = true;
      }
      else
      {
        flagRobotReady = false;
      }
    }
    flagRobotReady = true; /*kangneoung mod*/
    ROS_INFO("Robot '%s' ready for visiting GPS positions.", robotName.c_str());



    // Control loop - visit GPS positions
    while ( ros::ok() && actionServerSuccess && !flagC2RequestReturnHomeBase && !flagBatteryEmergencyReturnHomeBase && !flagRobotArrivedGoalPoint )
    {
      // Check for move_base failure
      moveBaseGoalState = moveBaseActionClient.getState();
      if ( (moveBaseCommandedOnce) && (moveBaseGoalState.isDone()) && (moveBaseGoalState != actionlib::SimpleClientGoalState::SUCCEEDED) )
      {
        ROS_ERROR("%s: Robot '%s' observed that move_base stopped with error: '%s'.", action_name_.c_str(), robotName.c_str(), moveBaseGoalState.toString().c_str());
        
        actionServerSuccess = false;
        break;
      }

      // Check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_ERROR("%s: Preempted", action_name_.c_str());

        // Set the action state to preempted
        as_.setPreempted();

        // Exit
        actionServerSuccess = false;
        ROS_ERROR("%s: Pre-empted -> Shutting down.", action_name_.c_str());
      }

      // Check if C2 has requested for robots to return to home base
     // if ( c2CommandListener.getReturnHomeBase() )
    //  {
    //    flagC2RequestReturnHomeBase = true;
     //   ROS_INFO("Robot flagC2RequestReturnHomeBase was set.");    /*Kangneoung add*/
     //   continue;
     // }

      // Check if battery level dictates an emergency return to home base
      if ( mavrosBatteryLevelListener.getReturnHomeBase() )
      {
        flagBatteryEmergencyReturnHomeBase =false; // true; /*kangneoung mod*/
         //ROS_INFO("Robot flagBatteryEmergencyReturnHomeBase was set.");   /*Kangneoung add*/
       // continue; /*kangneoung mod*/
      }

      // Obtain robot pose
      currentRobotPose = odomListenerRobot.getRobotPose();
      currentRobotPoseGlobal = odomListenerRobot.getRobotGlobalPosition();

      // Ensure robot arrives at GPS position, orients to be facing heading input, and waits at GPS position before moving onto next GPS position
      if (1/* mavrosListenerRobot.getMode() == "GUIDED"*/ )/*kangneoung mod*/
      {
        // Follow road network to get to next GPS position (ignoring orientation and wait-time for each road network position)
        if ( flagUseRoadNetwork && !flagRobotArrivedRoadNetworkGoalPoint )
        {
          // Check if robot has already been commanded to move
          if ( moveBaseCommandedOnce )
          {
            // Check if robot has reached road network waypoint 
            if ( robotPositionControl.robotHasArrivedAtTargetPoint(currentRobotPose, roadNetworkGoalPointRobot, noTravelingDistanceMeter) )
            {
              if ( !flagRobotArrivedRoadNetworkTargetPoint )
              {
                ROS_INFO("   - Robot has arrived at road network GPS position #%d/%d.", roadNetworkTargetPointNumber+1, roadNetworkGoalPointsRobot.size());
                flagRobotArrivedRoadNetworkTargetPoint = true;
              }

              if ( (flagRobotArrivedRoadNetworkTargetPoint) && (roadNetworkTargetPointNumber < roadNetworkGoalPointsRobot.size()-1) )
              {
                roadNetworkTargetPointNumber++;

                roadNetworkGoalPointRobot = roadNetworkGoalPointsRobot[roadNetworkTargetPointNumber];

                tf::Vector3 axis(0, 0, 1);
                float angleRad = atan2( (roadNetworkGoalPointRobot.y - roadNetworkGoalPointsRobot[roadNetworkTargetPointNumber-1].y), (roadNetworkGoalPointRobot.x - roadNetworkGoalPointsRobot[roadNetworkTargetPointNumber-1].x) );
                tf::Quaternion quatTF(axis, angleRad);

                tf::quaternionTFToMsg(quatTF, quatOrienationRobot);

                ROS_INFO(" - Set new road network GPS position for robot: <%.10f, %.10f>.", waypointsToApproachTargetArea[roadNetworkTargetPointNumber].latitude, waypointsToApproachTargetArea[roadNetworkTargetPointNumber].longitude);

                moveBaseGoal.target_pose.header.frame_id = "map";
                moveBaseGoal.target_pose.header.stamp = ros::Time::now();
                moveBaseGoal.target_pose.pose.position.x = roadNetworkGoalPointRobot.x;
                moveBaseGoal.target_pose.pose.position.y = roadNetworkGoalPointRobot.y;
                moveBaseGoal.target_pose.pose.position.z = roadNetworkGoalPointRobot.z;
                moveBaseGoal.target_pose.pose.orientation = quatOrienationRobot;
                moveBaseActionClient.sendGoal(moveBaseGoal);
                moveBaseCommandedOnce = true;

                flagRobotArrivedRoadNetworkTargetPoint = false;
              }
              else if ( !flagRobotArrivedRoadNetworkGoalPoint )
              {
                ROS_INFO("Robot has arrived at GPS position using road network.");

                flagRobotArrivedRoadNetworkGoalPoint = true;
                flagUseRoadNetwork  = false;
              }
            }
            else
            {
              if (isDebugVelocityControl)
              {
                ROS_INFO("-----------------------------------------------");
                ROS_INFO("Target point: <%.2f, %.2f> m.", roadNetworkGoalPointRobot.x, roadNetworkGoalPointRobot.y);
                ROS_INFO("Current pose: <%.2f, %.2f> m.", currentRobotPose.pose.position.x, currentRobotPose.pose.position.y);
              }
            }
          }
          else if ( !(moveBaseCommandedOnce) || (moveBaseGoalState == actionlib::SimpleClientGoalState::PENDING) )
          {
            // Continue to send current goal point for robot so controller does not exit prematurely
            moveBaseGoal.target_pose.header.frame_id = "map";
            moveBaseGoal.target_pose.header.stamp = ros::Time::now();
            moveBaseGoal.target_pose.pose.position.x = roadNetworkGoalPointRobot.x;
            moveBaseGoal.target_pose.pose.position.y = roadNetworkGoalPointRobot.y;
            moveBaseGoal.target_pose.pose.position.z = roadNetworkGoalPointRobot.z;
            moveBaseGoal.target_pose.pose.orientation = quatOrienationRobot;
            moveBaseActionClient.sendGoal(moveBaseGoal);
            moveBaseCommandedOnce = true;

            flagRobotArrivedTargetPoint = false;
            flagRobotOrientedForHeading = false;
            flagRobotWaitedAtTargetPoint = false;
          }
        }
        else if ( !flagUseRoadNetwork )
        {
          // Check if robot has arrived at GPS position
          if ( moveBaseCommandedOnce && moveBaseGoalState == actionlib::SimpleClientGoalState::SUCCEEDED )
          {
            // Once arrived at GPS position, orient to be facing heading input
            flagRobotArrivedTargetPoint = true;
			
			ROS_INFO("   - Robot 'flagRobotArrivedTargetPoint is true.") ; /*Kangneoung add*/
			
            if (flagRobotArrivedTargetPoint && !flagRobotOrientedForHeading)
            {
              ROS_INFO("   - Robot '%s' has arrived at GPS position #%d and is oriented facing %.3f deg.", robotName.c_str(), robotTargetPointNumber+1, headingInputsWaypointsDegrees[robotTargetPointNumber]);

              flagRobotOrientedForHeading = true;

              // // Check if robot is correctly facing heading input
              // if ( robotPositionControl.robotHasAlignedWithHeadingInput(currentRobotPose, headingInputsWaypointsRadians[robotTargetPointNumber], noTravelingRotationRadian) )
              // {
              //   ROS_INFO("   - Robot '%s' has arrived at GPS position #%d and is oriented facing %.3f deg.", robotName.c_str(), robotTargetPointNumber+1, headingInputsWaypointsDegrees[robotTargetPointNumber]);

              //   flagRobotOrientedForHeading = true;
              // }
              // else
              // {
              //   // Continue to send current goal point for robot so controller does not exit prematurely
              //   moveBaseGoal.target_pose.header.frame_id = "map";
              //   moveBaseGoal.target_pose.header.stamp = ros::Time::now();
              //   moveBaseGoal.target_pose.pose.position.x = goalPointRobot.x;
              //   moveBaseGoal.target_pose.pose.position.y = goalPointRobot.y;
              //   moveBaseGoal.target_pose.pose.position.z = goalPointRobot.z;
              //   moveBaseGoal.target_pose.pose.orientation = orientationHeadingInput;
              //   moveBaseActionClient.sendGoal(moveBaseGoal);
              //   moveBaseCommandedOnce = true;

              //   // Compute target angle in radians
              //   double roll, pitch, yaw;
              //   tf::Quaternion quat;
              //   tf::quaternionMsgToTF(currentRobotPose.pose.orientation, quat);
              //   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

              //   ROS_INFO("Robot '%s' has not yet oriented itself for GPS position #%d.", robotName.c_str(), robotTargetPointNumber+1);
              //   ROS_INFO("  - Current heading: %.3f deg.", (yaw * 180.0/M_PI));
              //   ROS_INFO("  - Target heading:  %.3f deg.", headingInputsWaypointsDegrees[robotTargetPointNumber]);

              //   flagRobotOrientedForHeading = false;
              // }            
            }

            // Wait at GPS position before moving onto next position
            if (flagRobotArrivedTargetPoint && flagRobotOrientedForHeading && !flagRobotWaitedAtTargetPoint)
            {
              if (!flagStartedTimer)
              {
                ROS_INFO("   - Robot '%s' waiting %.3f seconds before moving on.", robotName.c_str(), waitTimesAfterPosition[robotTargetPointNumber]);

                lastRequestRobot = ros::Time::now();
                flagStartedTimer = true;
              }

              while ( (ros::Time::now() - lastRequestRobot) < ros::Duration(waitTimesAfterPosition[robotTargetPointNumber]) )
              {
                ros::spinOnce();
                loop_rate.sleep();
              }

              ROS_INFO("   - Robot '%s' has finished waiting %.3f seconds at GPS position #%d.", robotName.c_str(), waitTimesAfterPosition[robotTargetPointNumber], robotTargetPointNumber+1);

              flagStartedTimer = false;
              flagRobotWaitedAtTargetPoint = true;
            }

            // Check if robot has progressed through all GPS positions in provided list
            if ( flagRobotArrivedTargetPoint && flagRobotOrientedForHeading && flagRobotWaitedAtTargetPoint)
            {
              if ( robotTargetPointNumber < (goalPointsRobotGlobal.size()-1) )
              {
                robotTargetPointNumber++;

                roadNetworkTargetPointNumber = 0;
                roadNetworkGoalPointsRobot.clear();
                robotWaypointGpsLocationsLocalPose.clear();

                flagRobotArrivedRoadNetworkTargetPoint = false;
                flagRobotArrivedRoadNetworkGoalPoint = false;

                goalPointRobot = goalPointsRobot[robotTargetPointNumber];
                goalPointRobotGlobal = goalPointsRobotGlobal[robotTargetPointNumber];
                orientationHeadingInput = orientationsHeadingInput[robotTargetPointNumber];

                ROS_INFO(" - Set new GPS position for robot '%s': <%.10f, %.10f, %.10f>.", robotName.c_str(), goalPointRobotGlobal.latitude, goalPointRobotGlobal.longitude, goalPointRobotGlobal.altitude);

                // Check if road network is recommended to get to net GPS position
                findGPSPathCommand.request.start_position = odomListenerRobot.getRobotGlobalPosition();
                findGPSPathCommand.request.end_position = goalPointRobotGlobal;
                if ( findGPSPathClient.call(findGPSPathCommand) && (findGPSPathCommand.response.gps_coordinates.size() > 0) )
                {
                  ROS_INFO("GPS waypoints to safely traverse between GPS positions were found, size of %ld, path length of %f m.", findGPSPathCommand.response.gps_coordinates.size(), findGPSPathCommand.response.path_length);

                  if ( findGPSPathCommand.response.gps_coordinates.size() > 2 )
                  {
                    ROS_INFO(" - Road network will be used to get to GPS position #%d.", robotTargetPointNumber+1);

                    flagUseRoadNetwork = true;
                    waypointsToApproachTargetArea.clear();
                    waypointsToApproachTargetArea = findGPSPathCommand.response.gps_coordinates;

                    // Convert road network GPS positions to local coordinates
                    for(int i = 0; i < waypointsToApproachTargetArea.size(); i ++)
                    {
                      // Convert latitude/longitude to radians.
                      lat1 = DEGREES_TO_RADIANS(currentRobotGPSPose.latitude);
                      lat2 = DEGREES_TO_RADIANS(waypointsToApproachTargetArea[i].latitude);
                      lon1 = DEGREES_TO_RADIANS(currentRobotGPSPose.longitude);
                      lon2 = DEGREES_TO_RADIANS(waypointsToApproachTargetArea[i].longitude);
                      dLat = lat2 - lat1;
                      dLon = lon2 - lon1;

                      // Use haversine formula to find distance between points.
                      //halfChordSq = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
                      //angDist = 2 * atan2(sqrt(halfChordSq), sqrt(1 - halfChordSq));
                      //gcDist = RADIUS_EARTH * angDist;

                      // Calculate initial bearing. This may lead to error over many miles since heading naturally varies
                      // along a great-circle path, but works very well for small distances like those used by OFFSET.
                      //theta = atan2(sin(dLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon));

                      // Find the offset in meters from starting position.
                      //offsetX_old = sin(theta) * gcDist;
                      //offsetY_old = cos(theta) * gcDist;

                      if(dLat>=0)
                      {
                         lat_sign = 1;
                      }
                      else
                      {
                         lat_sign = -1;
                      }

                      if(dLon>=0)
                      {
                         long_sign = 1;
                      }
                      else
                      {
                         long_sign = -1;
                      }
                     
                      //East : X, North : Y
                      offsetX = long_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLon/2.0)*sin(dLon/2.0)*cos(lat1)*cos(lat1)));
                      offsetY = lat_sign*2*RADIUS_EARTH*asin(sqrt(sin(dLat/2.0)*sin(dLat/2.0)));

                      // Add offset to initial position to get goal position.
                      // For some reason the existing code uses X for latitude and Y for longitude - NED?
                      robotWaypointGpsLocationLocalPose.latitude = robotStartingPose.pose.position.x + offsetX;
                      robotWaypointGpsLocationLocalPose.longitude = robotStartingPose.pose.position.y + offsetY;

                      robotWaypointGpsLocationsLocalPose.push_back(robotWaypointGpsLocationLocalPose);
                    }

                    // Transform coordinates from world frame to robot frame
                    for (int i=0; i<robotWaypointGpsLocationsLocalPose.size() ; i++)
                    {
                      roadNetworkGoalPointRobot.x = robotWaypointGpsLocationsLocalPose[i].latitude;
                      roadNetworkGoalPointRobot.y = robotWaypointGpsLocationsLocalPose[i].longitude;

                      ROS_INFO("Adding point #%d to road network goal points.", i+1);

                      roadNetworkGoalPointsRobot.push_back(roadNetworkGoalPointRobot);
                    }

                    // Set first road network goal point for robot
                    if (roadNetworkGoalPointsRobot.size() > 0)
                    {
                      roadNetworkGoalPointRobot = roadNetworkGoalPointsRobot[roadNetworkTargetPointNumber];

                      // Get current robot pose.
                      currentRobotPose = odomListenerRobot.getRobotPose();

                      tf::Vector3 axis(0, 0, 1);
                      float angleRad = atan2( (roadNetworkGoalPointRobot.y - currentRobotPose.pose.position.y), (roadNetworkGoalPointRobot.x - currentRobotPose.pose.position.x) );
                      tf::Quaternion quatTF(axis, angleRad);

                      tf::quaternionTFToMsg(quatTF, quatOrienationRobot);

                      if ( quatOrienationRobot.x == 0 && quatOrienationRobot.y == 0 && quatOrienationRobot.z == 0 && quatOrienationRobot.w == 0)
                      {
                        quatOrienationRobot.w = 1.0;
                      }

                      ROS_INFO("Robot is moving to GPS position using road network.");
                      ROS_INFO(" - Set new GPS position for robot: <%.10f, %.10f>.", waypointsToApproachTargetArea[roadNetworkTargetPointNumber].latitude, waypointsToApproachTargetArea[roadNetworkTargetPointNumber].longitude);
                    }
                    else
                    {
                      actionServerSuccess = false;
                      ROS_ERROR("No goal points were found in ROS param server.");
                      ROS_ERROR("Shutting down.");
                    }

                    // Continue to send current goal point for robot so controller does not exit prematurely
                    moveBaseGoal.target_pose.header.frame_id = "map";
                    moveBaseGoal.target_pose.header.stamp = ros::Time::now();
                    moveBaseGoal.target_pose.pose.position.x = roadNetworkGoalPointRobot.x;
                    moveBaseGoal.target_pose.pose.position.y = roadNetworkGoalPointRobot.y;
                    moveBaseGoal.target_pose.pose.position.z = roadNetworkGoalPointRobot.z;
                    moveBaseGoal.target_pose.pose.orientation = orientationHeadingInput;
                    moveBaseActionClient.sendGoal(moveBaseGoal);
                    moveBaseCommandedOnce = true;
                  }
                  else
                  {
                    ROS_INFO(" - Robot is moving straight to GPS position #%d.", robotTargetPointNumber+1);

                    flagUseRoadNetwork = false;
                  }        
                }
                else
                {
                  actionServerSuccess = false;
                  ROS_INFO("GPS waypoints to safely traverse between GPS positions were not able to be found.");          
                }

                flagRobotArrivedTargetPoint = false;
                moveBaseCommandedOnce = false;
              }
              else
              {
                ROS_INFO("Robot '%s' has arrived at last GPS position.", robotName.c_str());
                flagRobotArrivedGoalPoint = true;
              }
            }
          }
          else if ( !(moveBaseCommandedOnce) || (moveBaseGoalState == actionlib::SimpleClientGoalState::PENDING) )
          {
            if ( isDebugVelocityControl )
            {
              ROS_INFO("Robot '%s' has not yet arrived at GPS position.", robotName.c_str());
              ROS_INFO("  - Current GPS position: <%f, %f>.", currentRobotPoseGlobal.latitude, currentRobotPoseGlobal.longitude);
              ROS_INFO("  - Target GPS position:  <%f, %f>.", goalPointRobotGlobal.latitude, goalPointRobotGlobal.longitude);
              ROS_INFO("  - Current pose: <%.2f, %.2f> m.", currentRobotPose.pose.position.x, currentRobotPose.pose.position.y);
              ROS_INFO("  - Target point: <%.2f, %.2f> m.", goalPointRobot.x, goalPointRobot.y);
            }

            // Continue to send current goal point for robot so controller does not exit prematurely
            moveBaseGoal.target_pose.header.frame_id = "map";
            moveBaseGoal.target_pose.header.stamp = ros::Time::now();
            moveBaseGoal.target_pose.pose.position.x = goalPointRobot.x;
            moveBaseGoal.target_pose.pose.position.y = goalPointRobot.y;
            moveBaseGoal.target_pose.pose.position.z = goalPointRobot.z;
            moveBaseGoal.target_pose.pose.orientation = orientationHeadingInput;
            moveBaseActionClient.sendGoal(moveBaseGoal);
            moveBaseCommandedOnce = true;

            flagRobotArrivedTargetPoint = false;
            flagRobotOrientedForHeading = false;
            flagRobotWaitedAtTargetPoint = false;
          }
        }
      }
      else
      {
        ROS_ERROR("Robot '%s' is not in 'GUIDED' mode: '%s'.", robotName.c_str(), mavrosListenerRobot.getMode().c_str());

        actionServerSuccess = false;
        flagRobotArrivedTargetPoint = false;

        // Cancel move_base goal
        robot_cancel_move_base_pub_.publish(cancelMoveBaseAction);
        break;
      }

      // Specify current pose of robot for feedback
      feedback_.currentPositionNumber = robotTargetPointNumber+1;
      feedback_.totalNumberPositionsToVisit = goalPointsRobot.size();

      feedback_.currentPoseRobot.header.seq = feedback_.currentPoseRobot.header.seq + 1;
      feedback_.currentPoseRobot.header.stamp = ros::Time::now();
      feedback_.currentPoseRobot.pose = currentRobotPose.pose;

      // Publish the feedback
      as_.publishFeedback(feedback_);

      // Continue running at specified frequency
      ros::spinOnce();
      loop_rate.sleep();
    }

    ROS_INFO("Robot '%s' has visited all GPS positions.", robotName.c_str());

    // Cancel move_base goal
    robot_cancel_move_base_pub_.publish(cancelMoveBaseAction);

    // Check if action has successed
    if (actionServerSuccess)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      result_.finalPoseRobot = feedback_.currentPoseRobot;
      result_.requestReturnHomeBase = flagC2RequestReturnHomeBase;
      result_.emergencyRequestReturnHomeBase = flagBatteryEmergencyReturnHomeBase;

      // Set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else if (flagC2RequestReturnHomeBase)
    {
      ROS_INFO("%s: C2 request to return to home base", action_name_.c_str());
      result_.finalPoseRobot = feedback_.currentPoseRobot;
      result_.requestReturnHomeBase = flagC2RequestReturnHomeBase;
      result_.emergencyRequestReturnHomeBase = flagBatteryEmergencyReturnHomeBase;

      // Set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else if (flagBatteryEmergencyReturnHomeBase)
    {
      ROS_INFO("%s: Emergency battery level, need to return to home base", action_name_.c_str());
      result_.finalPoseRobot = feedback_.currentPoseRobot;
      result_.requestReturnHomeBase = flagC2RequestReturnHomeBase;
      result_.emergencyRequestReturnHomeBase = flagBatteryEmergencyReturnHomeBase;

      // Set the action state to succeeded
      as_.setSucceeded(result_);
    }
    else
    {
      ROS_ERROR("%s: Aborted", action_name_.c_str());
      result_.finalPoseRobot = feedback_.currentPoseRobot;
      result_.requestReturnHomeBase = flagC2RequestReturnHomeBase;
      result_.emergencyRequestReturnHomeBase = flagBatteryEmergencyReturnHomeBase;

      // Set the action state to succeeded
      as_.setAborted(result_);
    }

    ROS_INFO("========================================================");
  }

protected:

  // Initialize node handle
  ros::NodeHandle nodeHandle_;

  // NodeHandle instance must be created before this line, otherwise strange error occurs
  actionlib::SimpleActionServer<visitgpsclient::VisitPositionsRobotGPSAction> as_;
  std::string action_name_;

  // Create messages that are used to published feedback/result
  visitgpsclient::VisitPositionsRobotGPSFeedback feedback_;
  visitgpsclient::VisitPositionsRobotGPSResult result_;

  // Grab name of robot using ROS_NAMESPACE
  std::string robotName;

  // Initialize variables for creating topicName
  std::string suffix;
  std::string topicName;

  // Initialize variables from config file (.yaml)
  int FORMATION_POSITION_ID_ROBOT;
  int FORMATION_PARAM_FILE_MAX_FRONT_SIDE_FORMATION_POSITION_ID;
  double FORMATION_PARAM_FILE_INTERVAL_METER_DEFAULT;
  double FORMATION_PARAM_FILE_OFFSET_ANGLE_RAD;
  double NO_TRAVELING_DISTANCE_METER_DEFAULT;
  double NO_TRAVELING_ROTATION_RADIAN_DEFAULT;
  double MIN_POINT_TURN_ANGLE_RAD_DEFAULT;
  double MAX_SCALAR_SPEED_DEFAULT;
  double MAX_ROTATIONAL_RATE_RAD_PER_SEC_DEFAULT;
  double STEER_POWER_DEFAULT;
  double CAUTIOUS_TIME_SEC_DEFAULT;
  double AVOID_D_GAIN;
  double AVOID_CRITICAL_DISTANCE;
  double AVOID_MIN_CRITICAL_DISTANCE;
  double AVOID_TURN_GAIN;
  double AVOIDANCE_GAIN;
  double AVOIDANCE_POWER;
  std::string LEADER_FOLLOWER_FORMATION_DEFAULT;
  std::string MAVROS_LOCAL_POSITION_TYPE_DEFAULT;
  bool USE_OBSTACLE_AVOIDANCE_DEFAULT;
  bool IS_DEBUG_VELOCITY_CONTROL_DEFAULT;

  double robotInterdistanceSpacing;
  double noTravelingDistanceMeter;
  double noTravelingRotationRadian;
  double minPointTurnAngleRad;
  double maxScalarSpeedMeterPerSec;
  double maxRotationalRateRadPerSec;
  double steerPower;
  double cautiousTimeSec;
  double avoidDGain;
  double avoidCriticalDistance;
  double avoidMinCriticalDistance;
  double avoidTurnGain;
  double avoidGain;
  double avoidPower;
  std::string leaderFollowerFormation;
  std::string mavrosLocalPositionType;
  bool useObstacleAvoidance;
  bool isDebugVelocityControl;

  // Create subscriber to odometry (local/global)
  OFFSET_IAI_SOURCE_CODE_KANG::OdometryRobot odomListenerRobot;
  OFFSET_IAI_SOURCE_CODE_KANG::MavrosRobot mavrosListenerRobot;
 // OFFSET_IAI_SOURCE_CODE_KANG::UvMapObstacleDetectorRobot uvMapObstacleDetectorLeader;
 // OFFSET_IAI_SOURCE_CODE_KANG::C2_Commands c2CommandListener;
  OFFSET_IAI_SOURCE_CODE_KANG::Mavros_Battery_Level_Detector mavrosBatteryLevelListener;

  ros::Subscriber robot_odom_local_sub;
  ros::Subscriber robot_odom_global_sub;
  ros::Subscriber robot_state_sub;
  //ros::Subscriber leader_uv_map_obstacle_detector_sub;
  //ros::Subscriber c2_command_return_home_base_sub;
  ros::Subscriber mavros_battery_level_return_home_base_sub;

  ros::ServiceClient findGPSPathClient;
};


int main(int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "groundVisitPositionsRobotGPS_server");

  // Start groundVisitPositionsRobotGPS action server
  VisitPositionsRobotGPSAction groundVisitPositionsRobotGPS("groundVisitPositionsRobotGPS");

  ROS_INFO("Started VisitPositionsRobotGPS action server for ground robots.");
  ROS_INFO("=============================================");

  // Spin to allow action requests
  ros::spin();

  return 0;
}
