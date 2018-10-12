/**
 * This script moves the pickup camera to the desired position in cartesian space.
 */

#include "MovePickup.h"

MovePickup::MovePickup()
{
	init();
}

MovePickup::~MovePickup()
{

}

void MovePickup::callbackPSM(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// storing only pose not timestamp details
  	ToolTipPos_msg = msg->pose;
}

void MovePickup::callbackInputPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    newTopicInput = true;
    TopicInputPose = msg->pose;
}

void MovePickup::callbackJaw(const sensor_msgs::JointState::ConstPtr& msg)
{
    JawPosition = msg->position[0];
    // ROS_INFO("%f", JawPosition);
}


void MovePickup::init()
{
    ROS_INFO("Init");

  	// subscribers
 	tooltip_sub = n.subscribe("/dvrk/PSM1/position_cartesian_current", 1000, &MovePickup::callbackPSM, this);
    jaw_sub = n.subscribe("/dvrk/PSM1/state_jaw_current", 1000, &MovePickup::callbackJaw, this);
    input_sub = n.subscribe("/pickup/set_input", 1000, &MovePickup::callbackInputPose, this);

 	// publishers
 	goal_pub = n.advertise<geometry_msgs::Pose>("/dvrk/PSM1/set_position_goal_cartesian", 1000);
    jaw_pub = n.advertise<sensor_msgs::JointState>("/dvrk/PSM1/set_position_goal_jaw", 1000);
    input_pub = n.advertise<geometry_msgs::PoseStamped>("/pickup/set_input", 1000);

    // initialize flags
    newTopicInput = false;
    newSensorInput = false;
    jawSet = true;

 	// camera frame, H_ToolTip_Cam
    geometry_msgs::Pose temp1;
    temp1.position.x = 0.035;
    temp1.position.y = 0.0;
    temp1.position.z = 0.0;
    temp1.orientation.w = 1; //0.9659; 
    temp1.orientation.x = 0; //-0.2588; 
    temp1.orientation.y = 0.0;
    temp1.orientation.z = 0.0;
    tf::poseMsgToTF(temp1, H_ToolTip_Cam);

    H_Cam_ToolTip = H_ToolTip_Cam.inverse();
}

#if 0
void MovePickup::SetInput()
{
    InputRotation = tf::createQuaternionFromRPY(0, 0, 0.17); // in radians
    InputRotation.normalize();
    // geometry_msgs::Pose temp;
    // quaternionTFToMsg(InputRotation, temp.orientation);
    // ROS_INFO("SetInput: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);
}
#endif


void MovePickup::MoveBy(geometry_msgs::Pose InputPose)
{

	// update tooltip position
    tf::poseMsgToTF(ToolTipPos_msg, ToolTipPos);
    ROS_INFO("Tooltip: Position: [%f %f %f] Orientation: [%f %f %f %f]", ToolTipPos_msg.position.x, ToolTipPos_msg.position.y, ToolTipPos_msg.position.z, ToolTipPos_msg.orientation.w, ToolTipPos_msg.orientation.x, ToolTipPos_msg.orientation.y, ToolTipPos_msg.orientation.z);
	
    // copying 
    NewToolTipPos = tf::Transform(ToolTipPos);

    // for rotation----
    // extract quaternion component
    tf::Quaternion q_old;
    q_old = ToolTipPos.getRotation();
    // quaternionTFToMsg(q_old, temp.orientation);
    // ROS_INFO("Input: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);

    // multiply quaternions
    tf::Quaternion q_new;
    tf::Transform temp;
    tf::poseMsgToTF(InputPose, temp);
    q_new = temp.getRotation() * q_old;
	q_new.normalize();    
    // geometry_msgs::Pose temp;
    // quaternionTFToMsg(q_new, temp.orientation);
    // ROS_INFO("new: Position: [%f %f %f] Orientation: [%f %f %f %f]", temp.position.x, temp.position.y, temp.position.z, temp.orientation.w, temp.orientation.x, temp.orientation.y, temp.orientation.z);

    // setting new position
    NewToolTipPos.setRotation(q_new);
    tf::poseTFToMsg(NewToolTipPos, NewToolTipPos_msg);
    
    // for translation----
    NewToolTipPos_msg.position.x = ToolTipPos_msg.position.x + InputPose.position.x;
    NewToolTipPos_msg.position.y = ToolTipPos_msg.position.y + InputPose.position.y;
    NewToolTipPos_msg.position.z = ToolTipPos_msg.position.z + InputPose.position.z;

    // move to new position
    // goal_pub.publish(NewToolTipPos_msg);
    ROS_INFO("New position updated");
    ROS_INFO("NewToolTipPos: Position: [%f %f %f] Orientation: [%f %f %f %f]", NewToolTipPos_msg.position.x, NewToolTipPos_msg.position.y, NewToolTipPos_msg.position.z, NewToolTipPos_msg.orientation.w, NewToolTipPos_msg.orientation.x, NewToolTipPos_msg.orientation.y, NewToolTipPos_msg.orientation.z);

}

void MovePickup::error(const char *msg)
{
    perror(msg);
    exit(1);
}

void MovePickup::run()
{
    // initialize socket variables
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    char data[28];
    struct sockaddr_in serv_addr, cli_addr;
    int n;

    sensor_pose newpose;

    // initializing socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) 
        error("ERROR opening socket");

    bzero((char *) &serv_addr, sizeof(serv_addr));
    
    // set server details
    portno = 12345;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    
    // bind socket to server
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        error("ERROR on binding");


#if SOCKET_ENABLED
    // listen for clients
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

    if (newsockfd < 0) 
        error("ERROR on accept");

    ROS_INFO("Client connected");
#endif


	ros::Rate loop_rate(10);

	while(ros::ok())
	{
        ROS_INFO("Running");

        // poll callbacks
        ros::spinOnce();

        if(!jawSet)
        {
            jawSet = true;
            sensor_msgs::JointState JawState;
            ROS_INFO("Enter jaw");
            // JawState.name[0] = '';
            JawState.position[0] = JawPosition;
            JawState.velocity[0] = 0;
            JawState.effort[0] = 0;
            jaw_pub.publish(JawState);
        }

#if SOCKET_ENABLED
        // read data from client
        n = read(newsockfd,data,28);

        if (n < 0) 
            error("ERROR reading from socket");
        else
        {
            newSensorInput = true;
            sensor_pose * newpose = (sensor_pose*) data;
            SensorInputPose.position.x = newpose->xpos;
            SensorInputPose.position.y = newpose->ypos;
            SensorInputPose.position.z = newpose->zpos;

            SensorInputPose.orientation.w = newpose->qw;
            SensorInputPose.orientation.x = newpose->qx;
            SensorInputPose.orientation.y = newpose->qy;
            SensorInputPose.orientation.z = newpose->qz;

            ROS_INFO("Position = [%f %f %f] Orientation = [%f %f %f %f]", SensorInputPose.position.x, SensorInputPose.position.y, SensorInputPose.position.z, SensorInputPose.orientation.w, SensorInputPose.orientation.x, SensorInputPose.orientation.y, SensorInputPose.orientation.z);
            // break;
        }
#endif

        if(newSensorInput)
        {
            ROS_INFO("Input recieved from sensor");
            newSensorInput = false;

            MoveBy(SensorInputPose);
        }

        if(newTopicInput)
        {
            ROS_INFO("Input recieved from ROS topic");
            newTopicInput = false;

            MoveBy(TopicInputPose);
        }

		loop_rate.sleep();
	}

    // terminate socket
    close(sockfd);

}

