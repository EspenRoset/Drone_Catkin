#include "../include/ControlLibrary/DroneController.h"

void DroneControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void DroneControl::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
   double quatx= msg->pose.pose.orientation.x;
   double quaty= msg->pose.pose.orientation.y;
   double quatz= msg->pose.pose.orientation.z;
   double quatw= msg->pose.pose.orientation.w;

    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    current_position = *msg;
}

void DroneControl::collision_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    Obstacle_detected = static_cast<int>(msg->data[2]);
    Roof_limit = static_cast<int>(msg->data[2]); 
    Floor_limit = static_cast<int>(msg->data[3]);
    AvoidReverse = msg->data[0];
    AvoidRoll = msg->data[1];
    //Obstacle_position = {msg->data[0], msg->data[1], msg->data[3], msg->data[4]}; // X, Y, to high, too low
}

void DroneControl::RunDrone(){
    while(true){
        switch (state)
        {
        case 0 /*StartUp*/: // Initialize, Change to OFFBOARD Mode, Arm Quad
                DroneControl::WaitForFCUConnection(); // Wait for FCU Connection
                // Wait for arm command
                ROS_INFO_STREAM("Waiting for start/arm command (Rb)");
                {
                std::unique_lock<std::mutex> lk(ArmMutex); // Block here until user is ready
                cv.wait(lk, [&]{return ArmDrone;});
                }

                ROS_INFO_STREAM("Drone starting up");
                DroneControl::SendNWaipoints(100); // Send somewaypoints to prepare for mode change
                if (current_state.mode != "OFFBOARD"){
                    DroneControl::SetPX4Mode("OFFBOARD"); // Change mode to offboard
                }
                //DroneControl::PX4Arm();
                ResetWaypoints();
                AddWaypoint();

                StartingHeight = current_position.pose.pose.position.z; //Reference height used for calcing takeoff height
                ROS_INFO_STREAM("Waiting for Takeoff command (Rb + A)");
                state = Takeoff;


            break;
        case 1 /*TakeOff*/: // Check OFFBOARD and arm, Take off to flight altitude
                if (InitiateTakeoff /*&& !Roof_limit*/){
                    DroneControl::PX4Arm();
                    DroneControl::DroneTakeoff(StartingHeight + TakeoffAltitude); // Takes of and changes state to flying
                }
            break;
        case 2 /*Flying*/: // Get input from controller and update Targetpotsiion
                TargetPosition = InputTargetPosition; // Input from controller
                TargetPosition.velocity.x = TargetPosition.velocity.x + AvoidReverse; // Obstacle avoidance
                TargetPosition.velocity.y = TargetPosition.velocity.y + AvoidRoll;      // Obstacle avoidance

                if (InitiateLanding){
                    state = Landing;
                } if (Roof_limit || Floor_limit){
                    state = AvoidObstacle;
                }
                if (InitiateReturn){
                    state = ReturnHome;
                    ROS_INFO_STREAM("Returning home");

                }
            break;
        case 3 /*Landing*/: // Land quad
                DroneControl::DroneLand();
            break;
        case 4 /*Landed*/: // Be ready to change mode to takeoff
                ROS_INFO_STREAM("Landed and waiting");
                ROS_INFO_STREAM("Press takeoff to fly");

                if (current_state.mode != "OFFBOARD"){
                    DroneControl::SetPX4Mode("OFFBOARD"); // Change mode to offboard
                }else {
                    StartingHeight = current_position.pose.pose.position.z;
                    ResetWaypoints();
                    AddWaypoint();
                    state = Takeoff;
                }
            break;
        case 5 /*Avoid Obstacle*/: // Limit movement to avoid crashing
                // Get input from Controller
                TargetPosition = InputTargetPosition;
                ROS_INFO_STREAM("Avoiding obstacle");
                // Limit Movement
                /*
                if (Obstacle_detected){ // Use velocities from anti_collision to avoid obstacle
                    TargetPosition.velocity.x = TargetPosition.velocity.x + AvoidReverse;
                    TargetPosition.velocity.y = TargetPosition.velocity.y + AvoidRoll;
        
                }*/
                if (Roof_limit){ // Roof too close, limit z velocity to negative
                    if (TargetPosition.velocity.z > 0){
                        TargetPosition.velocity.z = 0;
                    }
                }
                if (Floor_limit){ // Floor too close, limit z velocity to positive
                    if (TargetPosition.velocity.z > 0){
                        TargetPosition.velocity.z = 0;
                    }
                }
                if (!Roof_limit && !Floor_limit){
                    state = Flying;
                }
                if (InitiateReturn){
                    state = ReturnHome;
                    ROS_INFO_STREAM("Returning home");
                }
            break;
        case 6 /*ReturnHome*/: //Follow waypoints back home
            ChangeToLocalFrame();
            CalcYawRate(); // Keep drone pointing towards next point
            while (check_position(ReturnWaypoints.back(), 0.5 && ReturnWaypoints.size() > 1)){
                ROS_INFO_STREAM("Going to the next waypoint");
                ReturnWaypoints.pop_back();
            }
            TargetPosition.position.x = ReturnWaypoints.back()[0];
            TargetPosition.position.y = ReturnWaypoints.back()[1];
            TargetPosition.position.z = ReturnWaypoints.back()[2];
            TargetPosition.yaw_rate = 0;

            if (ReturnWaypoints.size() <= 1){
                InitiateReturn = false;
                ChangeToBodyFrame();
                state = Landing;
            }
            if (InitiateTakeoff){
                InitiateReturn = false;
                InitiateTakeoff = false;
                ChangeToBodyFrame();
                state = Flying;
            }

            break;
        default: // Maybe do something
            break;
        }
        
        if((state != ReturnHome) && !check_position(ReturnWaypoints.back(), 0.1)){
            AddWaypoint();
        }
        pos_pub.publish(TargetPosition);
        rate_->sleep();
    }
}

void DroneControl::WaitForFCUConnection(){ // Wait until connection with PX4 is established
    while (ros::ok() && !current_state.connected)
            {
                ros::spinOnce();
                rate_->sleep();
            }
    ROS_INFO_STREAM("PX4 Connected");

}

void DroneControl::SendNWaipoints(int n){ // Send n number of waypoints (used to be able to change mode to offboard)
     for (int i = n; ros::ok() && i > 0; --i)
  {
    pos_pub.publish(TargetPosition);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::SetPX4Mode(std::string Mode){ // set given mode on px4
    offb_set_mode.request.custom_mode = Mode;
        if (current_state.mode != Mode && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
}

void DroneControl::PX4Arm(){ // Arm quad and reset arm bool
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        ROS_INFO_STREAM("Trying to arm");
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    ArmDrone = false;
}

void DroneControl::DroneTakeoff(float altitude){ // Take off the drone and change mode to Flying
    if(ros::ok() && (current_position.pose.pose.position.z < altitude)){
        ROS_INFO_STREAM("Initiating Takeoff");
        TargetPosition.velocity.z = 0.5;
    }else {
        ROS_INFO_STREAM("Cruising altitude reached");
        TargetPosition.velocity.z = 0;
        InitiateTakeoff = false;
        state = Flying;
    }
}

void DroneControl::DroneLand(){ // Land the drone and change mode to Startup
    while (!(landing_client.call(land_cmd) &&
            land_cmd.response.success) ){
            ROS_INFO("trying to land");
            ros::spinOnce();
            rate_->sleep();
        }
    std::this_thread::sleep_for(std::chrono::seconds(5)); // Sleep to allow drone to fully land and disarm  
    if (current_position.pose.pose.position.z < 0.1){
        InitiateLanding = false;
        state = Landed;         
    }

}

void DroneControl::ReverseDrone(int reversemillisec){

}

bool DroneControl::check_position(std::vector<double> pos, double ErrorTolerance){
  double Errorx = pos[0] - current_position.pose.pose.position.x;
  double Errory = pos[1] - current_position.pose.pose.position.y;
  double Errorz = pos[2] - current_position.pose.pose.position.z;

  double magnitude = std::sqrt((Errorx * Errorx) + (Errory * Errory) + (Errorz * Errorz));
 
  if (magnitude < ErrorTolerance)
  {
    return true;
  }
  else
  {
    return false;
  }

}

void DroneControl::ResetWaypoints(){
    ROS_INFO("Before clear()");
    ReturnWaypoints.clear();
    ROS_INFO("After clear()");
}

void DroneControl::AddWaypoint(){
    ROS_INFO("Before try statement");
    try{
    ReturnWaypoints.push_back({
        current_position.pose.pose.position.x,
        current_position.pose.pose.position.y,
        current_position.pose.pose.position.z});
    } catch(std::exception &e){
        ROS_INFO_STREAM(e.what());
        ROS_INFO_STREAM(ReturnWaypoints.size());
    }
    ROS_INFO("After try statement");
}

float DroneControl::CalcYawRate(){
    TargetPosition.type_mask = 2048; // Ignoring yaw rate, using yaw angle

    float x2 = ReturnWaypoints.back()[0] - current_position.pose.pose.position.x; // Move objective point
    float y2 = ReturnWaypoints.back()[1] - current_position.pose.pose.position.y;
   

    float x1 = 1; //Reference Point
    float y1 = 0; 

    float dot = x1*x2 + y1*y2;      // dot product between [x1, y1] and [x2, y2]
    float det = x1*y2 - y1*x2;      // determinant
    float angle = std::atan2(det, dot); //+3.14159265359;  // atan2(y, x) or atan2(sin, cos)
    TargetPosition.yaw = angle;
    return angle;
}

void DroneControl::ChangeToBodyFrame(){ // Change to body frame when sending velocity setpoints
    TargetPosition.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    TargetPosition.type_mask = 1024; // Ingoring yaw angle, using yaw rate
    TargetPosition.position.x = 0;
    TargetPosition.position.y = 0;
    TargetPosition.position.z = 0;
}

void DroneControl::ChangeToLocalFrame(){ // Change to Local(world) frame when sending position setpoints
    TargetPosition.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    TargetPosition.velocity.x = 0;
    TargetPosition.velocity.y = 0;
    TargetPosition.velocity.z = 0;
}