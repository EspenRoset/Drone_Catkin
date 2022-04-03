#include "../include/ControlLibrary/DroneController.h"

void DroneControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void DroneControl::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

void DroneControl::collision_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    Obstacle_detected = static_cast<int>(msg->data[2]);
    //Roof_limit = static_cast<int>(msg->data[3]); 
    Roof_limit = 0; //Disable roof_limit
    Floor_limit = static_cast<int>(msg->data[4]);
    AvoidReverse = msg->data[5];
    AvoidRoll = msg->data[6];
    Obstacle_position = {msg->data[0], msg->data[1], msg->data[3], msg->data[4]}; // X, Y, to high, too low
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
                DroneControl::SendNWaipoints(100); // Send some waypoints to prepare for mode change
                if (current_state.mode != "OFFBOARD"){
                    DroneControl::SetPX4Mode("OFFBOARD"); // Change mode to offboard
                }


                StartingHeight = current_position.pose.pose.position.z; //Reference height used for calcing takeoff height
                ROS_INFO_STREAM("Waiting for Takeoff command (Rb + A)");
                state = Takeoff;
                

            break;
        case 1 /*TakeOff*/: // Check OFFBOARD and arm, Take off to flight altitude
                if (InitiateTakeoff /*&& !Roof_limit*/){
                    DroneControl::DroneTakeoff(StartingHeight + TakeoffAltitude); // Takes of and changes state to flying
                }
            break;
        case 2 /*Flying*/: // Get input from controller and update Targetpotsiion
                TargetPosition = InputTargetPosition;
                if (InitiateLanding){
                    state = Landing;
                } if (Obstacle_detected || Roof_limit || Floor_limit){
                    state = AvoidObstacle;
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
                    state = Takeoff;
                }
            break;
        case 5 /*Avoid Obstacle*/: // Limit movement to avoid crashing
                // Get input from Controller
                TargetPosition = InputTargetPosition;
                ROS_INFO_STREAM("Avoiding obstacle");
                // Limit Movement
                if (Obstacle_detected){ // Use velocities from anti_collision to avoid obstacle
                    TargetPosition.velocity.x = TargetPosition.velocity.x + AvoidReverse;
                    TargetPosition.velocity.y = TargetPosition.velocity.y + AvoidRoll;
        
                }
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
                if (!Obstacle_detected && !Roof_limit && !Floor_limit){
                    state = Flying;
                }
            break;
        default: // Maybe do something
            break;
        }
        pos_pub.publish(TargetPosition);
        ROS_INFO_STREAM(state);
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
    if (current_position.pose.pose.position.z < 0.1){
        InitiateLanding = false;
        state = Landed;         
    }

}

void DroneControl::ReverseDrone(int reversemillisec){

}

