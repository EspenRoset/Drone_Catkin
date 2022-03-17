#include "../include/ControlLibrary/DroneController.h"

//Github Test
void DroneControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void DroneControl::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_position = *msg;
}

void DroneControl::collision_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    Obstacle_detected = static_cast<int>(msg->data[2]);
    Obstacle_position = {msg->data[0], msg->data[1]};
}

void DroneControl::RunDrone(){
    while(true){
        switch (state)
        {
        case 0 /*StartUp*/: // Initialize, Change to OFFBOARD Mode, Arm Quad
                DroneControl::WaitForFCUConnection(); // Wait for FCU Connection
                DroneControl::SendNWaipoints(100); // Send some waypoints to prepare for mode change
                DroneControl::SetPX4Mode("OFFBOARD"); // Change mode to offboard
                // Wait for arm command
                ROS_INFO_STREAM("Waiting for arming");
                {
                std::unique_lock<std::mutex> lk(ArmMutex);
                cv.wait(lk, [&]{return ArmDrone;});
                }
                DroneControl::PX4Arm();
                // Wait for takeoff command
                ROS_INFO_STREAM("Waiting for Takeoff command");
                {
                std::unique_lock<std::mutex> lk(ArmMutex);
                cv.wait(lk, [&]{return InitiateTakeoff;});
                }
                state = Takeoff;

            break;
        case 1 /*TakeOff*/: // Check OFFBOARD and arm, Take off to flight altitude
                if ((current_state.mode == "OFFBOARD") && (current_state.armed)){
                    DroneControl::DroneTakeoff(TakeoffAltitude); // Takes of and changes state to flying
                } else{
                    state = Startup;
                }
            break;
        case 2 /*Flying*/: // Get input from controller and update Targetpotsiion
                TargetPosition = InputTargetPosition;
                if (InitiateLanding){
                    state = Landing;
                } if (Obstacle_detected){
                    state = AvoidObstacle;
                }
            break;
        case 3 /*Landing*/: // Land quad
                DroneControl::DroneLand();
            break;
        case 4 /*Landed*/: // Be ready to change mode to takeoff
                ROS_INFO_STREAM("Landed and waiting");
                ROS_INFO_STREAM("Press takeoff to fly");
                {
                std::unique_lock<std::mutex> lk(ArmMutex);
                cv.wait(lk, [&]{return InitiateTakeoff;});
                }
                state = Takeoff;
            break;
        case 5 /*Avoid Obstacle*/: // Limit movement crashing is avoided
                // Get input from Controller
                TargetPosition = InputTargetPosition;
                // Limit Movement
                TargetPosition.velocity.x = 0;
                if (!Obstacle_detected){
                    state = Flying;
                }
            break;
        default: // Maybe do something
            break;
        }
        pos_pub.publish(TargetPosition);
        //ROS_INFO_STREAM("Targetposition published");
        rate_->sleep();
    }
}

void DroneControl::WaitForFCUConnection(){
    while (ros::ok() && !current_state.connected)
            {
                ros::spinOnce();
                rate_->sleep();
            }
    ROS_INFO_STREAM("PX4 Connected");

}

void DroneControl::SendNWaipoints(int n){
     for (int i = n; ros::ok() && i > 0; --i)
  {
    pos_pub.publish(TargetPosition);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::SetPX4Mode(std::string Mode){
    offb_set_mode.request.custom_mode = Mode;
    while(ros::ok() && current_state.mode != Mode){
        if (current_state.mode != Mode && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    }
    }

}

void DroneControl::PX4Arm(){
    while(ros::ok() && !current_state.armed){
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    ArmDrone = false;
}

void DroneControl::DroneTakeoff(float altitude){
    if(ros::ok() && (current_position.pose.pose.position.z < altitude)){
        ROS_INFO_STREAM("Initiating Takeoff");
        TargetPosition.velocity.z = 0.5;
    }else {
    TargetPosition.velocity.z = 0;
    InitiateTakeoff = false;
    state = Flying;
    }
}

void DroneControl::DroneLand(){
    while (!(landing_client.call(land_cmd) &&
            land_cmd.response.success) ){
              ROS_INFO("trying to land");
            ros::spinOnce();
            rate_->sleep();
        }
    if (current_position.pose.pose.position.z < 0.1){
        InitiateLanding = false;
        state = Startup;         
    }
}