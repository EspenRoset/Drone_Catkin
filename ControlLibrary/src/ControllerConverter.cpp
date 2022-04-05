#include "../include/ControlLibrary/ControllerConverter.h"

void sayHello()
{
    ROS_INFO("Hello!");
}

void ControllerConverter::ControlUpdate(const sensor_msgs::JoyConstPtr& msg){
 // Update values on change
 ControllerConverter::Vx_ = msg->axes[4] * ControllerConverter::Scaling[0];
 ControllerConverter::Vz_ = msg->axes[1] * ControllerConverter::Scaling[2];
 ControllerConverter::Yaw_ = msg->axes[3]*ControllerConverter::Scaling[1];
 ControllerConverter::Vmultiplier = (((msg->axes[5]-1)*-1)+1)*ControllerConverter::Scaling[3];
 ControllerConverter::takeoffBtn = msg->buttons[0];
 ControllerConverter::landBtn = msg->buttons[1];
 ControllerConverter::armingBtn = msg->buttons[5];
 ControllerConverter::returnBtn = msg->buttons[2];
 if (armingBtn){
     Arm = true;
 } else{
     Arm = false;
 }
 if (armingBtn && landBtn){
     Land = true;
 } else{
     Land = false;
 }
 if (armingBtn && takeoffBtn){
     Takeoff = true;
 } else{
     Takeoff = false;
 }
 if (armingBtn && returnBtn){
     Return = true;
 } else{
     Return = false;
 }
}

void ControllerConverter::pose_cb(const nav_msgs::Odometry::ConstPtr& msg){
    ControllerConverter::Z = msg->pose.pose.position.z;
}

void ControllerConverter::CalcYaw(float yaw){
    if (((Yaw_Temp_ +yaw) < 360) && ((Yaw_Temp_ +yaw) > 0)){
        Yaw_Temp_  += yaw;   
    } else if ((Yaw_Temp_ +yaw)>360){
        Yaw_Temp_  = 0;
    } else if ((Yaw_Temp_ +yaw)<0){
        Yaw_Temp_  = 359;
    }
}

void ControllerConverter::UpdateVelocities(){
    while(true){
        ControllerConverter::Target.velocity.x = Vx_*Vmultiplier;
        ControllerConverter::Target.velocity.z = Vz_*Vmultiplier;
        ControllerConverter::Target.yaw_rate = Yaw_*Vmultiplier;
        rate_->sleep();
    }
}