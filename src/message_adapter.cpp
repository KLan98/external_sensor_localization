#include "ros/ros.h"
#include "std_msgs/String.h"
#include "external_sensor_localization/hedge_pos_ang.h"
#include "external_sensor_localization/hedge_imu_fusion.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"

#define ROS_NODE_NAME "message_adapter_node"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "hedge_pos_ang"
#define HEDGE_IMU_FUSION_TOPIC_NAME "hedge_imu_fusion"

#define ADAPTED_HEDGE_IMU_FUSION_TOPIC_NAME "adapted_imu"
#define ADAPTED_HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "adapted_pos_ang"

class message_adapter_node{
public:
    message_adapter_node(){
        ros::NodeHandle nodehandle; 
        ros::NodeHandle covhandle("~"); // for handling covariances 

        // Topics you want to subscribe
        // hedge_imu_sub = nodehandle.subscribe(HEDGE_IMU_FUSION_TOPIC_NAME, 1000, &message_adapter_node::ImuFusionCallback, this);
        hedge_pos_ang_sub = nodehandle.subscribe(HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 1000, &message_adapter_node::PosAngCallback, this);

        // Topics you want to publish
        // hedge_imu_pub = nodehandle.advertise<sensor_msgs::Imu>(ADAPTED_HEDGE_IMU_FUSION_TOPIC_NAME, 1000);
        hedge_pos_ang_pub = nodehandle.advertise<geometry_msgs::PoseWithCovarianceStamped>(ADAPTED_HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 1000);

        // Declare beacon frames
        // imu_obj.header.frame_id = "beacon_imu_frame";
        pose_obj.header.frame_id = "beacon_map_frame";

        // Data dumping to covariance matrices: orientation, angular velocity, linear acceleration and pose covariances
        cov_data_dump(covhandle);
    }
  
    // void callback(const SUBSCRIBED_MESSAGE_TYPE& input){
    //   PUBLISHED_MESSAGE_TYPE output;
    //   //.... do something with the input and generate the output...
    //   publish.publish(output);
    // }
  
    void PosAngCallback(const external_sensor_localization::hedge_pos_ang::ConstPtr& hedge_pos_msg){
        pose_obj.pose.pose.position.x = hedge_pos_msg->x_m; // coordinates in meters
        pose_obj.pose.pose.position.y = hedge_pos_msg->y_m;
        pose_obj.pose.pose.position.z = hedge_pos_msg->z_m;

        pose_obj.pose.pose.orientation = tf::createQuaternionMsgFromYaw(hedge_pos_msg->angle); 

        // publish geometry message
        hedge_pos_ang_pub.publish(pose_obj);
    }
    
    // void ImuFusionCallback(const external_sensor_localization::hedge_imu_fusion::ConstPtr& hedge_imu_msg){
    //     imu_obj.orientation.x = hedge_imu_msg->qx; // orientation quaternion of mobile beacon
    //     imu_obj.orientation.y = hedge_imu_msg->qy;
    //     imu_obj.orientation.z = hedge_imu_msg->qz;
    //     imu_obj.orientation.w = hedge_imu_msg->qw; // angle 

    //     imu_obj.angular_velocity.x = hedge_imu_msg->ax; // acceleration of mobile beacon m/s^2
    //     imu_obj.angular_velocity.y = hedge_imu_msg->ay;
    //     imu_obj.angular_velocity.z = hedge_imu_msg->az;

    //     imu_obj.linear_acceleration.x = hedge_imu_msg->vx; // speed vector of mobile beacon m/s
    //     imu_obj.linear_acceleration.y = hedge_imu_msg->vy;
    //     imu_obj.linear_acceleration.z = hedge_imu_msg->vz;

    //     hedge_imu_pub.publish(imu_obj);
    // }

    void cov_data_dump(ros::NodeHandle &covhandle){
        std::vector<float> pos_ang_cov;
        // std::vector<float> imu_angular_velocity_cov;
        // std::vector<float> imu_linear_acceleration_cov;
        // std::vector<float> imu_orientation_cov;

        covhandle.getParam("pos_ang_covariance", pos_ang_cov);
        // covhandle.getParam("angular_velocity_covariance", imu_angular_velocity_cov);
        // covhandle.getParam("linear_acceleration_covariance", imu_linear_acceleration_cov);
        // covhandle.getParam("orientation_covariance", imu_orientation_cov);     

        for(int i=0; i < pos_ang_cov.size(); i++){
            pose_obj.pose.covariance[i] = pos_ang_cov.at(i);
        }
        // for(int i=0; i < imu_angular_velocity_cov.size(); i++){
        //     imu_obj.angular_velocity_covariance[i] = imu_angular_velocity_cov.at(i);
        // }
        // for(int i=0; i < imu_linear_acceleration_cov.size(); i++){
        //     imu_obj.linear_acceleration_covariance[i] = imu_linear_acceleration_cov.at(i);
        // }
        // for(int i=0; i < imu_orientation_cov.size(); i++){
        //     imu_obj.orientation_covariance[i] = imu_orientation_cov.at(i);
        // }
    }

private:
    // publisher objects
    // ros::Publisher hedge_imu_pub;
    ros::Publisher hedge_pos_ang_pub;
  
    // subscriber objects
    // ros::Subscriber hedge_imu_sub;
    ros::Subscriber hedge_pos_ang_sub;

    // message objects
    // sensor_msgs::Imu imu_obj;
    geometry_msgs::PoseWithCovarianceStamped pose_obj;

};//End of class message_adapter_node

int main(int argc, char **argv){
    //Initiate ROS
    ros::init(argc, argv, ROS_NODE_NAME);
    
    //Create an object of class message_adapter_node that will take care of everything
    message_adapter_node adapter_obj;

    ros::spin();
   
    return 0;
}