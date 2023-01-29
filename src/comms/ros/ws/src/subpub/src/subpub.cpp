#include "subpub/subpub.hpp"

Subpub::Subpub(){
    char **d; int a;
    ros::init(a, d, "subpub_node");
    ros::NodeHandle nh;
    subposearray = nh.subscribe<geometry_msgs::PoseArray>("/tag_poses", 
                    1, &Subpub::pose_array_callback, this);
    ROS_INFO("READY"); 
    ros::spinOnce();
}

void Subpub::pose_array_callback(const geometry_msgs::PoseArray::ConstPtr& inposes){
    poses.header = inposes->header;
    poses.poses = inposes->poses; 

    return;
}

std::vector<std::vector<double>> Subpub::get_poses(){
    ros::spinOnce();
    std::vector<std::vector<double>> out_poses;
    for (auto pose : poses.poses){
        std::vector<double>out_pose;
        // out_pose.push_back(pose.position.x);
        out_pose.push_back(poses.header.seq);
        out_pose.push_back(pose.position.y);
        out_pose.push_back(pose.position.z);
        out_pose.push_back(pose.orientation.w);
        out_pose.push_back(pose.orientation.x);
        out_pose.push_back(pose.orientation.y);
        out_pose.push_back(pose.orientation.z);
        out_poses.push_back(out_pose);
    }
    return out_poses;
}