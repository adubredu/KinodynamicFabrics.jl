#include "subpub/subpub.hpp"

Subpub::Subpub(ros::NodeHandle* nodehandle):nh(*nodehandle){
    ros::init(argc, argv, "subpub_node");
    ros::NodeHandle nh;
    subposearray = nh.subscribe<geometry_msgs::PoseArray>("/tag_poses", 
                    1, &Subpub::pose_array_callback, this);
    ROS_INFO("READY");
}

void Subpub::pose_array_callback(const geometry_msgs::PoseArray::ConstPtr& inposes){
    poses.header = inposes->header;
    poses.poses = inposes->poses;

    // posearray_t lcmposes;
    // for (auto pose : inposes->poses){
    //     pose_t lcmpose;
    //     lcmpose.timestamp = inposes->header.stamp.nsec;
    //     lcmpose.x = pose.position.x;
    //     lcmpose.y = pose.position.y; 
    //     lcmpose.z = pose.position.z;
    //     lcmpose.qw = pose.orientation.w;
    //     lcmpose.qx = pose.orientation.x;
    //     lcmpose.qy = pose.orientation.y;
    //     lcmpose.qz = pose.orientation.z;
    //     lcmposes.poses.push_back(lcmpose);
    // }
    // lcmposes.num_poses = lcmposes.poses.size();
    // lcm.publish("tag_poses", &lcmposes);


    ROS_INFO("Sequence: %d", inposes->header.seq);

    return;
}

std::vector<std::vector<double>> Subpub::get_poses(){
    std::vector<std::vector<double>> out_poses;
    for (auto pose : poses.poses){
        std::vector<double>out_pose;
        out_pose.push_back(pose.position.x);
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