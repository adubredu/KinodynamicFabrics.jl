#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h> 
#include "pose_t.hpp"
#include "posearray_t.hpp"
#include <jlcxx/jlcxx.hpp>


using namespace std;

class Subpub{
    private:
        geometry_msgs::PoseArray poses; 
        ros::Subscriber subposearray; 

    public:
        Subpub();
        void pose_array_callback(const geometry_msgs::PoseArray::ConstPtr& posearray);
        std::vector<std::vector<double>> get_poses();
}; 

JLCXX_MODULE define_julia_module(jlcxx::Module& types)
{ 
    types.add_type<Subpub>("Subpub")
        .method("get_poses",[](Subpub& s){
            std::vector<std::vector<double>>poses = s.get_poses();
            jlcxx::Array<double> jposes;
            for (auto pose : poses){ 
                for (auto coord : pose){
                    jposes.push_back(coord);
                } 
            }
            return jposes;
        });
}