#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <tfpose_ros/Persons.h>
#include <monocular_people_tracking/TrackArray.h>

#include <ccf_person_identification/ccf_person_identification.hpp>


class MonocularPersonFollowingNode {
public:
    MonocularPersonFollowingNode()
        : nh(),
          private_nh("~"),
          image_sub(nh, "/csi_cam_0/sd/image_rect", 10),
          poses_sub(nh, "/pose_estimator/pose", 10),
          tracks_sub(nh, "/monocular_people_tracking_node/tracks", 10),
          sync(image_sub, poses_sub, tracks_sub, 30)
    {
        sync.registerCallback(boost::bind(&MonocularPersonFollowingNode::callback, this, _1, _2, _3));
    }

    void callback(const sensor_msgs::ImageConstPtr& image_msg, const tfpose_ros::PersonsConstPtr& poses_msg, const monocular_people_tracking::TrackArrayConstPtr& tracks_msg) {
        auto cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");

    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    message_filters::Subscriber<tfpose_ros::Persons> poses_sub;
    message_filters::Subscriber<monocular_people_tracking::TrackArray> tracks_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, tfpose_ros::Persons, monocular_people_tracking::TrackArray> sync;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "monocular_person_following_node");
    std::unique_ptr<MonocularPersonFollowingNode> node(new MonocularPersonFollowingNode());
    ros::spin();

    return 0;
}
