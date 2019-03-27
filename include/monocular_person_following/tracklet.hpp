#ifndef TRACKLET_HPP
#define TRACKLET_HPP

#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <monocular_people_tracking/Track.h>
#include <ccf_person_identification/online_classifier.hpp>

namespace monocular_person_following {

struct Tracklet {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<Tracklet>;

    Tracklet(tf::TransformListener& tf_listener, const std_msgs::Header& header, const monocular_people_tracking::Track& track_msg)
        : confidence(boost::none),
          track_msg(&track_msg)
    {
        geometry_msgs::PointStamped point;
        point.header = header;
        point.point = track_msg.pos;

        geometry_msgs::PointStamped transformed;
        tf_listener.transformPoint("base_link", point, transformed);

        pos_in_baselink = Eigen::Vector2f(transformed.point.x, transformed.point.y);
    }

public:
    boost::optional<double> confidence;
    boost::optional<cv::Rect> person_region;
    ccf_person_classifier::Input::Ptr input;
    ccf_person_classifier::Features::Ptr features;

    Eigen::Vector2f pos_in_baselink;
    const monocular_people_tracking::Track* track_msg;
};

}

#endif // TRACKLET_HPP
