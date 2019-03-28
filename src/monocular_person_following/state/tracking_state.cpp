#include <monocular_person_following/state/tracking_state.hpp>

#include <monocular_person_following/state/reid_state.hpp>

namespace monocular_person_following {

State* TrackingState::update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) {
    auto found = tracks.find(target_id);
    if(found == tracks.end()) {
        ROS_INFO_STREAM("target lost!!");
        return new ReidState();
    }

    boost::optional<double> pred = context.predict(found->second);
    if(pred && *pred < nh.param<double>("id_switch_detection_thresh", -0.1)) {
        ROS_INFO_STREAM("ID switch detected!!");
        return new ReidState();
    }

    if(!pred || *pred < nh.param<double>("min_target_cofidence", 0.1)) {
        ROS_INFO_STREAM("do not update");
        return this;
    }

    for(const auto& track: tracks) {
        double label = track.first == target_id ? 1.0 : -1.0;
        context.update_classifier(label, track.second);
    }

    return this;
}


}
