#include <monocular_person_following/state/initial_training_state.hpp>

#include <monocular_person_following/state/initial_state.hpp>
#include <monocular_person_following/state/tracking_state.hpp>

namespace monocular_person_following {

State* InitialTrainingState::update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) {
    auto found = tracks.find(target_id);
    if(found == tracks.end()) {
        ROS_INFO_STREAM("lost target during the initial training!!");
        return new InitialState();
    }

    for(const auto& track: tracks) {
        double label = track.first == target_id ? 1.0 : -1.0;
        boost::optional<double> pred = context.predict(track.second);

        context.update_classifier(label, track.second);

        if(label > 0.0) {
            num_pos_samples ++;
        }
    }

    if(num_pos_samples >= nh.param<int>("initial_training_num_samples", 10)) {
        return new TrackingState(target_id);
    }

    return this;
}

}
