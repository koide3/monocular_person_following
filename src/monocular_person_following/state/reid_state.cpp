#include <monocular_person_following/state/reid_state.hpp>

#include <monocular_person_following/state/tracking_state.hpp>

namespace monocular_person_following {

State* ReidState::update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) {
    for(const auto& track: tracks) {
        boost::optional<double> pred = context.predict(track.second);

        if(!pred || pred < nh.param<double>("reid_confidence_thresh", 0.2)) {
            continue;
        }

        auto found = positive_count.find(track.first);
        if(found == positive_count.end()) {
            positive_count[track.first] = 0;
        }
        positive_count[track.first] ++;

        if(positive_count[track.first] >= nh.param<int>("reid_positive_count", 5)) {
            return new TrackingState(track.first);
        }
    }

    return this;
}

}
