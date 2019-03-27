#include <monocular_person_following/state/initial_state.hpp>

#include <monocular_person_following/state/initial_training_state.hpp>

namespace monocular_person_following {

State* InitialState::update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) {
    long target_id = select_target(nh, tracks);

    if(target_id < 0) {
        return this;
    }

    return new InitialTrainingState(target_id);
}

long InitialState::select_target(ros::NodeHandle& nh, const std::unordered_map<long, Tracklet::Ptr>& tracks) {
    long target_id = -1;
    double distance = 0.0;

    for(const auto& track: tracks) {
        const auto& pos = track.second->pos_in_baselink;
        if(pos.x() > nh.param<double>("imprinting_max_dist", 4.0) || std::abs(pos.y()) > pos.x()) {
            continue;
        }

        if(target_id == -1 || distance > pos.norm()) {
            target_id = track.first;
            distance = pos.norm();
        }
    }

    return target_id;
}

}
