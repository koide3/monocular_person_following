#ifndef TRACKING_STATE_HPP
#define TRACKING_STATE_HPP

#include <monocular_person_following/state/state.hpp>

namespace monocular_person_following {

class TrackingState : public State {
public:
    TrackingState(long target_id)
        : target_id(target_id)
    {}

    virtual ~TrackingState() override {}

    virtual long target() const override {
        return target_id;
    }

    virtual std::string state_name() const override {
        return "tracking";
    }

    virtual State* update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) override;

private:
    long target_id;
};

}

#endif // TRACKING_STATE_HPP
