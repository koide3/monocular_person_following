#ifndef MONOCULAR_PERSON_FOLLOWING_INITIAL_STATE_HPP
#define MONOCULAR_PERSON_FOLLOWING_INITIAL_STATE_HPP

#include <monocular_person_following/state/state.hpp>

namespace monocular_person_following {

class InitialState : public State {
public:
    InitialState() {}

    virtual ~InitialState() override {}

    virtual std::string state_name() const override {
        return "initial";
    }

    virtual State* update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) override;

private:
    long select_target(ros::NodeHandle& nh, const std::unordered_map<long, Tracklet::Ptr>& tracks);
};

}

#endif // INITIAL_STATE_HPP
