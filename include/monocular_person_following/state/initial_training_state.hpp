#ifndef MONOCULAR_PERSON_FOLLOWING_INITIAL_TRAINING_STATE_HPP
#define MONOCULAR_PERSON_FOLLOWING_INITIAL_TRAINING_STATE_HPP

#include <monocular_person_following/state/state.hpp>

namespace monocular_person_following {

class InitialTrainingState : public State {
public:
    InitialTrainingState(long target_id)
        : target_id(target_id),
          num_pos_samples(0)
    {}

    virtual ~InitialTrainingState() override {}

    virtual long target() const override {
        return target_id;
    }

    virtual std::string state_name() const override {
        return "initial_training";
    }

    virtual State* update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) override;

private:
    long target_id;
    long num_pos_samples;
};

}

#endif // INITIAL_TRAINING_STATE_HPP
