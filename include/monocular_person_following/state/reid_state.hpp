#ifndef REID_STATE_HPP
#define REID_STATE_HPP

#include <monocular_person_following/state/state.hpp>

namespace monocular_person_following {

class ReidState : public State {
public:
    ReidState() {}
    virtual ~ReidState() override {}

    virtual std::string state_name() const override {
        return "re-identification";
    }

    virtual State* update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) override;

private:
    std::unordered_map<long, int> positive_count;
};

}

#endif // REID_STATE_HPP
