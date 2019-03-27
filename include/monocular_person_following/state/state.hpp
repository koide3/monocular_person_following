#ifndef MONOCULAR_PERSON_FOLLOWING_STATE_HPP
#define MONOCULAR_PERSON_FOLLOWING_STATE_HPP

#include <monocular_person_following/context.hpp>

namespace monocular_person_following {

class State {
public:
    State() {}
    virtual ~State() {}

    virtual long target() const { return -1; }

    virtual std::string state_name() const = 0;

    virtual State* update(ros::NodeHandle& nh, Context& context, const std::unordered_map<long, Tracklet::Ptr>& tracks) = 0;
private:

};

}

#endif
