#ifndef MONOCULAR_PERSON_FOLLOWING_CONTEXT_HPP
#define MONOCULAR_PERSON_FOLLOWING_CONTEXT_HPP

#include <unordered_map>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <monocular_person_following/tracklet.hpp>

namespace ccf_person_classifier {

class PersonInput;
class PersonFeatures;
class PersonClassifier;

}


namespace monocular_person_following {

class Context {
public:
    using Input = ccf_person_classifier::PersonInput;
    using Features = ccf_person_classifier::PersonFeatures;
    using Classifier = ccf_person_classifier::PersonClassifier;

    Context(ros::NodeHandle& nh);
    ~Context();

public:
    void extract_features(const cv::Mat& bgr_image, std::unordered_map<long, Tracklet::Ptr>& tracks);
    bool update_classifier(double label, const Tracklet::Ptr& track);
    boost::optional<double> predict(const Tracklet::Ptr& track);

    cv::Mat visualize_body_features();

private:
    std::unique_ptr<Classifier> classifier;
};

}

#endif
