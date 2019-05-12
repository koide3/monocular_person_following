#ifndef MONOCULAR_PERSON_FOLLOWING_CONTEXT_HPP
#define MONOCULAR_PERSON_FOLLOWING_CONTEXT_HPP

#include <random>
#include <unordered_map>
#include <boost/optional.hpp>
#include <boost/circular_buffer.hpp>

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
    using PersonInput = ccf_person_classifier::PersonInput;
    using PersonFeatures = ccf_person_classifier::PersonFeatures;
    using PersonClassifier = ccf_person_classifier::PersonClassifier;

    Context(ros::NodeHandle& nh);
    ~Context();

public:
    void extract_features(const cv::Mat& bgr_image, std::unordered_map<long, Tracklet::Ptr>& tracks);
    bool update_classifier(double label, const Tracklet::Ptr& track);
    boost::optional<double> predict(const Tracklet::Ptr& track);

    cv::Mat visualize_body_features();

private:
    std::mt19937 mt;
    std::unique_ptr<PersonClassifier> classifier;

    boost::circular_buffer<std::shared_ptr<ccf_person_classifier::Features>> pos_feature_bank;
    boost::circular_buffer<std::shared_ptr<ccf_person_classifier::Features>> neg_feature_bank;
};

}

#endif
