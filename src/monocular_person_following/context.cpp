#include <monocular_person_following/context.hpp>

#include <ccf_person_identification/person_classifier.hpp>

namespace monocular_person_following {

Context::Context(ros::NodeHandle& nh) {
    classifier.reset(new PersonClassifier(nh));

    pos_feature_bank.resize(nh.param<int>("feature_bank_size", 32));
    neg_feature_bank.resize(nh.param<int>("feature_bank_size", 32));
}

Context::~Context() {}


void Context::extract_features(const cv::Mat& bgr_image, std::unordered_map<long, Tracklet::Ptr>& tracks) {
    for(auto& track: tracks) {
        if(!track.second->person_region) {
            continue;
        }

        if(track.second->person_region->width < 20 || track.second->person_region->height < 20) {
            continue;
        }

        track.second->input.reset(new PersonInput());
        track.second->features.reset(new PersonFeatures());

        cv::Mat roi(bgr_image, *track.second->person_region);
        if(!classifier->extractInput(track.second->input, roi)) {
            ROS_WARN_STREAM("failed to extract input data");
            continue;
        }

        if(!classifier->extractFeatures(track.second->features, track.second->input)) {
            ROS_WARN_STREAM("failed to extract input features");
            continue;
        }
    }
}


bool Context::update_classifier(double label, const Tracklet::Ptr& track) {
    auto pred = classifier->predict(track->features);
    if(pred) {
        track->confidence = *pred;
    }

    auto& p_bank = label > 0.0 ? pos_feature_bank : neg_feature_bank;
    auto& n_bank = label > 0.0 ? neg_feature_bank : pos_feature_bank;

    if(!n_bank.empty()) {
        size_t i = std::uniform_int_distribution<>(0, n_bank.size())(mt);
        classifier->update(-label, n_bank[i]);
    }

    if(!p_bank.full()) {
        p_bank.push_back(track->features);
    } else {
        size_t i = std::uniform_int_distribution<>(0, p_bank.size())(mt);
        p_bank[i] = track->features;
    }


    return classifier->update(label, track->features);
}

boost::optional<double> Context::predict(const Tracklet::Ptr& track) {
    auto pred = classifier->predict(track->features);
    if(pred) {
        track->confidence = *pred;
    }

    return pred;
}

cv::Mat Context::visualize_body_features() {
    ccf_person_classifier::BodyClassifier::Ptr body_classifier = classifier->getClassifier<ccf_person_classifier::BodyClassifier>("body");
    if(body_classifier) {
        cv::Mat feature_map = body_classifier->visualize();
        return feature_map;
    }

    return cv::Mat();
}


}


