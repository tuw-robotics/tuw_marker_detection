//
// Created by privacy on 31.08.16.
//

#ifndef TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H
#define TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H

#include "opencv2/opencv.hpp"

class MarkerMapDetails;

class MarkerDetails;

class MarkerMapConfig {

public:
    std::vector<MarkerMapDetails> markerMaps;

    MarkerMapConfig();

    void write(cv::FileStorage &fs) const;

    void read(const cv::FileNode &node);

    static MarkerMapConfig readFromFile(std::string path);

    static void writeFromFile(std::string path, MarkerMapConfig markerMapConfig);

};

static void write(cv::FileStorage &fs, const std::string &, const MarkerMapConfig &x) {
    x.write(fs);
}

static void read(const cv::FileNode &node, MarkerMapConfig &x,
                 const MarkerMapConfig &default_value = MarkerMapConfig()) {
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}


class MarkerMapDetails {
public:
    MarkerMapDetails();

    int id;
    std::string type;
    std::vector<MarkerDetails> markers;

    void write(cv::FileStorage &fs) const;

    void read(const cv::FileNode &node);
};

static void write(cv::FileStorage &fs, const std::string &, const MarkerMapDetails &x) {
    x.write(fs);
}

static void read(const cv::FileNode &node, MarkerMapDetails &x,
                 const MarkerMapDetails &default_value = MarkerMapDetails()) {
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}


class MarkerDetails {
public:
    int id;
    std::string type;
    cv::Mat position;
    cv::Mat rotation;

    MarkerDetails();

    void write(cv::FileStorage &fs) const;

    void read(const cv::FileNode &node);
};

static void write(cv::FileStorage &fs, const std::string &, const MarkerDetails &x) {
    x.write(fs);
}

static void read(const cv::FileNode &node, MarkerDetails &x,
                 const MarkerDetails &default_value = MarkerDetails()) {
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}


#endif //TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H
