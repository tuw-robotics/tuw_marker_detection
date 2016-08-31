//
// Created by privacy on 31.08.16.
//

#ifndef TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H
#define TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H

#include <tf/tf.h>

class MarkerMapDetails;
class MarkerDetails;

class MarkerMapConfig {

public:
    std::vector <MarkerMapDetails> markerMaps;

};

class MarkerMapDetails {
public:
    int id;
    std::string type;
    std::vector <MarkerDetails> markers;
};

class MarkerDetails {
public:
    int id;
    std::string type;
    tf::Vector3 position;
    tf::Quaternion rotation;
};

#endif //TUW_MARKER_POSE_ESTIMATION_MARKER_MAP_CONFIG_H
