/*
 * Copyright (c) 2016, Lukas Pfeifhofer <lukas.pfeifhofer@devlabs.pro>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "markermap/marker_map_config.h"

using namespace cv;

MarkerMapConfig MarkerMapConfig::readFromFile(std::string path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    MarkerMapConfig markerMapConfig;
    fs["markerMapConfig"] >> markerMapConfig;
    return markerMapConfig;
}

void MarkerMapConfig::writeFromFile(std::string path, MarkerMapConfig markerMapConfig) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "markerMapConfig" << markerMapConfig;
    fs.release();
}

MarkerDetails::MarkerDetails() : id(-1), type("") {

}

void MarkerDetails::write(cv::FileStorage &fs) const {
    fs << "{";
        fs << "id" << this->id;
        fs << "type" << this->type;
        fs << "position" << this->position;
        fs << "rotation" << this->rotation;
    fs << "}";
}

void MarkerDetails::read(const cv::FileNode &node) {
    this->id = (int) node["id"];
    this->type = (std::string) node["type"];
    node["position"] >> this->position;
    node["rotation"] >> this->rotation;
}

MarkerMapDetails::MarkerMapDetails() {

}

void MarkerMapDetails::write(cv::FileStorage &fs) const {
    fs << "{";
        fs << "id" << this->id;
        fs << "type" << this->type;
        fs << "markers";
            fs << "{";
            fs << "size" << (int) this->markers.size();
            fs << "data";
                fs << "[";
                for(auto &markerDetails:this->markers){
                    fs << markerDetails;
                }
                fs << "]";
            fs << "}";
    fs << "}";
}

void MarkerMapDetails::read(const cv::FileNode &node) {
    this->id = (int) node["id"];
    this->type = (std::string) node["type"];

    FileNode markersNode = node["markers"];

    // int markersArraySize = markersNode["size"]; // Size is ignored
    FileNode dataNodes = markersNode["data"];
    if (dataNodes.type() != FileNode::SEQ) {
        return;
    }

    FileNodeIterator it = dataNodes.begin(), it_end = dataNodes.end();
    for (; it != it_end; ++it) {
        MarkerDetails markerDetails;
        *it >> markerDetails;
        this->markers.push_back(markerDetails);
    }
}


MarkerMapConfig::MarkerMapConfig() {

}

void MarkerMapConfig::write(cv::FileStorage &fs) const {
    fs << "{";
        fs << "size" << (int) this->markerMaps.size();
        fs << "data";
            fs << "[";
            for(auto &markerMapDetails:this->markerMaps){
                fs << markerMapDetails;
            }
            fs << "]";
    fs << "}";
}

void MarkerMapConfig::read(const cv::FileNode &node) {
    // int markersArraySize = node["size"]; // Size is ignored
    FileNode dataNodes = node["data"];
    if (dataNodes.type() != FileNode::SEQ) {
        return;
    }

    FileNodeIterator it = dataNodes.begin(), it_end = dataNodes.end();
    for (; it != it_end; ++it) {
        MarkerMapDetails markerMapDetails;
        *it >> markerMapDetails;
        this->markerMaps.push_back(markerMapDetails);
    }
}
