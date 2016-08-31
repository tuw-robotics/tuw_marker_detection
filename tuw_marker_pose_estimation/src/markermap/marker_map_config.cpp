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

MarkerDetails::MarkerDetails() {

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
    /*
    int a = (int) node["A"];
    int b = (int) node["X"];
    int c = (int) node["id"];
    */
}
