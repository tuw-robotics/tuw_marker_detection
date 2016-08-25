//
// Created by privacy on 25.08.16.
//

#ifndef TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_PARAMETERS_H
#define TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_PARAMETERS_H

class PoseEstimationParameters {
public:
    PoseEstimationParameters();
    ~PoseEstimationParameters();

    void setPoseEstimatorType(int type);
    void setPublishTf(bool b);
    void setPublishMarkers(bool b);

    int getPoseEstimatorType();
    bool getPublishTf();
    bool getPublishMarkers();

private:
    int pose_estimator_type_;
    bool publish_tf_;
    bool publish_markers_;

};

#endif //TUW_MARKER_POSE_ESTIMATION_POSE_ESTIMATION_PARAMETERS_H
