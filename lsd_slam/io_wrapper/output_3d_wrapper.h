/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <string>
#include <vector>
#include "util/sophus_util.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include "lsd_slam/model/frame.h"
#include <unistd.h>
#include <cstring>
#include <opencv2/core/core.hpp>


namespace cv {
class Mat;
}

namespace lsd_slam
{

// class Frame;

class KeyFrameGraph;
// class Frame;
#define MAX_CLOUD_POINTS 300000
struct CloudPoint3D{
    float x;
    float y;
    float z;
};

struct CloudPoints3D
{
    uint32_t id;
    uint32_t cloud_points_num;
    float time;
    bool isKeyframe;
    CloudPoint3D cloud_points[MAX_CLOUD_POINTS];
    float scale;
    float camToWorld[7];
};

/**
 * Virtual 3D display object.
 */
class Output3DWrapper
{

public:
    Output3DWrapper(){};
    virtual ~Output3DWrapper();
    void setSocket(int output_client_fd);
    void sendBufferData(std::vector<uchar> buffer);
    virtual void publishKeyframeGraph(KeyFrameGraph* graph) {};

    // publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
    virtual void publishKeyframe(Frame* f);
        
     
 
             

    // published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
    virtual void publishTrackedFrame(Frame* kf) {};

    // publishes graph and all constraints, as well as updated KF poses.
    virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>>
                                   trajectory, std::string identifier) {};
    virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
                                            std::string identifier) {};

    virtual void publishDebugInfo(const Eigen::Matrix<float, 20, 1>& data) {};
    int initSocket();
private:
    // struct sockaddr_in server_addr_;
    int sock_fd_ = -1;
};
}
