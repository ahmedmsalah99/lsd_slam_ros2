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

#include "live_slam_wrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/global_funcs.h"
#include "slam_system.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
//#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
//#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/undistorter.h"
//#include <ros/package.h>

#include "opencv2/opencv.hpp"


// Gets current projection matrix (= PerspectiveMatrix * CameraPoseMatrix)
template<typename Tracker>
Eigen::Matrix<double, 3, 4> get_projection(Tracker& tracker) {
    std::vector<double> cam_params = tracker.GetCameraParams();
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = cam_params[0];
    intrinsics(1, 1) = cam_params[1];
    intrinsics(0, 2) = cam_params[2];
    intrinsics(1, 2) = cam_params[3];

    std::vector<double> rot = tracker.GetCurrentPose();
    Eigen::Matrix<double, 3, 3> mrot;
    mrot << rot[0], rot[1], rot[2],
         rot[3], rot[4], rot[5],
         rot[6], rot[7], rot[8];
    Eigen::Matrix<double, 3, 4> pose;
    pose.setZero();
    pose.block<3, 3>(0, 0) = mrot;

    Eigen::Matrix<double, 3, 4> projection = intrinsics * pose;
    return projection;
}

// Draws desirable target in world coordinate to current color image
template<typename Tracker>
void draw_target(cv::Mat& rgb_img, Tracker& tracker) {
    const Eigen::Vector4d point_x(0.1, 0, 1, 1);
    const Eigen::Vector4d point_y(0, 0.1, 1, 1);
    const Eigen::Vector4d point_z(0, 0, 1.1, 1);
    const Eigen::Vector4d point_target(0, 0, 1.0, 1);
    Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
    Eigen::Vector3d point_cam = proj * point_target;
    Eigen::Vector3d pointx_cam = proj * point_x;
    Eigen::Vector3d pointy_cam = proj * point_y;
    Eigen::Vector3d pointz_cam = proj * point_z;
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(255, 0, 0), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(0, 0, 255), 3);
}


std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                    std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

bool recv_exact(int sock, uint8_t* buffer, size_t size) {
    size_t total_received = 0;
    while (total_received < size) {
        ssize_t bytes = recv(sock, buffer + total_received, size - total_received, 0);
        if (bytes <= 0) {
            return false;  // Connection closed or error
        }
        total_received += bytes;
    }
    return true;
}



int init_connection(int PORT,int &server_fd,int &client_fd ){
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    // Create socket
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("Socket failed");
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind
    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        return 1;
    }

    // Listen
    if (listen(server_fd, 1) < 0) {
        perror("Listen failed");
        close(server_fd);
        return 1;
    }

    std::cout << "Waiting for TCP connection on port " << PORT << "...\n";
    client_fd = accept(server_fd, (struct sockaddr*)&address, &addrlen);
    if (client_fd < 0) {
        perror("Accept failed");
        close(server_fd);
        return 1;
    }
    std::cout << "Client connected.\n";
    return 0;
}

cv::Mat read_sock_img(const int client_fd)
{
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t size_buf[4];
    if (!recv_exact(client_fd, size_buf, 4)) {
        std::cout << "Connection closed or error receiving size.\n";
        return cv::Mat();
        // break;
    }

    uint32_t buffer_size = (size_buf[0] << 24) | (size_buf[1] << 16) |
                            (size_buf[2] << 8) | size_buf[3];

    std::cout << "Expecting image of " << buffer_size << " bytes\n";
    std::vector<uint8_t> img_data(buffer_size);

    if (!recv_exact(client_fd, img_data.data(), buffer_size)) {
        std::cout << "Connection closed during image reception.\n";
        return cv::Mat();
        // break;
    }

    cv::Mat img_array(img_data);
    cv::Mat img = cv::imdecode(img_array, cv::IMREAD_GRAYSCALE);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;
    return img;
}
std::string append_slash_to_dirname(std::string dirname) {
    if(dirname[dirname.length()-1] == '/') {
        return dirname;
    }
    return dirname + "/";
}


using namespace lsd_slam;
int main(int argc, char* argv[])
{

    if(argc < 2) {
        std::cout << "Usage: $./bin/main_on_images_sock data/sequence_${sequence_number}/"
                  << std::endl;
        exit(-1);
    }

    const std::string dataset_root = append_slash_to_dirname(std::string(argv[1]));
    const std::string calib_file = dataset_root + "camera.txt";
    Undistorter* undistorter = 0;

    undistorter = Undistorter::getUndistorterForFile(calib_file);

    if(undistorter == 0)
    {
        printf("need camera calibration file! (set using _calib:=FILE)\n");
        exit(0);
    }

    int w = undistorter->getOutputWidth();
    int h = undistorter->getOutputHeight();
    float fx = undistorter->getK().at<double>(0, 0);
    float fy = undistorter->getK().at<double>(1, 1);
    float cx = undistorter->getK().at<double>(2, 0);
    float cy = undistorter->getK().at<double>(2, 1);
    Sophus::Matrix3f K;
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;


    // make output wrapper. just set to zero if no output is required.
    Output3DWrapper* outputWrapper = new Output3DWrapper(); // new ROSOutput3DWrapper(w,h);

    // make slam system
    SlamSystem* system = new SlamSystem(w, h, K, doSlam);
    system->setVisualization(outputWrapper);
    int status = 0;
    int output_server_fd, output_client_fd;
    status = init_connection(9002,output_server_fd,output_client_fd);
    if(status == 1){
        std::cout << "couldn't initiate connection" << std::endl;
        return 1;
    }

    int input_server_fd, input_client_fd;
    status = init_connection(9000,input_server_fd,input_client_fd);
    if(status == 1){
        std::cout << "couldn't initiate connection" << std::endl;
        return 1;
    }

    
    
    outputWrapper->setSocket(output_client_fd);




    cv::Mat image = cv::Mat(h,w,CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;


    while(true)
    {
        // std::cout << "reading " << files[i] << std::endl;

        // cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat imageDist = read_sock_img(input_client_fd);

        assert(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);
        assert(image.type() == CV_8U);
        auto start = std::chrono::high_resolution_clock::now();
        if(runningIDX == 0)
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        else
            system->trackFrame(image.data, runningIDX, false,fakeTimeStamp);
         auto end = std::chrono::high_resolution_clock::now();
         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;
        runningIDX++;
        fakeTimeStamp+=0.03;

        //if(hz != 0)
        //    r.sleep();

        if(fullResetRequested)
        {

            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->setVisualization(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }

        //ros::spinOnce();
    }


    system->finalize();


    close(input_client_fd);
    close(input_server_fd);
    delete system;
    delete undistorter;
    delete outputWrapper;
    return 0;
}
