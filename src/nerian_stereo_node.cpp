/*******************************************************************************
 * Copyright (c) 2016 Nerian Vision Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *******************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <visiontransfer/asynctransfer.h>
#include <visiontransfer/reconstruct3d.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <nerian_stereo/msg/stereo_camera_info.hpp>
#include <colorcoder.h>

using namespace std;

/**
 * \brief A driver node that receives data from SceneScan/SP1 and forwards
 * it to ROS.
 *
 * SceneScan and SP1 by Nerian Vision Technologies are hardware systems for
 * real-time stereo vision. They transmit a computed disparity map (an
 * inverse depth map) through gigagibt ethernet, which is then received by
 * this node. The node converts the received data into ROS messages, which
 * contain the following data:
 *
 * - Point cloud of reconstructed 3D locations
 * - Disparity map with optional color coding
 * - Rectified left camera image
 *
 * In addition, camera calibration information is also published. For
 * configuration parameters, please see the provided example launch file.
 * For more information about Nerian's SceneScan system, please visit
 * http://nerian.com/products/scenescan-stereo-vision/
 */

class StereoNode {
public:
    StereoNode(): frameNum(0) {
    }

    ~StereoNode() {
    }

    /**
     * \brief Performs general initializations
     */
    void init() {
        nodeObj = rclcpp::Node::make_shared("nerian_stereo");
        clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Read all ROS parameters
        // TODO: Test!

        rclcpp::parameter::ParameterVariant param;
        if(!nodeObj->get_parameter("point_cloud_intensity_channel", param)) {
            intensityChannel = true;
        } else {
            intensityChannel = param.as_bool();
        }

        if(!nodeObj->get_parameter("color_code_disparity_map", param)) {
            colorCodeDispMap = "";
        } else {
            colorCodeDispMap = param.as_string();
        }

        if(!nodeObj->get_parameter("color_code_legend", param)) {
            colorCodeLegend = false;
        } else {
            colorCodeLegend = param.as_bool();
        }

        if(!nodeObj->get_parameter("frame", param)) {
            frame = "world";
        } else {
            frame = param.as_string();
        }

        if(!nodeObj->get_parameter("remote_port", param)) {
            remotePort = "7681";
        } else {
            remotePort = param.as_string();
        }

        if(!nodeObj->get_parameter("remote_host", param)) {
            remoteHost = "0.0.0.0";
        } else {
            remoteHost = param.as_string();
        }

        if(!nodeObj->get_parameter("local_port", param)) {
            localPort = "7681";
        } else {
            localPort = param.as_string();
        }

        if(!nodeObj->get_parameter("local_host", param)) {
            localHost = "0.0.0.0";
        } else {
            localHost = param.as_string();
        }

        if(!nodeObj->get_parameter("use_tcp", param)) {
            useTcp = false;
        } else {
            useTcp = param.as_bool();
        }

        if(!nodeObj->get_parameter("ros_coordinate_system", param)) {
            rosCoordinateSystem = true;
        } else {
            rosCoordinateSystem = param.as_bool();
        }

        if(!nodeObj->get_parameter("calibration_file", param)) {
            calibFile = "";
        } else {
            calibFile = param.as_string();
        }

        if(!nodeObj->get_parameter("delay_execution", param)) {
            execDelay = 0;
        } else {
            execDelay = param.as_int();
        }

        if(!nodeObj->get_parameter("max_depth", param)) {
            maxDepth = -1;
        } else {
            maxDepth = param.as_int();
        }

        // Apply an initial delay if configured
        std::this_thread::sleep_for(std::chrono::duration<double>(execDelay));

        // Create publishers
        disparityPublisher = nodeObj->create_publisher<sensor_msgs::msg::Image>(
            "/nerian_stereo/disparity_map", rmw_qos_profile_default);
        leftImagePublisher = nodeObj->create_publisher<sensor_msgs::msg::Image>(
            "/nerian_stereo/left_image", rmw_qos_profile_default);
        rightImagePublisher = nodeObj->create_publisher<sensor_msgs::msg::Image>(
            "/nerian_stereo/right_image", rmw_qos_profile_default);

        loadCameraCalibration();

        cameraInfoPublisher = nodeObj->create_publisher<nerian_stereo::msg::StereoCameraInfo>(
            "/nerian_stereo/stereo_camera_info", rmw_qos_profile_default);
        cloudPublisher = nodeObj->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/nerian_stereo/point_cloud", rmw_qos_profile_default);
    }

    /**
     * \brief The main loop of this node
     */
    int run() {
        try {
            rclcpp::Time lastLogTime = clock->now();
            int lastLogFrames = 0;

            AsyncTransfer asyncTransfer(useTcp ? ImageTransfer::TCP_CLIENT : ImageTransfer::UDP,
                remoteHost.c_str(), remotePort.c_str(), localHost.c_str(), localPort.c_str());

            while(rclcpp::ok()) {
                // Receive image data
                ImagePair imagePair;
                if(!asyncTransfer.collectReceivedImagePair(imagePair, 0.5)) {
                    continue;
                }

                rclcpp::Time stamp = clock->now();

                // Publish image data messages
                publishImageMsg(imagePair, 0, stamp, false, leftImagePublisher);
                if(imagePair.isImageDisparityPair()) {
                    publishImageMsg(imagePair, 1, stamp, true, disparityPublisher);
                } else {
                    publishImageMsg(imagePair, 1, stamp, false, rightImagePublisher);
                }

                // TODO
                //if(cloudPublisher->getNumSubscribers() > 0) {
                    if(recon3d == nullptr) {
                        // First initialize
                        initPointCloud();
                    }

                    publishPointCloudMsg(imagePair, stamp);
                //}

                // TODO
                if(cameraInfoPublisher != NULL) {// && cameraInfoPublisher->getNumSubscribers() > 0) {
                    publishCameraInfo(stamp, imagePair);
                }

                // Display some simple statistics
                frameNum++;
                uint64_t dt = stamp.nanoseconds() - lastLogTime.nanoseconds();
                if(dt > 1e9) {
                    double fps = (frameNum - lastLogFrames) / (dt*1e-9);
                    printf("%.1f fps\n", fps);

                    lastLogFrames = frameNum;
                    lastLogTime = stamp;
                }
            }
        } catch(const std::exception& ex) {
            cerr << "Exception occured: %s" << ex.what() << endl;
        }
    }

private:
    // ROS related objects
    std::shared_ptr<rclcpp::Node> nodeObj;
    std::shared_ptr<rclcpp::Clock> clock;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > cloudPublisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > disparityPublisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > leftImagePublisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > rightImagePublisher;
    std::shared_ptr<rclcpp::Publisher<nerian_stereo::msg::StereoCameraInfo> > cameraInfoPublisher;

    // Parameters
    bool intensityChannel;
    bool useTcp;
    std::string colorCodeDispMap;
    bool colorCodeLegend;
    bool rosCoordinateSystem;
    std::string remotePort;
    std::string localPort;
    std::string frame;
    std::string remoteHost;
    std::string localHost;
    std::string calibFile;
    double execDelay;
    double maxDepth;

    // Other members
    int frameNum;
    std::shared_ptr<Reconstruct3D> recon3d;
    std::shared_ptr<ColorCoder> colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointCloudMsg;
    cv::FileStorage calibStorage;
    std::shared_ptr<nerian_stereo::msg::StereoCameraInfo> camInfoMsg;
    rclcpp::Time lastCamInfoPublish;

    /**
     * \brief Loads a camera calibration file if configured
     */
    void loadCameraCalibration() {
        if(calibFile == "" ) {
            cerr << "No camera calibration file configured. Cannot publish detailed camera information!" << endl;
        } else {
            bool success = false;
            try {
                if (calibStorage.open(calibFile, cv::FileStorage::READ)) {
                    success = true;
                }
            } catch(...) {
            }

            if(!success) {
                cerr << "Error reading calibration file: " << calibFile.c_str() << endl
                    << "Cannot publish detailed camera information!" << endl;
            }
        }
    }

    /**
     * \brief Publishes the disparity map as 16-bit grayscale image or color coded
     * RGB image
     */
    void publishImageMsg(const ImagePair& imagePair, int imageIndex, rclcpp::Time stamp, bool allowColorCode,
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image> > publisher) {

        // TODO
        //if(publisher->getNumSubscribers() <= 0) {
            //return; //No subscribers
        //}

        std::shared_ptr<sensor_msgs::msg::Image> msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.frame_id = frame;
        msg->header.stamp = stamp;

        bool format12Bit = (imagePair.getPixelFormat(imageIndex) == ImagePair::FORMAT_12_BIT);
        cv::Mat monoImg(imagePair.getHeight(), imagePair.getWidth(),
            format12Bit ? CV_16UC1 : CV_8UC1,
            imagePair.getPixelData(imageIndex), imagePair.getRowStride(imageIndex));

        if(colorCodeDispMap == "" || colorCodeDispMap == "none" || !allowColorCode || !format12Bit) {
            msg->encoding = (format12Bit ? "mono16": "mono8");
            msg->height = monoImg.rows;
            msg->width = monoImg.cols;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(monoImg.step);
            msg->data.resize(monoImg.step * monoImg.rows);
            memcpy(&msg->data[0], monoImg.data, msg->data.size());
        } else {
            if(colCoder == NULL) {
                int dispMin = 0, dispMax = 0;
                imagePair.getDisparityRange(dispMin, dispMax);

                colCoder.reset(new ColorCoder(
                    colorCodeDispMap == "rainbow" ? ColorCoder::COLOR_RAINBOW_BGR : ColorCoder::COLOR_RED_BLUE_BGR,
                    dispMin*16, dispMax*16, true, true));
                if(colorCodeLegend) {
                    // Create the legend
                    colDispMap = colCoder->createLegendBorder(monoImg.cols, monoImg.rows, 1.0/16.0);
                } else {
                    colDispMap = cv::Mat_<cv::Vec3b>(monoImg.rows, monoImg.cols);
                }
            }

            cv::Mat_<cv::Vec3b> dispSection = colDispMap(cv::Rect(0, 0, monoImg.cols, monoImg.rows));
            colCoder->codeImage(cv::Mat_<unsigned short>(monoImg), dispSection);

            msg->encoding = "bgr8";
            msg->height = colDispMap.rows;
            msg->width = colDispMap.cols;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(colDispMap.step);
            msg->data.resize(colDispMap.step * colDispMap.rows);
            memcpy(&msg->data[0], colDispMap.data, msg->data.size());
        }

        publisher->publish(msg);
    }

    /**
     * \brief Transform Q matrix to match the ROS coordinate system:
     * Swap y/z axis, then swap x/y axis, then invert y and z axis.
     */
    void qMatrixToRosCoords(const float* src, float* dst) {
        dst[0] = src[8];   dst[1] = src[9];
        dst[2] = src[10];  dst[3] = src[11];

        dst[4] = -src[0];  dst[5] = -src[1];
        dst[6] = -src[2];  dst[7] = -src[3];

        dst[8] = -src[4];  dst[9] = -src[5];
        dst[10] = -src[6]; dst[11] = -src[7];

        dst[12] = src[12]; dst[13] = src[13];
        dst[14] = src[14]; dst[15] = src[15];
    }

    /**
     * \brief Reconstructs the 3D locations form the disparity map and publishes them
     * as point cloud.
     */
    void publishPointCloudMsg(ImagePair& imagePair, rclcpp::Time stamp) {
        if(imagePair.getPixelFormat(1) != ImagePair::FORMAT_12_BIT) {
            return; // This is not a disparity map
        }

        // Transform Q-matrix if desired
        float qRos[16];
        if(rosCoordinateSystem) {
            qMatrixToRosCoords(imagePair.getQMatrix(), qRos);
            imagePair.setQMatrix(qRos);
        }

        // Get 3D points
        float* pointMap = nullptr;
        try {
            pointMap = recon3d->createPointMap(imagePair, 0);
        } catch(std::exception& ex) {
            cerr << "Error creating point cloud: " << ex.what() << endl;
            return;
        }

        // Create message object and set header
        pointCloudMsg->header.stamp = stamp;
        pointCloudMsg->header.frame_id = frame;

        // Copy 3D points
        if(pointCloudMsg->data.size() != imagePair.getWidth()*imagePair.getHeight()*4*sizeof(float)) {
            // Allocate buffer
            pointCloudMsg->data.resize(imagePair.getWidth()*imagePair.getHeight()*4*sizeof(float));

            // Set basic data
            pointCloudMsg->width = imagePair.getWidth();
            pointCloudMsg->height = imagePair.getHeight();
            pointCloudMsg->is_bigendian = false;
            pointCloudMsg->point_step = 4*sizeof(float);
            pointCloudMsg->row_step = imagePair.getWidth() * pointCloudMsg->point_step;
            pointCloudMsg->is_dense = false;
        }

        if(maxDepth < 0) {
            // Just copy everything
            memcpy(&pointCloudMsg->data[0], pointMap,
                imagePair.getWidth()*imagePair.getHeight()*4*sizeof(float));
        } else {
            // Only copy points up to maximum depth
            if(rosCoordinateSystem) {
                copyPointCloudClamped<0>(pointMap, reinterpret_cast<float*>(&pointCloudMsg->data[0]),
                    imagePair.getWidth()*imagePair.getHeight());
            } else {
                copyPointCloudClamped<2>(pointMap, reinterpret_cast<float*>(&pointCloudMsg->data[0]),
                    imagePair.getWidth()*imagePair.getHeight());
            }
        }

        // Copy intensity values
        if(intensityChannel) {
            // Get pointers to the beginnig and end of the point cloud
            unsigned char* cloudStart = &pointCloudMsg->data[0];
            unsigned char* cloudEnd = &pointCloudMsg->data[0]
                + imagePair.getWidth()*imagePair.getHeight()*4*sizeof(float);

            if(imagePair.getPixelFormat(0) == ImagePair::FORMAT_8_BIT) {
                // Get pointer to the current pixel and end of current row
                unsigned char* imagePtr = imagePair.getPixelData(0);
                unsigned char* rowEndPtr = imagePtr + imagePair.getWidth();
                int rowIncrement = imagePair.getRowStride(0) - imagePair.getWidth();

                for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                        cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {
                    *cloudPtr = *imagePtr;

                    imagePtr++;
                    if(imagePtr == rowEndPtr) {
                        // Progress to next row
                        imagePtr += rowIncrement;
                        rowEndPtr = imagePtr + imagePair.getWidth();
                    }
                }
            } else { // 12-bit
                // Get pointer to the current pixel and end of current row
                unsigned short* imagePtr = reinterpret_cast<unsigned short*>(imagePair.getPixelData(0));
                unsigned short* rowEndPtr = imagePtr + imagePair.getWidth();
                int rowIncrement = imagePair.getRowStride(0) - 2*imagePair.getWidth();

                for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                        cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {
                    *cloudPtr = *imagePtr/16;

                    imagePtr++;
                    if(imagePtr == rowEndPtr) {
                        // Progress to next row
                        imagePtr += rowIncrement;
                        rowEndPtr = imagePtr + imagePair.getWidth();
                    }
                }
            }
        }

        cloudPublisher->publish(pointCloudMsg);
    }

    /**
     * \brief Copies all points in a point cloud that have a depth smaller
     * than maxDepth. Other points are set to NaN.
     */
    template <int coord> void copyPointCloudClamped(float* src, float* dst, int size) {
        // Only copy points that are below the minimum depth
        float* endPtr = src + 4*size;
        for(float *srcPtr = src, *dstPtr = dst; srcPtr < endPtr; srcPtr+=4, dstPtr+=4) {
            if(srcPtr[coord] > maxDepth) {
                dstPtr[0] = std::numeric_limits<float>::quiet_NaN();
                dstPtr[1] = std::numeric_limits<float>::quiet_NaN();
                dstPtr[2] = std::numeric_limits<float>::quiet_NaN();
            } else {
                dstPtr[0] = srcPtr[0];
                dstPtr[1] = srcPtr[1];
                dstPtr[2] = srcPtr[2];
            }
        }
    }

    /**
     * \brief Performs all neccessary initializations for point cloud+
     * publishing
     */
    void initPointCloud() {
        // Initialize 3D reconstruction class
        recon3d.reset(new Reconstruct3D);

        // Initialize message
        pointCloudMsg.reset(new sensor_msgs::msg::PointCloud2);

        // Set channel information.
        sensor_msgs::msg::PointField fieldX;
        fieldX.name ="x";
        fieldX.offset = 0;
        fieldX.datatype = sensor_msgs::msg::PointField::FLOAT32;
        fieldX.count = 1;
        pointCloudMsg->fields.push_back(fieldX);

        sensor_msgs::msg::PointField fieldY;
        fieldY.name ="y";
        fieldY.offset = sizeof(float);
        fieldY.datatype = sensor_msgs::msg::PointField::FLOAT32;
        fieldY.count = 1;
        pointCloudMsg->fields.push_back(fieldY);

        sensor_msgs::msg::PointField fieldZ;
        fieldZ.name ="z";
        fieldZ.offset = 2*sizeof(float);
        fieldZ.datatype = sensor_msgs::msg::PointField::FLOAT32;
        fieldZ.count = 1;
        pointCloudMsg->fields.push_back(fieldZ);

        if(intensityChannel) {
            sensor_msgs::msg::PointField fieldI;
            fieldI.name ="intensity";
            fieldI.offset = 3*sizeof(float);
            fieldI.datatype = sensor_msgs::msg::PointField::UINT8;
            fieldI.count = 1;
            pointCloudMsg->fields.push_back(fieldI);
        }
    }

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(rclcpp::Time stamp, const ImagePair& imagePair) {
        if(camInfoMsg == NULL) {
            // Initialize the camera info structure
            camInfoMsg.reset(new nerian_stereo::msg::StereoCameraInfo);

            camInfoMsg->header.frame_id = frame;

            if(calibFile != "") {
                std::vector<int> sizeVec;
                calibStorage["size"] >> sizeVec;
                if(sizeVec.size() != 2) {
                    std::runtime_error("Calibration file format error!");
                }

                camInfoMsg->left_info.header = camInfoMsg->header;
                camInfoMsg->left_info.width = sizeVec[0];
                camInfoMsg->left_info.height = sizeVec[1];
                camInfoMsg->left_info.distortion_model = "plumb_bob";
                calibStorage["D1"] >> camInfoMsg->left_info.d;
                readCalibrationArray("M1", camInfoMsg->left_info.k);
                readCalibrationArray("R1", camInfoMsg->left_info.r);
                readCalibrationArray("P1", camInfoMsg->left_info.p);
                camInfoMsg->left_info.binning_x = 1;
                camInfoMsg->left_info.binning_y = 1;
                camInfoMsg->left_info.roi.do_rectify = false;
                camInfoMsg->left_info.roi.height = 0;
                camInfoMsg->left_info.roi.width = 0;
                camInfoMsg->left_info.roi.x_offset = 0;
                camInfoMsg->left_info.roi.y_offset = 0;

                camInfoMsg->right_info.header = camInfoMsg->header;
                camInfoMsg->right_info.width = sizeVec[0];
                camInfoMsg->right_info.height = sizeVec[1];
                camInfoMsg->right_info.distortion_model = "plumb_bob";
                calibStorage["D2"] >> camInfoMsg->right_info.d;
                readCalibrationArray("M2", camInfoMsg->right_info.k);
                readCalibrationArray("R2", camInfoMsg->right_info.r);
                readCalibrationArray("P2", camInfoMsg->right_info.p);
                camInfoMsg->right_info.binning_x = 1;
                camInfoMsg->right_info.binning_y = 1;
                camInfoMsg->right_info.roi.do_rectify = false;
                camInfoMsg->right_info.roi.height = 0;
                camInfoMsg->right_info.roi.width = 0;
                camInfoMsg->right_info.roi.x_offset = 0;
                camInfoMsg->right_info.roi.y_offset = 0;

                readCalibrationArray("Q", camInfoMsg->q);
                readCalibrationArray("T", camInfoMsg->t_left_right);
                readCalibrationArray("R", camInfoMsg->r_left_right);
            }
        }

        uint64_t dt = stamp.nanoseconds() - lastCamInfoPublish.nanoseconds();
        if(dt > 1e6) {
            // Rather use the Q-matrix that we received over the network if it is valid
            const float* qMatrix = imagePair.getQMatrix();
            if(qMatrix[0] != 0.0) {
                for(int i=0; i<16; i++) {
                    camInfoMsg->q[i] = static_cast<double>(qMatrix[i]);
                }
            }

            // Publish once per second
            camInfoMsg->header.stamp = stamp;
            camInfoMsg->left_info.header.stamp = stamp;
            camInfoMsg->right_info.header.stamp = stamp;
            cameraInfoPublisher->publish(camInfoMsg);

            lastCamInfoPublish = stamp;
        }
    }

    /**
     * \brief Reads a vector from the calibration file to a boost:array
     */
    template<class T>
    void readCalibrationArray(const char* key, T& dest) {
        std::vector<double> doubleVec;
        calibStorage[key] >> doubleVec;

        if(doubleVec.size() != dest.size()) {
            std::runtime_error("Calibration file format error!");
        }

        std::copy(doubleVec.begin(), doubleVec.end(), dest.begin());
    }
};

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        StereoNode stereoNode;
        stereoNode.init();
        return stereoNode.run();
    } catch(const std::exception& ex) {
        cerr << "Exception occured: " <<  ex.what() << endl; // TODO:: Use ROS logging again
        return 1;
    }
}
