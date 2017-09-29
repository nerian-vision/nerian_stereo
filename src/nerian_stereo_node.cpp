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

#include <ros/ros.h>
#include <visiontransfer/asynctransfer.h>
#include <visiontransfer/reconstruct3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <iomanip>
#include <nerian_stereo/StereoCameraInfo.h>
#include <boost/smart_ptr.hpp>
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
        ros::NodeHandle privateNh("~");

        // Read all ROS parameters
        if (!privateNh.getParam("point_cloud_intensity_channel", intensityChannel)) {
            intensityChannel = true;
        }

        if (!privateNh.getParam("color_code_disparity_map", colorCodeDispMap)) {
            colorCodeDispMap = true;
        }

        if (!privateNh.getParam("color_code_legend", colorCodeLegend)) {
            colorCodeLegend = false;
        }

        if (!privateNh.getParam("frame", frame)) {
            frame = "world";
        }

        if (!privateNh.getParam("remote_port", remotePort)) {
            remotePort = "7681";
        }

        if (!privateNh.getParam("remote_host", remoteHost)) {
            remoteHost = "0.0.0.0";
        }

        if (!privateNh.getParam("local_port", localPort)) {
            localPort = "7681";
        }

        if (!privateNh.getParam("local_host", localHost)) {
            localHost = "0.0.0.0";
        }

        if (!privateNh.getParam("use_tcp", useTcp)) {
            useTcp = false;
        }

        if (!privateNh.getParam("ros_coordinate_system", rosCoordinateSystem)) {
            rosCoordinateSystem = true;
        }

        if (!privateNh.getParam("calibration_file", calibFile)) {
            calibFile = "";
        }

        if (!privateNh.getParam("delay_execution", execDelay)) {
            execDelay = 0;
        }

        if (!privateNh.getParam("max_depth", maxDepth)) {
            maxDepth = -1;
        }

        // Apply an initial delay if configured
        ros::Duration(execDelay).sleep();

        // Create publishers
        disparityPublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_stereo/disparity_map", 5)));
        leftImagePublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_stereo/left_image", 5)));
        rightImagePublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::Image>(
            "/nerian_stereo/right_image", 5)));

        loadCameraCalibration();

        cameraInfoPublisher.reset(new ros::Publisher(nh.advertise<nerian_stereo::StereoCameraInfo>(
            "/nerian_stereo/stereo_camera_info", 1)));
        cloudPublisher.reset(new ros::Publisher(nh.advertise<sensor_msgs::PointCloud2>(
            "/nerian_stereo/point_cloud", 5)));
    }

    /**
     * \brief The main loop of this node
     */
    int run() {
        try {
            ros::Time lastLogTime;
            int lastLogFrames = 0;

            AsyncTransfer asyncTransfer(useTcp ? ImageTransfer::TCP_CLIENT : ImageTransfer::UDP,
                remoteHost.c_str(), remotePort.c_str(), localHost.c_str(), localPort.c_str());

            while(ros::ok()) {
                // Receive image data
                ImagePair imagePair;
                if(!asyncTransfer.collectReceivedImagePair(imagePair, 0.5)) {
                    continue;
                }

                ros::Time stamp = ros::Time::now();

                // Publish the selected messages
                if(leftImagePublisher->getNumSubscribers() > 0) {
                    publishImageMsg(imagePair, stamp);
                }

                if(disparityPublisher->getNumSubscribers() > 0 || rightImagePublisher->getNumSubscribers() > 0) {
                    publishDispMapMsg(imagePair, stamp);
                }

                if(cloudPublisher->getNumSubscribers() > 0) {
                    if(recon3d == nullptr) {
                        // First initialize
                        initPointCloud();
                    }

                    publishPointCloudMsg(imagePair, stamp);
                }

                if(cameraInfoPublisher != NULL && cameraInfoPublisher->getNumSubscribers() > 0) {
                    publishCameraInfo(stamp, imagePair);
                }

                // Display some simple statistics
                frameNum++;
                if(stamp.sec != lastLogTime.sec) {
                    if(lastLogTime != ros::Time()) {
                        double dt = (stamp - lastLogTime).toSec();
                        double fps = (frameNum - lastLogFrames) / dt;
                        ROS_INFO("%.1f fps", fps);
                    }
                    lastLogFrames = frameNum;
                    lastLogTime = stamp;
                }
            }
        } catch(const std::exception& ex) {
            ROS_FATAL("Exception occured: %s", ex.what());
        }
    }

private:
    // ROS related objects
    ros::NodeHandle nh;
    boost::scoped_ptr<ros::Publisher> cloudPublisher;
    boost::scoped_ptr<ros::Publisher> disparityPublisher;
    boost::scoped_ptr<ros::Publisher> leftImagePublisher;
    boost::scoped_ptr<ros::Publisher> rightImagePublisher;
    boost::scoped_ptr<ros::Publisher> cameraInfoPublisher;

    // Parameters
    bool intensityChannel;
    bool useTcp;
    bool colorCodeDispMap;
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
    boost::scoped_ptr<Reconstruct3D> recon3d;
    boost::scoped_ptr<ColorCoder> colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    sensor_msgs::PointCloud2Ptr pointCloudMsg;
    cv::FileStorage calibStorage;
    nerian_stereo::StereoCameraInfoPtr camInfoMsg;
    ros::Time lastCamInfoPublish;

    /**
     * \brief Loads a camera calibration file if configured
     */
    void loadCameraCalibration() {
        if(calibFile == "" ) {
            ROS_WARN("No camera calibration file configured. Cannot publish detailed camera information!");
        } else {
            bool success = false;
            try {
                if (calibStorage.open(calibFile, cv::FileStorage::READ)) {
                    success = true;
                }
            } catch(...) {
            }

            if(!success) {
                ROS_WARN("Error reading calibration file: %s\n"
                    "Cannot publish detailed camera information!", calibFile.c_str());
            }
        }
    }

    /**
     * \brief Publishes a rectified left camera image
     */
    void publishImageMsg(const ImagePair& imagePair, ros::Time stamp) {
        cv_bridge::CvImage cvImg;
        cvImg.header.frame_id = frame;
        cvImg.header.stamp = stamp;
        cvImg.header.seq = imagePair.getSequenceNumber(); // Actually ROS will overwrite this

        cvImg.image = cv::Mat_<unsigned char>(imagePair.getHeight(),
            imagePair.getWidth(), imagePair.getPixelData(0), imagePair.getRowStride(0));
        sensor_msgs::ImagePtr msg = cvImg.toImageMsg();


        msg->encoding = "mono8";
        leftImagePublisher->publish(msg);
    }

    /**
     * \brief Publishes the disparity map as 16-bit grayscale image or color coded
     * RGB image
     */
    void publishDispMapMsg(const ImagePair& imagePair, ros::Time stamp) {
        cv_bridge::CvImage cvImg;
        cvImg.header.frame_id = frame;
        cvImg.header.stamp = stamp;
        cvImg.header.seq = imagePair.getSequenceNumber(); // Actually ROS will overwrite this

        bool format12Bit = (imagePair.getPixelFormat(1) == ImagePair::FORMAT_12_BIT);
        cv::Mat monoImg(imagePair.getHeight(), imagePair.getWidth(),
            format12Bit ? CV_16UC1 : CV_8UC1,
            imagePair.getPixelData(1), imagePair.getRowStride(1));
        string encoding = "";

        if(!colorCodeDispMap || !format12Bit) {
            cvImg.image = monoImg;
            encoding = (format12Bit ? "mono16": "mono8");
        } else {
            if(colCoder == NULL) {
                colCoder.reset(new ColorCoder(ColorCoder::COLOR_RED_BLUE_BGR, 0, 16*111, true, true));
                if(colorCodeLegend) {
                    // Create the legend
                    colDispMap = colCoder->createLegendBorder(monoImg.cols, monoImg.rows, 1.0/16.0);
                } else {
                    colDispMap = cv::Mat_<cv::Vec3b>(monoImg.rows, monoImg.cols);
                }
            }

            cv::Mat_<cv::Vec3b> dispSection = colDispMap(cv::Rect(0, 0, monoImg.cols, monoImg.rows));

            colCoder->codeImage(cv::Mat_<unsigned short>(monoImg), dispSection);
            cvImg.image = colDispMap;
            encoding = "bgr8";
        }

        sensor_msgs::ImagePtr msg = cvImg.toImageMsg();
        msg->encoding = encoding;

        if(disparityPublisher->getNumSubscribers() > 0 && format12Bit) {
            disparityPublisher->publish(msg);
        }
        if(rightImagePublisher->getNumSubscribers() > 0 && !format12Bit) {
            rightImagePublisher->publish(msg);
        }
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
    void publishPointCloudMsg(ImagePair& imagePair, ros::Time stamp) {
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
        pointCloudMsg->header.seq = imagePair.getSequenceNumber(); // Actually ROS will overwrite this

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
        ros::NodeHandle privateNh("~");

        // Initialize 3D reconstruction class
        recon3d.reset(new Reconstruct3D);

        // Initialize message
        pointCloudMsg.reset(new sensor_msgs::PointCloud2);

        // Set channel information.
        sensor_msgs::PointField fieldX;
        fieldX.name ="x";
        fieldX.offset = 0;
        fieldX.datatype = sensor_msgs::PointField::FLOAT32;
        fieldX.count = 1;
        pointCloudMsg->fields.push_back(fieldX);

        sensor_msgs::PointField fieldY;
        fieldY.name ="y";
        fieldY.offset = sizeof(float);
        fieldY.datatype = sensor_msgs::PointField::FLOAT32;
        fieldY.count = 1;
        pointCloudMsg->fields.push_back(fieldY);

        sensor_msgs::PointField fieldZ;
        fieldZ.name ="z";
        fieldZ.offset = 2*sizeof(float);
        fieldZ.datatype = sensor_msgs::PointField::FLOAT32;
        fieldZ.count = 1;
        pointCloudMsg->fields.push_back(fieldZ);

        if(intensityChannel) {
            sensor_msgs::PointField fieldI;
            fieldI.name ="intensity";
            fieldI.offset = 3*sizeof(float);
            fieldI.datatype = sensor_msgs::PointField::UINT8;
            fieldI.count = 1;
            pointCloudMsg->fields.push_back(fieldI);
        }
    }

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(ros::Time stamp, const ImagePair& imagePair) {
        if(camInfoMsg == NULL) {
            // Initialize the camera info structure
            camInfoMsg.reset(new nerian_stereo::StereoCameraInfo);

            camInfoMsg->header.frame_id = frame;
            camInfoMsg->header.seq = imagePair.getSequenceNumber(); // Actually ROS will overwrite this

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
                calibStorage["D1"] >> camInfoMsg->left_info.D;
                readCalibrationArray("M1", camInfoMsg->left_info.K);
                readCalibrationArray("R1", camInfoMsg->left_info.R);
                readCalibrationArray("P1", camInfoMsg->left_info.P);
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
                calibStorage["D2"] >> camInfoMsg->right_info.D;
                readCalibrationArray("M2", camInfoMsg->right_info.K);
                readCalibrationArray("R2", camInfoMsg->right_info.R);
                readCalibrationArray("P2", camInfoMsg->right_info.P);
                camInfoMsg->right_info.binning_x = 1;
                camInfoMsg->right_info.binning_y = 1;
                camInfoMsg->right_info.roi.do_rectify = false;
                camInfoMsg->right_info.roi.height = 0;
                camInfoMsg->right_info.roi.width = 0;
                camInfoMsg->right_info.roi.x_offset = 0;
                camInfoMsg->right_info.roi.y_offset = 0;

                readCalibrationArray("Q", camInfoMsg->Q);
                readCalibrationArray("T", camInfoMsg->T_left_right);
                readCalibrationArray("R", camInfoMsg->R_left_right);
            }
        }

        double dt = (stamp - lastCamInfoPublish).toSec();
        if(dt > 1.0) {
            // Rather use the Q-matrix that we received over the network if it is valid
            const float* qMatrix = imagePair.getQMatrix();
            if(qMatrix[0] != 0.0) {
                for(int i=0; i<16; i++) {
                    camInfoMsg->Q[i] = static_cast<double>(qMatrix[i]);
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
        ros::init(argc, argv, "nerian_stereo");
        StereoNode node;
        node.init();
        return node.run();
    } catch(const std::exception& ex) {
        ROS_FATAL("Exception occured: %s", ex.what());
        return 1;
    }
}
