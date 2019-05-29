/*******************************************************************************
 * Copyright (c) 2019 Nerian Vision GmbH
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

#ifndef __NERIAN_STEREO_NODE_H__
#define __NERIAN_STEREO_NODE_H__

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

#include <dynamic_reconfigure/server.h>
#include <nerian_stereo/NerianStereoConfig.h>
#include <visiontransfer/scenescanparameters.h>

using namespace std;
using namespace visiontransfer;

/**
 * \brief A driver node that receives data from SceneScan/SP1 and forwards
 * it to ROS.
 *
 * SceneScan and SP1 by Nerian Vision GmbH are hardware systems for
 * real-time stereo vision. They transmit a computed disparity map (an
 * inverse depth map) through gigabit ethernet, which is then received by
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

namespace nerian_stereo {

class StereoNodeBase {
public:
    StereoNodeBase(): frameNum(0), initialConfigReceived(false) {
    }

    ~StereoNodeBase() {
    }

    /*
     * \brief Callback that receives an updated configuration from ROS
     */
    void dynamicReconfigureCallback(nerian_stereo::NerianStereoConfig &config, uint32_t level);
    void updateParameterServerFromDevice(std::map<std::string, ParameterInfo>& cfg);
    void updateConfigFromDevice(std::map<std::string, ParameterInfo>& cfg);
    
    /*
     * \brief Initialize and publish configuration with a dynamic_reconfigure server
     */
    void initDynamicReconfigure();

    /**
     * \brief Performs general initializations
     */
    void init();

    void prepareAsyncTransfer();

    void processOneImagePair();

private:
    enum PointCloudColorMode {
        RGB_SEPARATE,
        RGB_COMBINED,
        INTENSITY,
        NONE
    };

    virtual ros::NodeHandle& getNH() = 0;
    virtual ros::NodeHandle& getPrivateNH() = 0;

    //
    boost::scoped_ptr<ros::Publisher> cloudPublisher;
    boost::scoped_ptr<ros::Publisher> disparityPublisher;
    boost::scoped_ptr<ros::Publisher> leftImagePublisher;
    boost::scoped_ptr<ros::Publisher> rightImagePublisher;
    boost::scoped_ptr<ros::Publisher> cameraInfoPublisher;

    // ROS dynamic_reconfigure
    boost::shared_ptr<dynamic_reconfigure::Server<nerian_stereo::NerianStereoConfig>> dynReconfServer;
    nerian_stereo::NerianStereoConfig lastKnownConfig;
    bool initialConfigReceived;
    
    // Connection to parameter server on device
    boost::shared_ptr<SceneScanParameters> sceneScanParameters;

    // Parameters
    bool useTcp;
    std::string colorCodeDispMap;
    bool colorCodeLegend;
    bool rosCoordinateSystem;
    bool rosTimestamps;
    std::string remotePort;
    std::string frame;
    std::string remoteHost;
    std::string calibFile;
    double execDelay;
    double maxDepth;
    bool useQFromCalibFile;
    PointCloudColorMode pointCloudColorMode;

    // Other members
    int frameNum;
    boost::scoped_ptr<Reconstruct3D> recon3d;
    boost::scoped_ptr<ColorCoder> colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    sensor_msgs::PointCloud2Ptr pointCloudMsg;
    cv::FileStorage calibStorage;
    nerian_stereo::StereoCameraInfoPtr camInfoMsg;
    ros::Time lastCamInfoPublish;

    boost::scoped_ptr<AsyncTransfer> asyncTransfer;
    ros::Time lastLogTime;
    int lastLogFrames = 0;


    /**
     * \brief Loads a camera calibration file if configured
     */
    void loadCameraCalibration();

    /**
     * \brief Publishes the disparity map as 16-bit grayscale image or color coded
     * RGB image
     */
    void publishImageMsg(const ImagePair& imagePair, int imageIndex, ros::Time stamp, bool allowColorCode,
            ros::Publisher* publisher);

    /**
     * \brief Transform Q matrix to match the ROS coordinate system:
     * Swap y/z axis, then swap x/y axis, then invert y and z axis.
     */
    void qMatrixToRosCoords(const float* src, float* dst);

    /**
     * \brief Reconstructs the 3D locations form the disparity map and publishes them
     * as point cloud.
     */
    void publishPointCloudMsg(ImagePair& imagePair, ros::Time stamp);

    /**
     * \brief Copies the intensity or RGB data to the point cloud
     */
    template <PointCloudColorMode colorMode> void copyPointCloudIntensity(ImagePair& imagePair) {
        // Get pointers to the beginnig and end of the point cloud
        unsigned char* cloudStart = &pointCloudMsg->data[0];
        unsigned char* cloudEnd = &pointCloudMsg->data[0]
            + imagePair.getWidth()*imagePair.getHeight()*4*sizeof(float);

        if(imagePair.getPixelFormat(0) == ImagePair::FORMAT_8_BIT_MONO) {
            // Get pointer to the current pixel and end of current row
            unsigned char* imagePtr = imagePair.getPixelData(0);
            unsigned char* rowEndPtr = imagePtr + imagePair.getWidth();
            int rowIncrement = imagePair.getRowStride(0) - imagePair.getWidth();

            for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                    cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {
                if(colorMode == RGB_SEPARATE) {// RGB as float
                    *reinterpret_cast<float*>(cloudPtr) = static_cast<float>(*imagePtr) / 255.0F;
                } else if(colorMode == RGB_COMBINED) {// RGB as integer
                    const unsigned char intensity = *imagePtr;
                    *reinterpret_cast<unsigned int*>(cloudPtr) = (intensity << 16) | (intensity << 8) | intensity;
                } else {
                    *cloudPtr = *imagePtr;
                }

                imagePtr++;
                if(imagePtr == rowEndPtr) {
                    // Progress to next row
                    imagePtr += rowIncrement;
                    rowEndPtr = imagePtr + imagePair.getWidth();
                }
            }
        } else if(imagePair.getPixelFormat(0) == ImagePair::FORMAT_12_BIT_MONO) {
            // Get pointer to the current pixel and end of current row
            unsigned short* imagePtr = reinterpret_cast<unsigned short*>(imagePair.getPixelData(0));
            unsigned short* rowEndPtr = imagePtr + imagePair.getWidth();
            int rowIncrement = imagePair.getRowStride(0) - 2*imagePair.getWidth();

            for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                    cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {

                if(colorMode == RGB_SEPARATE) {// RGB as float
                    *reinterpret_cast<float*>(cloudPtr) = static_cast<float>(*imagePtr) / 4095.0F;
                } else if(colorMode == RGB_COMBINED) {// RGB as integer
                    const unsigned char intensity = *imagePtr/16;
                    *reinterpret_cast<unsigned int*>(cloudPtr) = (intensity << 16) | (intensity << 8) | intensity;
                } else {
                    *cloudPtr = *imagePtr/16;
                }

                imagePtr++;
                if(imagePtr == rowEndPtr) {
                    // Progress to next row
                    imagePtr += rowIncrement;
                    rowEndPtr = imagePtr + imagePair.getWidth();
                }
            }
        } else if(imagePair.getPixelFormat(0) == ImagePair::FORMAT_8_BIT_RGB) {
            // Get pointer to the current pixel and end of current row
            unsigned char* imagePtr = imagePair.getPixelData(0);
            unsigned char* rowEndPtr = imagePtr + imagePair.getWidth();
            int rowIncrement = imagePair.getRowStride(0) - imagePair.getWidth();

            static bool warned = false;
            if(colorMode == RGB_SEPARATE && !warned) {
                warned = true;
                ROS_WARN("RGBF32 is not supported for color images. Please use RGB8!");
            }

            for(unsigned char* cloudPtr = cloudStart + 3*sizeof(float);
                    cloudPtr < cloudEnd; cloudPtr+= 4*sizeof(float)) {
                if(colorMode == RGB_SEPARATE) {// RGB as float
                    *reinterpret_cast<float*>(cloudPtr) = static_cast<float>(imagePtr[2]) / 255.0F;
                } else if(colorMode == RGB_COMBINED) {// RGB as integer
                    *reinterpret_cast<unsigned int*>(cloudPtr) = (imagePtr[0] << 16) | (imagePtr[1] << 8) | imagePtr[2];
                } else {
                    *cloudPtr = (imagePtr[0] + imagePtr[1]*2 + imagePtr[2])/4;
                }

                imagePtr+=3;
                if(imagePtr == rowEndPtr) {
                    // Progress to next row
                    imagePtr += rowIncrement;
                    rowEndPtr = imagePtr + imagePair.getWidth();
                }
            }
        } else {
            throw std::runtime_error("Invalid pixel format!");
        }
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
    void initPointCloud();

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(ros::Time stamp, const ImagePair& imagePair);

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

    // The following three implementations are autogenerated from the .cfg file
    void autogen_dynamicReconfigureCallback(nerian_stereo::NerianStereoConfig &config, uint32_t level);
    void autogen_updateParameterServerFromDevice(std::map<std::string, ParameterInfo>& cfg);
    void autogen_updateConfigFromDevice(std::map<std::string, ParameterInfo>& cfg);

};

} // namespace

#endif

