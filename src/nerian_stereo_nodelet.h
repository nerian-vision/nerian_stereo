/*******************************************************************************
 * Copyright (c) 2019 Nerian Vision Technologies
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

#include <nodelet/nodelet.h>
#include "nerian_stereo_node.h"

namespace nerian_stereo {

class StereoNodelet: public StereoNode, public nodelet::Nodelet {
private:
    // The nodelet does not initialize its own node handles
    inline ros::NodeHandle& getNH() override { return nodelet::Nodelet::getNodeHandle(); }
    inline ros::NodeHandle& getPrivateNH() override { return nodelet::Nodelet::getPrivateNodeHandle(); }
    ros::Timer timer;
public:
    void stereoIteration(const ros::TimerEvent&) {
        processOneImagePair();
    }
    virtual void onInit() {
        StereoNode::init();
        StereoNode::initDynamicReconfigure();
        prepareAsyncTransfer();
        // 2kHz timer for lower latency (stereoIteration will then block)
        timer = getNH().createTimer(ros::Duration(0.0005), &StereoNodelet::stereoIteration, this);
    }
};

} // namespace

