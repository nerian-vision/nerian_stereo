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

#include "nerian_stereo_node.h"

class StereoNodeStandalone: public StereoNode {

private:
    // ROS related objects
    ros::NodeHandle nhInternal;
    ros::NodeHandle privateNhInternal;
    inline ros::NodeHandle& getNH() { return nhInternal; }
    inline ros::NodeHandle& getPrivateNH() { return privateNhInternal; }
public:
    StereoNodeStandalone(): privateNhInternal("~") {}
};


int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "nerian_stereo");
        StereoNodeStandalone node;
        node.init();
        node.initDynamicReconfigure();
        ROS_INFO("Run node ...");
        return node.run();
    } catch(const std::exception& ex) {
        ROS_FATAL("Exception occured: %s", ex.what());
        return 1;
    }
}

