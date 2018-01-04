/*
 * Date Started: 20/10/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 2017
 */

 #include "ROSCANConstants.hpp"
 #include "TopicBuffers.hpp"

static void TopicBuffers::initBuffers(){

}

static void TopicBuffers::processData(int key, uint8_t* data, int d_len, bool last_msg){

    TopicBuffers::appendData(data);

    if(last_msg){
        // TODO: Convert to flattype and publish to ros network
    }
}


static void TopicBuffers::appendData(uint8_t* data){

}
