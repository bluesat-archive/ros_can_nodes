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

static void initBuffers(){

}

static void processData(short key, uint8_t* data, int d_len, bool last_msg){

    TopicBuffers::appendData(key, data);
    int x = d_len;

    if(last_msg){
        // TODO: Convert to flattype and publish to ros network


    }
}


void TopicBuffers::appendData(short key, uint8_t* data){
    //TODO: append data
    topic_buffers[key] = data;
}
