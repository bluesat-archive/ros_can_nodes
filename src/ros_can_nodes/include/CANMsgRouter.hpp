/*
 * Date Started: 29/09/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 201
 */


class CANMsgRouter{

    public:

        static void processCANMsg(CANMsg msg);

    private:

        static void routeControlMsg(uint32_t identifier, uint8_t *data);

        static void routePublishMsg(uint32_t identifier, uint8_t *data);

        //TODO: abstract decoder jumptable
}
