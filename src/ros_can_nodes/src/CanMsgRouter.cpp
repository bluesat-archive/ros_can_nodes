/*
 * Date Started: 29/09/2017
 * Original Author: Simon Ireland
 * Editors:
 * ROS Node Name:
 * ROS Package: ros_can_nodes
 * Purpose:
 * This code is released under the MIT  License. Copyright BLUEsat UNSW, 201
 */

typedef uint32_t canid_t;

 /**
  * struct can_frame - basic CAN frame structure
  * @can_id:  CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
  * @can_dlc: frame payload length in byte (0 .. 8) aka data length code
  *           N.B. the DLC field from ISO 11898-1 Chapter 8.4.2.3 has a 1:1
  *           mapping of the 'data length code' to the real payload length
  * @__pad:   padding
  * @__res0:  reserved / padding
  * @__res1:  reserved / padding
  * @data:    CAN frame payload (up to 8 byte)
  */
 struct can_frame {
 	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
 	uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
 	uint8_t __pad;   /* padding */
 	uint8_t __res0;  /* reserved / padding */
 	uint8_t __res1;  /* reserved / padding */
 	uint8_t data[CAN_MAX_DLEN] __attribute__((aligned(8)));
 };

static void CANMsgRouter::processCANMsg(CANMsg msg){

}

static void CANMsgRouter::routeControlMsg(uint32_t identifier, uint8_t *data);

static void CANMsgRouter::routePublishMsg(uint32_t identifier, uint8_t *data);
