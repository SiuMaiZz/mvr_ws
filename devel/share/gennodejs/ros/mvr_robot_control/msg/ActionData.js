// Auto-generated. Do not edit!

// (in-package mvr_robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ActionData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_pos')) {
        this.joint_pos = initObj.joint_pos
      }
      else {
        this.joint_pos = new Array(1).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionData
    // Check that the constant length array field [joint_pos] has the right length
    if (obj.joint_pos.length !== 1) {
      throw new Error('Unable to serialize array field joint_pos - length must be 1')
    }
    // Serialize message field [joint_pos]
    bufferOffset = _arraySerializer.float32(obj.joint_pos, buffer, bufferOffset, 1);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionData
    let len;
    let data = new ActionData(null);
    // Deserialize message field [joint_pos]
    data.joint_pos = _arrayDeserializer.float32(buffer, bufferOffset, 1)
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mvr_robot_control/ActionData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a07255ea0da1785d1c58f5945b919bcb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[1] joint_pos
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionData(null);
    if (msg.joint_pos !== undefined) {
      resolved.joint_pos = msg.joint_pos;
    }
    else {
      resolved.joint_pos = new Array(1).fill(0)
    }

    return resolved;
    }
};

module.exports = ActionData;
