// Auto-generated. Do not edit!

// (in-package mvr_robot_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObserveData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.imu_angular_vel = null;
      this.joint_pos = null;
      this.joint_vel = null;
      this.commands = null;
      this.quat_float = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('imu_angular_vel')) {
        this.imu_angular_vel = initObj.imu_angular_vel
      }
      else {
        this.imu_angular_vel = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('joint_pos')) {
        this.joint_pos = initObj.joint_pos
      }
      else {
        this.joint_pos = new Array(1).fill(0);
      }
      if (initObj.hasOwnProperty('joint_vel')) {
        this.joint_vel = initObj.joint_vel
      }
      else {
        this.joint_vel = new Array(1).fill(0);
      }
      if (initObj.hasOwnProperty('commands')) {
        this.commands = initObj.commands
      }
      else {
        this.commands = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('quat_float')) {
        this.quat_float = initObj.quat_float
      }
      else {
        this.quat_float = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObserveData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [imu_angular_vel] has the right length
    if (obj.imu_angular_vel.length !== 3) {
      throw new Error('Unable to serialize array field imu_angular_vel - length must be 3')
    }
    // Serialize message field [imu_angular_vel]
    bufferOffset = _arraySerializer.float32(obj.imu_angular_vel, buffer, bufferOffset, 3);
    // Check that the constant length array field [joint_pos] has the right length
    if (obj.joint_pos.length !== 1) {
      throw new Error('Unable to serialize array field joint_pos - length must be 1')
    }
    // Serialize message field [joint_pos]
    bufferOffset = _arraySerializer.float32(obj.joint_pos, buffer, bufferOffset, 1);
    // Check that the constant length array field [joint_vel] has the right length
    if (obj.joint_vel.length !== 1) {
      throw new Error('Unable to serialize array field joint_vel - length must be 1')
    }
    // Serialize message field [joint_vel]
    bufferOffset = _arraySerializer.float32(obj.joint_vel, buffer, bufferOffset, 1);
    // Check that the constant length array field [commands] has the right length
    if (obj.commands.length !== 3) {
      throw new Error('Unable to serialize array field commands - length must be 3')
    }
    // Serialize message field [commands]
    bufferOffset = _arraySerializer.float32(obj.commands, buffer, bufferOffset, 3);
    // Check that the constant length array field [quat_float] has the right length
    if (obj.quat_float.length !== 4) {
      throw new Error('Unable to serialize array field quat_float - length must be 4')
    }
    // Serialize message field [quat_float]
    bufferOffset = _arraySerializer.float32(obj.quat_float, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObserveData
    let len;
    let data = new ObserveData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu_angular_vel]
    data.imu_angular_vel = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [joint_pos]
    data.joint_pos = _arrayDeserializer.float32(buffer, bufferOffset, 1)
    // Deserialize message field [joint_vel]
    data.joint_vel = _arrayDeserializer.float32(buffer, bufferOffset, 1)
    // Deserialize message field [commands]
    data.commands = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [quat_float]
    data.quat_float = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mvr_robot_control/ObserveData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50eb62f4305c2c959b5563826e09f256';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
        float32[3] imu_angular_vel
        float32[1] joint_pos
        float32[1] joint_vel
        float32[3] commands
        float32[4] quat_float
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObserveData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.imu_angular_vel !== undefined) {
      resolved.imu_angular_vel = msg.imu_angular_vel;
    }
    else {
      resolved.imu_angular_vel = new Array(3).fill(0)
    }

    if (msg.joint_pos !== undefined) {
      resolved.joint_pos = msg.joint_pos;
    }
    else {
      resolved.joint_pos = new Array(1).fill(0)
    }

    if (msg.joint_vel !== undefined) {
      resolved.joint_vel = msg.joint_vel;
    }
    else {
      resolved.joint_vel = new Array(1).fill(0)
    }

    if (msg.commands !== undefined) {
      resolved.commands = msg.commands;
    }
    else {
      resolved.commands = new Array(3).fill(0)
    }

    if (msg.quat_float !== undefined) {
      resolved.quat_float = msg.quat_float;
    }
    else {
      resolved.quat_float = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = ObserveData;
