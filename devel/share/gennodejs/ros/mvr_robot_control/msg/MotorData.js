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

class MotorData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.kp_ = null;
      this.kd_ = null;
      this.ff_ = null;
      this.pos_des_ = null;
      this.vel_des_ = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('kp_')) {
        this.kp_ = initObj.kp_
      }
      else {
        this.kp_ = 0.0;
      }
      if (initObj.hasOwnProperty('kd_')) {
        this.kd_ = initObj.kd_
      }
      else {
        this.kd_ = 0.0;
      }
      if (initObj.hasOwnProperty('ff_')) {
        this.ff_ = initObj.ff_
      }
      else {
        this.ff_ = 0.0;
      }
      if (initObj.hasOwnProperty('pos_des_')) {
        this.pos_des_ = initObj.pos_des_
      }
      else {
        this.pos_des_ = 0.0;
      }
      if (initObj.hasOwnProperty('vel_des_')) {
        this.vel_des_ = initObj.vel_des_
      }
      else {
        this.vel_des_ = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int16(obj.id, buffer, bufferOffset);
    // Serialize message field [kp_]
    bufferOffset = _serializer.float32(obj.kp_, buffer, bufferOffset);
    // Serialize message field [kd_]
    bufferOffset = _serializer.float32(obj.kd_, buffer, bufferOffset);
    // Serialize message field [ff_]
    bufferOffset = _serializer.float32(obj.ff_, buffer, bufferOffset);
    // Serialize message field [pos_des_]
    bufferOffset = _serializer.float32(obj.pos_des_, buffer, bufferOffset);
    // Serialize message field [vel_des_]
    bufferOffset = _serializer.float32(obj.vel_des_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorData
    let len;
    let data = new MotorData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [kp_]
    data.kp_ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kd_]
    data.kd_ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ff_]
    data.ff_ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pos_des_]
    data.pos_des_ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vel_des_]
    data.vel_des_ = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mvr_robot_control/MotorData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6bed2913051bec1ee3cdf057ad08a302';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
        int16 id
        float32 kp_
        float32 kd_
        float32 ff_
        float32 pos_des_ 
        float32 vel_des_
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
    const resolved = new MotorData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.kp_ !== undefined) {
      resolved.kp_ = msg.kp_;
    }
    else {
      resolved.kp_ = 0.0
    }

    if (msg.kd_ !== undefined) {
      resolved.kd_ = msg.kd_;
    }
    else {
      resolved.kd_ = 0.0
    }

    if (msg.ff_ !== undefined) {
      resolved.ff_ = msg.ff_;
    }
    else {
      resolved.ff_ = 0.0
    }

    if (msg.pos_des_ !== undefined) {
      resolved.pos_des_ = msg.pos_des_;
    }
    else {
      resolved.pos_des_ = 0.0
    }

    if (msg.vel_des_ !== undefined) {
      resolved.vel_des_ = msg.vel_des_;
    }
    else {
      resolved.vel_des_ = 0.0
    }

    return resolved;
    }
};

module.exports = MotorData;
