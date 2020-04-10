// Auto-generated. Do not edit!

// (in-package wizzybug_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class lidar_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.dist_to_pitfall = null;
      this.dist_to_obstacle = null;
      this.dist_to_floor = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('dist_to_pitfall')) {
        this.dist_to_pitfall = initObj.dist_to_pitfall
      }
      else {
        this.dist_to_pitfall = 0.0;
      }
      if (initObj.hasOwnProperty('dist_to_obstacle')) {
        this.dist_to_obstacle = initObj.dist_to_obstacle
      }
      else {
        this.dist_to_obstacle = 0.0;
      }
      if (initObj.hasOwnProperty('dist_to_floor')) {
        this.dist_to_floor = initObj.dist_to_floor
      }
      else {
        this.dist_to_floor = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lidar_data
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [dist_to_pitfall]
    bufferOffset = _serializer.float32(obj.dist_to_pitfall, buffer, bufferOffset);
    // Serialize message field [dist_to_obstacle]
    bufferOffset = _serializer.float32(obj.dist_to_obstacle, buffer, bufferOffset);
    // Serialize message field [dist_to_floor]
    bufferOffset = _serializer.float32(obj.dist_to_floor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lidar_data
    let len;
    let data = new lidar_data(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [dist_to_pitfall]
    data.dist_to_pitfall = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dist_to_obstacle]
    data.dist_to_obstacle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dist_to_floor]
    data.dist_to_floor = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wizzybug_msgs/lidar_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c19335d47678d872f794c1f089147d5f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32 dist_to_pitfall
    float32 dist_to_obstacle
    float32 dist_to_floor
    
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
    const resolved = new lidar_data(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.dist_to_pitfall !== undefined) {
      resolved.dist_to_pitfall = msg.dist_to_pitfall;
    }
    else {
      resolved.dist_to_pitfall = 0.0
    }

    if (msg.dist_to_obstacle !== undefined) {
      resolved.dist_to_obstacle = msg.dist_to_obstacle;
    }
    else {
      resolved.dist_to_obstacle = 0.0
    }

    if (msg.dist_to_floor !== undefined) {
      resolved.dist_to_floor = msg.dist_to_floor;
    }
    else {
      resolved.dist_to_floor = 0.0
    }

    return resolved;
    }
};

module.exports = lidar_data;
