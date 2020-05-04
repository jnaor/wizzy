// Auto-generated. Do not edit!

// (in-package wizzybug_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let obstacle = require('./obstacle.js');
let ttc = require('./ttc.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ChairState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.obstacles = null;
      this.state = null;
      this.ttc_msg = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('obstacles')) {
        this.obstacles = initObj.obstacles
      }
      else {
        this.obstacles = [];
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('ttc_msg')) {
        this.ttc_msg = initObj.ttc_msg
      }
      else {
        this.ttc_msg = new ttc();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChairState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [obstacles]
    // Serialize the length for message field [obstacles]
    bufferOffset = _serializer.uint32(obj.obstacles.length, buffer, bufferOffset);
    obj.obstacles.forEach((val) => {
      bufferOffset = obstacle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [state]
    bufferOffset = std_msgs.msg.String.serialize(obj.state, buffer, bufferOffset);
    // Serialize message field [ttc_msg]
    bufferOffset = ttc.serialize(obj.ttc_msg, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChairState
    let len;
    let data = new ChairState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacles]
    // Deserialize array length for message field [obstacles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacles[i] = obstacle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [state]
    data.state = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [ttc_msg]
    data.ttc_msg = ttc.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.obstacles.forEach((val) => {
      length += obstacle.getMessageSize(val);
    });
    length += std_msgs.msg.String.getMessageSize(object.state);
    length += ttc.getMessageSize(object.ttc_msg);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wizzybug_msgs/ChairState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '80026f80fdf5c7c2e3ba8933871ba1e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    wizzybug_msgs/obstacle[] obstacles
    std_msgs/String state
    wizzybug_msgs/ttc ttc_msg
    
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
    
    ================================================================================
    MSG: wizzybug_msgs/obstacle
    std_msgs/Float64 x
    std_msgs/Float64 y
    std_msgs/Float64 z
    std_msgs/Float64 width
    std_msgs/Float64 height
    std_msgs/Float64 length
    
    std_msgs/String classification
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: wizzybug_msgs/ttc
    Header header
    
    float32 ttc
    float32 ttc_azimuth
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChairState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.obstacles !== undefined) {
      resolved.obstacles = new Array(msg.obstacles.length);
      for (let i = 0; i < resolved.obstacles.length; ++i) {
        resolved.obstacles[i] = obstacle.Resolve(msg.obstacles[i]);
      }
    }
    else {
      resolved.obstacles = []
    }

    if (msg.state !== undefined) {
      resolved.state = std_msgs.msg.String.Resolve(msg.state)
    }
    else {
      resolved.state = new std_msgs.msg.String()
    }

    if (msg.ttc_msg !== undefined) {
      resolved.ttc_msg = ttc.Resolve(msg.ttc_msg)
    }
    else {
      resolved.ttc_msg = new ttc()
    }

    return resolved;
    }
};

module.exports = ChairState;
