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

class obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.width = null;
      this.height = null;
      this.length = null;
      this.classification = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('classification')) {
        this.classification = initObj.classification
      }
      else {
        this.classification = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obstacle
    // Serialize message field [x]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.z, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.width, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.height, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.length, buffer, bufferOffset);
    // Serialize message field [classification]
    bufferOffset = std_msgs.msg.String.serialize(obj.classification, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obstacle
    let len;
    let data = new obstacle(null);
    // Deserialize message field [x]
    data.x = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [classification]
    data.classification = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.classification);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wizzybug_msgs/obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8d2575537d9593df2bf86f5b4318f6e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obstacle(null);
    if (msg.x !== undefined) {
      resolved.x = std_msgs.msg.Float64.Resolve(msg.x)
    }
    else {
      resolved.x = new std_msgs.msg.Float64()
    }

    if (msg.y !== undefined) {
      resolved.y = std_msgs.msg.Float64.Resolve(msg.y)
    }
    else {
      resolved.y = new std_msgs.msg.Float64()
    }

    if (msg.z !== undefined) {
      resolved.z = std_msgs.msg.Float64.Resolve(msg.z)
    }
    else {
      resolved.z = new std_msgs.msg.Float64()
    }

    if (msg.width !== undefined) {
      resolved.width = std_msgs.msg.Float64.Resolve(msg.width)
    }
    else {
      resolved.width = new std_msgs.msg.Float64()
    }

    if (msg.height !== undefined) {
      resolved.height = std_msgs.msg.Float64.Resolve(msg.height)
    }
    else {
      resolved.height = new std_msgs.msg.Float64()
    }

    if (msg.length !== undefined) {
      resolved.length = std_msgs.msg.Float64.Resolve(msg.length)
    }
    else {
      resolved.length = new std_msgs.msg.Float64()
    }

    if (msg.classification !== undefined) {
      resolved.classification = std_msgs.msg.String.Resolve(msg.classification)
    }
    else {
      resolved.classification = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = obstacle;
