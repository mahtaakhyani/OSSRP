// Auto-generated. Do not edit!

// (in-package infrastructure.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Landmarks {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.face = null;
      this.left_hand = null;
      this.right_hand = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('face')) {
        this.face = initObj.face
      }
      else {
        this.face = [];
      }
      if (initObj.hasOwnProperty('left_hand')) {
        this.left_hand = initObj.left_hand
      }
      else {
        this.left_hand = [];
      }
      if (initObj.hasOwnProperty('right_hand')) {
        this.right_hand = initObj.right_hand
      }
      else {
        this.right_hand = [];
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Landmarks
    // Serialize message field [face]
    // Serialize the length for message field [face]
    bufferOffset = _serializer.uint32(obj.face.length, buffer, bufferOffset);
    obj.face.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [left_hand]
    // Serialize the length for message field [left_hand]
    bufferOffset = _serializer.uint32(obj.left_hand.length, buffer, bufferOffset);
    obj.left_hand.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [right_hand]
    // Serialize the length for message field [right_hand]
    bufferOffset = _serializer.uint32(obj.right_hand.length, buffer, bufferOffset);
    obj.right_hand.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [pose]
    // Serialize the length for message field [pose]
    bufferOffset = _serializer.uint32(obj.pose.length, buffer, bufferOffset);
    obj.pose.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Landmarks
    let len;
    let data = new Landmarks(null);
    // Deserialize message field [face]
    // Deserialize array length for message field [face]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.face = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.face[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [left_hand]
    // Deserialize array length for message field [left_hand]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.left_hand = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.left_hand[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [right_hand]
    // Deserialize array length for message field [right_hand]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.right_hand = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.right_hand[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [pose]
    // Deserialize array length for message field [pose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pose[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.face.length;
    length += 24 * object.left_hand.length;
    length += 24 * object.right_hand.length;
    length += 24 * object.pose.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'infrastructure/Landmarks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '58ff724fd227fb57ee478929cb224b52';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] face
    geometry_msgs/Point[] left_hand
    geometry_msgs/Point[] right_hand
    geometry_msgs/Point[] pose
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Landmarks(null);
    if (msg.face !== undefined) {
      resolved.face = new Array(msg.face.length);
      for (let i = 0; i < resolved.face.length; ++i) {
        resolved.face[i] = geometry_msgs.msg.Point.Resolve(msg.face[i]);
      }
    }
    else {
      resolved.face = []
    }

    if (msg.left_hand !== undefined) {
      resolved.left_hand = new Array(msg.left_hand.length);
      for (let i = 0; i < resolved.left_hand.length; ++i) {
        resolved.left_hand[i] = geometry_msgs.msg.Point.Resolve(msg.left_hand[i]);
      }
    }
    else {
      resolved.left_hand = []
    }

    if (msg.right_hand !== undefined) {
      resolved.right_hand = new Array(msg.right_hand.length);
      for (let i = 0; i < resolved.right_hand.length; ++i) {
        resolved.right_hand[i] = geometry_msgs.msg.Point.Resolve(msg.right_hand[i]);
      }
    }
    else {
      resolved.right_hand = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = new Array(msg.pose.length);
      for (let i = 0; i < resolved.pose.length; ++i) {
        resolved.pose[i] = geometry_msgs.msg.Point.Resolve(msg.pose[i]);
      }
    }
    else {
      resolved.pose = []
    }

    return resolved;
    }
};

module.exports = Landmarks;
