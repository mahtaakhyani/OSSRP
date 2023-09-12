// Auto-generated. Do not edit!

// (in-package infrastructure.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

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
    bufferOffset = _arraySerializer.string(obj.face, buffer, bufferOffset, null);
    // Serialize message field [left_hand]
    bufferOffset = _arraySerializer.string(obj.left_hand, buffer, bufferOffset, null);
    // Serialize message field [right_hand]
    bufferOffset = _arraySerializer.string(obj.right_hand, buffer, bufferOffset, null);
    // Serialize message field [pose]
    bufferOffset = _arraySerializer.string(obj.pose, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Landmarks
    let len;
    let data = new Landmarks(null);
    // Deserialize message field [face]
    data.face = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [left_hand]
    data.left_hand = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [right_hand]
    data.right_hand = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [pose]
    data.pose = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.face.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.left_hand.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.right_hand.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.pose.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'infrastructure/Landmarks';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6571e14d514c06424ea80b7c30f04d5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] face
    string[] left_hand
    string[] right_hand
    string[] pose
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Landmarks(null);
    if (msg.face !== undefined) {
      resolved.face = msg.face;
    }
    else {
      resolved.face = []
    }

    if (msg.left_hand !== undefined) {
      resolved.left_hand = msg.left_hand;
    }
    else {
      resolved.left_hand = []
    }

    if (msg.right_hand !== undefined) {
      resolved.right_hand = msg.right_hand;
    }
    else {
      resolved.right_hand = []
    }

    if (msg.pose !== undefined) {
      resolved.pose = msg.pose;
    }
    else {
      resolved.pose = []
    }

    return resolved;
    }
};

module.exports = Landmarks;
