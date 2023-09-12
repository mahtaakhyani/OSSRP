// Auto-generated. Do not edit!

// (in-package infrastructure.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let List = require('../msg/List.js');
let Landmarks = require('../msg/Landmarks.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GazeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame = null;
      this.landmark = null;
    }
    else {
      if (initObj.hasOwnProperty('frame')) {
        this.frame = initObj.frame
      }
      else {
        this.frame = new List();
      }
      if (initObj.hasOwnProperty('landmark')) {
        this.landmark = initObj.landmark
      }
      else {
        this.landmark = new Landmarks();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GazeRequest
    // Serialize message field [frame]
    bufferOffset = List.serialize(obj.frame, buffer, bufferOffset);
    // Serialize message field [landmark]
    bufferOffset = Landmarks.serialize(obj.landmark, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GazeRequest
    let len;
    let data = new GazeRequest(null);
    // Deserialize message field [frame]
    data.frame = List.deserialize(buffer, bufferOffset);
    // Deserialize message field [landmark]
    data.landmark = Landmarks.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += List.getMessageSize(object.frame);
    length += Landmarks.getMessageSize(object.landmark);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/GazeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '563138297254f9391391e1e861953236';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    infrastructure/List  frame
    infrastructure/Landmarks landmark
    
    ================================================================================
    MSG: infrastructure/List
    infrastructure/Array3D[] list
    ================================================================================
    MSG: infrastructure/Array3D
    float64[] data
    
    ================================================================================
    MSG: infrastructure/Landmarks
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
    const resolved = new GazeRequest(null);
    if (msg.frame !== undefined) {
      resolved.frame = List.Resolve(msg.frame)
    }
    else {
      resolved.frame = new List()
    }

    if (msg.landmark !== undefined) {
      resolved.landmark = Landmarks.Resolve(msg.landmark)
    }
    else {
      resolved.landmark = new Landmarks()
    }

    return resolved;
    }
};

class GazeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gazedirection = null;
    }
    else {
      if (initObj.hasOwnProperty('gazedirection')) {
        this.gazedirection = initObj.gazedirection
      }
      else {
        this.gazedirection = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GazeResponse
    // Serialize message field [gazedirection]
    bufferOffset = _serializer.string(obj.gazedirection, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GazeResponse
    let len;
    let data = new GazeResponse(null);
    // Deserialize message field [gazedirection]
    data.gazedirection = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.gazedirection);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/GazeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60309a766d9048157607b5f232e2ee59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string    gazedirection
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GazeResponse(null);
    if (msg.gazedirection !== undefined) {
      resolved.gazedirection = msg.gazedirection;
    }
    else {
      resolved.gazedirection = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GazeRequest,
  Response: GazeResponse,
  md5sum() { return '205259182beb600563ab418d4957eb6b'; },
  datatype() { return 'infrastructure/Gaze'; }
};
