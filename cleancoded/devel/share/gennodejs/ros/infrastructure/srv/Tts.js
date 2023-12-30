// Auto-generated. Do not edit!

// (in-package infrastructure.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let Tts_msg = require('../msg/Tts_msg.js');

//-----------------------------------------------------------

class TtsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TtsRequest
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TtsRequest
    let len;
    let data = new TtsRequest(null);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.text.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/TtsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74697ed3d931f6eede8bf3a8dfeca160';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TtsRequest(null);
    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

class TtsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speech = null;
    }
    else {
      if (initObj.hasOwnProperty('speech')) {
        this.speech = initObj.speech
      }
      else {
        this.speech = new Tts_msg();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TtsResponse
    // Serialize message field [speech]
    bufferOffset = Tts_msg.serialize(obj.speech, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TtsResponse
    let len;
    let data = new TtsResponse(null);
    // Deserialize message field [speech]
    data.speech = Tts_msg.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Tts_msg.getMessageSize(object.speech);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/TtsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2534803dfe8be7600047b56444e1d2de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    infrastructure/Tts_msg speech
    
    ================================================================================
    MSG: infrastructure/Tts_msg
    audio_common_msgs/AudioData[] data
    ================================================================================
    MSG: audio_common_msgs/AudioData
    uint8[] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TtsResponse(null);
    if (msg.speech !== undefined) {
      resolved.speech = Tts_msg.Resolve(msg.speech)
    }
    else {
      resolved.speech = new Tts_msg()
    }

    return resolved;
    }
};

module.exports = {
  Request: TtsRequest,
  Response: TtsResponse,
  md5sum() { return 'e8bbb23d743bc8a20e615c7ae7a757bd'; },
  datatype() { return 'infrastructure/Tts'; }
};
