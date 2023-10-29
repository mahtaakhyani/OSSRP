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

let audio_common_msgs = _finder('audio_common_msgs');

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
        this.speech = new audio_common_msgs.msg.AudioData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TtsResponse
    // Serialize message field [speech]
    bufferOffset = audio_common_msgs.msg.AudioData.serialize(obj.speech, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TtsResponse
    let len;
    let data = new TtsResponse(null);
    // Deserialize message field [speech]
    data.speech = audio_common_msgs.msg.AudioData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += audio_common_msgs.msg.AudioData.getMessageSize(object.speech);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/TtsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17bac3472f989c850bf73ced5a271756';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    audio_common_msgs/AudioData speech
    
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
      resolved.speech = audio_common_msgs.msg.AudioData.Resolve(msg.speech)
    }
    else {
      resolved.speech = new audio_common_msgs.msg.AudioData()
    }

    return resolved;
    }
};

module.exports = {
  Request: TtsRequest,
  Response: TtsResponse,
  md5sum() { return '32b04ab27e111935c7e979243abf2412'; },
  datatype() { return 'infrastructure/Tts'; }
};
