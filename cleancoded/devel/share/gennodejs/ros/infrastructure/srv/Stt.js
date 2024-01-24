// Auto-generated. Do not edit!

// (in-package infrastructure.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let audio_common_msgs = _finder('audio_common_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SttRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.audio = null;
    }
    else {
      if (initObj.hasOwnProperty('audio')) {
        this.audio = initObj.audio
      }
      else {
        this.audio = new audio_common_msgs.msg.AudioDataStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SttRequest
    // Serialize message field [audio]
    bufferOffset = audio_common_msgs.msg.AudioDataStamped.serialize(obj.audio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SttRequest
    let len;
    let data = new SttRequest(null);
    // Deserialize message field [audio]
    data.audio = audio_common_msgs.msg.AudioDataStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += audio_common_msgs.msg.AudioDataStamped.getMessageSize(object.audio);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/SttRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20c7f3b9bce053570f9d04bd783346e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    audio_common_msgs/AudioDataStamped audio
    
    ================================================================================
    MSG: audio_common_msgs/AudioDataStamped
    std_msgs/Header header
    audio_common_msgs/AudioData audio
    
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
    MSG: audio_common_msgs/AudioData
    uint8[] data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SttRequest(null);
    if (msg.audio !== undefined) {
      resolved.audio = audio_common_msgs.msg.AudioDataStamped.Resolve(msg.audio)
    }
    else {
      resolved.audio = new audio_common_msgs.msg.AudioDataStamped()
    }

    return resolved;
    }
};

class SttResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.transcription = null;
    }
    else {
      if (initObj.hasOwnProperty('transcription')) {
        this.transcription = initObj.transcription
      }
      else {
        this.transcription = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SttResponse
    // Serialize message field [transcription]
    bufferOffset = _serializer.string(obj.transcription, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SttResponse
    let len;
    let data = new SttResponse(null);
    // Deserialize message field [transcription]
    data.transcription = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.transcription);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/SttResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd87ceb831c5fd1ff2239ff1c5b3dfc96';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string transcription
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SttResponse(null);
    if (msg.transcription !== undefined) {
      resolved.transcription = msg.transcription;
    }
    else {
      resolved.transcription = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SttRequest,
  Response: SttResponse,
  md5sum() { return 'c292071d452451ec62e8cf113b51a9f0'; },
  datatype() { return 'infrastructure/Stt'; }
};
