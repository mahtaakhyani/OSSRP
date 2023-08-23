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

let AudFeatures = require('../msg/AudFeatures.js');

//-----------------------------------------------------------

class AudFeatureRequest {
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
    // Serializes a message object of type AudFeatureRequest
    // Serialize message field [audio]
    bufferOffset = audio_common_msgs.msg.AudioDataStamped.serialize(obj.audio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AudFeatureRequest
    let len;
    let data = new AudFeatureRequest(null);
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
    return 'infrastructure/AudFeatureRequest';
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
    const resolved = new AudFeatureRequest(null);
    if (msg.audio !== undefined) {
      resolved.audio = audio_common_msgs.msg.AudioDataStamped.Resolve(msg.audio)
    }
    else {
      resolved.audio = new audio_common_msgs.msg.AudioDataStamped()
    }

    return resolved;
    }
};

class AudFeatureResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.features = null;
    }
    else {
      if (initObj.hasOwnProperty('features')) {
        this.features = initObj.features
      }
      else {
        this.features = new AudFeatures();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AudFeatureResponse
    // Serialize message field [features]
    bufferOffset = AudFeatures.serialize(obj.features, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AudFeatureResponse
    let len;
    let data = new AudFeatureResponse(null);
    // Deserialize message field [features]
    data.features = AudFeatures.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/AudFeatureResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81e73fd091a8624334b5b3adb6e535ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    infrastructure/AudFeatures features
    
    ================================================================================
    MSG: infrastructure/AudFeatures
    float64 min_f0 #minimum fundamental frequency
    float64 max_f0 #maximum fundamental frequency
    float64 mean_f0 #mean fundamental frequency
    float64 min_int #minimum intensity
    float64 max_int #maximum intensity
    float64 mean_int #mean intensity
    float64 jitter #local
    float64 shimmer #local
    float64 hnr #harmonic to noise ratio
    float64 speaking_rate #of syllables per second
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AudFeatureResponse(null);
    if (msg.features !== undefined) {
      resolved.features = AudFeatures.Resolve(msg.features)
    }
    else {
      resolved.features = new AudFeatures()
    }

    return resolved;
    }
};

module.exports = {
  Request: AudFeatureRequest,
  Response: AudFeatureResponse,
  md5sum() { return 'b83fe3706a17dd745e1029708d72454b'; },
  datatype() { return 'infrastructure/AudFeature'; }
};
