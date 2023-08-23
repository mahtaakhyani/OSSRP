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

class AudFeatures {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min_f0 = null;
      this.max_f0 = null;
      this.mean_f0 = null;
      this.min_int = null;
      this.max_int = null;
      this.mean_int = null;
      this.jitter = null;
      this.shimmer = null;
      this.hnr = null;
      this.speaking_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('min_f0')) {
        this.min_f0 = initObj.min_f0
      }
      else {
        this.min_f0 = 0.0;
      }
      if (initObj.hasOwnProperty('max_f0')) {
        this.max_f0 = initObj.max_f0
      }
      else {
        this.max_f0 = 0.0;
      }
      if (initObj.hasOwnProperty('mean_f0')) {
        this.mean_f0 = initObj.mean_f0
      }
      else {
        this.mean_f0 = 0.0;
      }
      if (initObj.hasOwnProperty('min_int')) {
        this.min_int = initObj.min_int
      }
      else {
        this.min_int = 0.0;
      }
      if (initObj.hasOwnProperty('max_int')) {
        this.max_int = initObj.max_int
      }
      else {
        this.max_int = 0.0;
      }
      if (initObj.hasOwnProperty('mean_int')) {
        this.mean_int = initObj.mean_int
      }
      else {
        this.mean_int = 0.0;
      }
      if (initObj.hasOwnProperty('jitter')) {
        this.jitter = initObj.jitter
      }
      else {
        this.jitter = 0.0;
      }
      if (initObj.hasOwnProperty('shimmer')) {
        this.shimmer = initObj.shimmer
      }
      else {
        this.shimmer = 0.0;
      }
      if (initObj.hasOwnProperty('hnr')) {
        this.hnr = initObj.hnr
      }
      else {
        this.hnr = 0.0;
      }
      if (initObj.hasOwnProperty('speaking_rate')) {
        this.speaking_rate = initObj.speaking_rate
      }
      else {
        this.speaking_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AudFeatures
    // Serialize message field [min_f0]
    bufferOffset = _serializer.float64(obj.min_f0, buffer, bufferOffset);
    // Serialize message field [max_f0]
    bufferOffset = _serializer.float64(obj.max_f0, buffer, bufferOffset);
    // Serialize message field [mean_f0]
    bufferOffset = _serializer.float64(obj.mean_f0, buffer, bufferOffset);
    // Serialize message field [min_int]
    bufferOffset = _serializer.float64(obj.min_int, buffer, bufferOffset);
    // Serialize message field [max_int]
    bufferOffset = _serializer.float64(obj.max_int, buffer, bufferOffset);
    // Serialize message field [mean_int]
    bufferOffset = _serializer.float64(obj.mean_int, buffer, bufferOffset);
    // Serialize message field [jitter]
    bufferOffset = _serializer.float64(obj.jitter, buffer, bufferOffset);
    // Serialize message field [shimmer]
    bufferOffset = _serializer.float64(obj.shimmer, buffer, bufferOffset);
    // Serialize message field [hnr]
    bufferOffset = _serializer.float64(obj.hnr, buffer, bufferOffset);
    // Serialize message field [speaking_rate]
    bufferOffset = _serializer.float64(obj.speaking_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AudFeatures
    let len;
    let data = new AudFeatures(null);
    // Deserialize message field [min_f0]
    data.min_f0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_f0]
    data.max_f0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mean_f0]
    data.mean_f0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [min_int]
    data.min_int = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_int]
    data.max_int = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mean_int]
    data.mean_int = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [jitter]
    data.jitter = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [shimmer]
    data.shimmer = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [hnr]
    data.hnr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speaking_rate]
    data.speaking_rate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'infrastructure/AudFeatures';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c6ab6a6d7305a866c023967280bedf4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new AudFeatures(null);
    if (msg.min_f0 !== undefined) {
      resolved.min_f0 = msg.min_f0;
    }
    else {
      resolved.min_f0 = 0.0
    }

    if (msg.max_f0 !== undefined) {
      resolved.max_f0 = msg.max_f0;
    }
    else {
      resolved.max_f0 = 0.0
    }

    if (msg.mean_f0 !== undefined) {
      resolved.mean_f0 = msg.mean_f0;
    }
    else {
      resolved.mean_f0 = 0.0
    }

    if (msg.min_int !== undefined) {
      resolved.min_int = msg.min_int;
    }
    else {
      resolved.min_int = 0.0
    }

    if (msg.max_int !== undefined) {
      resolved.max_int = msg.max_int;
    }
    else {
      resolved.max_int = 0.0
    }

    if (msg.mean_int !== undefined) {
      resolved.mean_int = msg.mean_int;
    }
    else {
      resolved.mean_int = 0.0
    }

    if (msg.jitter !== undefined) {
      resolved.jitter = msg.jitter;
    }
    else {
      resolved.jitter = 0.0
    }

    if (msg.shimmer !== undefined) {
      resolved.shimmer = msg.shimmer;
    }
    else {
      resolved.shimmer = 0.0
    }

    if (msg.hnr !== undefined) {
      resolved.hnr = msg.hnr;
    }
    else {
      resolved.hnr = 0.0
    }

    if (msg.speaking_rate !== undefined) {
      resolved.speaking_rate = msg.speaking_rate;
    }
    else {
      resolved.speaking_rate = 0.0
    }

    return resolved;
    }
};

module.exports = AudFeatures;
