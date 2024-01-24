// Auto-generated. Do not edit!

// (in-package infrastructure.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Landmarks = require('../msg/Landmarks.js');
let sensor_msgs = _finder('sensor_msgs');

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
        this.frame = new sensor_msgs.msg.Image();
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
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.frame, buffer, bufferOffset);
    // Serialize message field [landmark]
    bufferOffset = Landmarks.serialize(obj.landmark, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GazeRequest
    let len;
    let data = new GazeRequest(null);
    // Deserialize message field [frame]
    data.frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [landmark]
    data.landmark = Landmarks.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.frame);
    length += Landmarks.getMessageSize(object.landmark);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'infrastructure/GazeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c1fe2cc637bb12c3915e2dc16eb8797a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image  frame
    infrastructure/Landmarks landmark
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
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
    MSG: infrastructure/Landmarks
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
    const resolved = new GazeRequest(null);
    if (msg.frame !== undefined) {
      resolved.frame = sensor_msgs.msg.Image.Resolve(msg.frame)
    }
    else {
      resolved.frame = new sensor_msgs.msg.Image()
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
  md5sum() { return '14f8acfcacac2e0e693c25a032fd2de8'; },
  datatype() { return 'infrastructure/Gaze'; }
};
