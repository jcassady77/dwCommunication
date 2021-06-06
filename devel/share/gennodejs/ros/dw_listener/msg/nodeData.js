// Auto-generated. Do not edit!

// (in-package dw_listener.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class nodeData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tagAddress = null;
      this.rangeNum = null;
      this.timeOfReception = null;
      this.distance = null;
      this.degrees = null;
      this.Xcoord = null;
      this.Ycoord = null;
      this.XcoordFiltered = null;
      this.YcoordFiltered = null;
      this.clockOffset = null;
      this.serviceData = null;
      this.Xaccel = null;
      this.Yaccel = null;
      this.Zaccel = null;
    }
    else {
      if (initObj.hasOwnProperty('tagAddress')) {
        this.tagAddress = initObj.tagAddress
      }
      else {
        this.tagAddress = '';
      }
      if (initObj.hasOwnProperty('rangeNum')) {
        this.rangeNum = initObj.rangeNum
      }
      else {
        this.rangeNum = 0;
      }
      if (initObj.hasOwnProperty('timeOfReception')) {
        this.timeOfReception = initObj.timeOfReception
      }
      else {
        this.timeOfReception = 0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0;
      }
      if (initObj.hasOwnProperty('degrees')) {
        this.degrees = initObj.degrees
      }
      else {
        this.degrees = 0;
      }
      if (initObj.hasOwnProperty('Xcoord')) {
        this.Xcoord = initObj.Xcoord
      }
      else {
        this.Xcoord = 0;
      }
      if (initObj.hasOwnProperty('Ycoord')) {
        this.Ycoord = initObj.Ycoord
      }
      else {
        this.Ycoord = 0;
      }
      if (initObj.hasOwnProperty('XcoordFiltered')) {
        this.XcoordFiltered = initObj.XcoordFiltered
      }
      else {
        this.XcoordFiltered = 0;
      }
      if (initObj.hasOwnProperty('YcoordFiltered')) {
        this.YcoordFiltered = initObj.YcoordFiltered
      }
      else {
        this.YcoordFiltered = 0;
      }
      if (initObj.hasOwnProperty('clockOffset')) {
        this.clockOffset = initObj.clockOffset
      }
      else {
        this.clockOffset = 0;
      }
      if (initObj.hasOwnProperty('serviceData')) {
        this.serviceData = initObj.serviceData
      }
      else {
        this.serviceData = 0;
      }
      if (initObj.hasOwnProperty('Xaccel')) {
        this.Xaccel = initObj.Xaccel
      }
      else {
        this.Xaccel = 0;
      }
      if (initObj.hasOwnProperty('Yaccel')) {
        this.Yaccel = initObj.Yaccel
      }
      else {
        this.Yaccel = 0;
      }
      if (initObj.hasOwnProperty('Zaccel')) {
        this.Zaccel = initObj.Zaccel
      }
      else {
        this.Zaccel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type nodeData
    // Serialize message field [tagAddress]
    bufferOffset = _serializer.string(obj.tagAddress, buffer, bufferOffset);
    // Serialize message field [rangeNum]
    bufferOffset = _serializer.int64(obj.rangeNum, buffer, bufferOffset);
    // Serialize message field [timeOfReception]
    bufferOffset = _serializer.int64(obj.timeOfReception, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.int64(obj.distance, buffer, bufferOffset);
    // Serialize message field [degrees]
    bufferOffset = _serializer.int64(obj.degrees, buffer, bufferOffset);
    // Serialize message field [Xcoord]
    bufferOffset = _serializer.int64(obj.Xcoord, buffer, bufferOffset);
    // Serialize message field [Ycoord]
    bufferOffset = _serializer.int64(obj.Ycoord, buffer, bufferOffset);
    // Serialize message field [XcoordFiltered]
    bufferOffset = _serializer.int64(obj.XcoordFiltered, buffer, bufferOffset);
    // Serialize message field [YcoordFiltered]
    bufferOffset = _serializer.int64(obj.YcoordFiltered, buffer, bufferOffset);
    // Serialize message field [clockOffset]
    bufferOffset = _serializer.int64(obj.clockOffset, buffer, bufferOffset);
    // Serialize message field [serviceData]
    bufferOffset = _serializer.int64(obj.serviceData, buffer, bufferOffset);
    // Serialize message field [Xaccel]
    bufferOffset = _serializer.int64(obj.Xaccel, buffer, bufferOffset);
    // Serialize message field [Yaccel]
    bufferOffset = _serializer.int64(obj.Yaccel, buffer, bufferOffset);
    // Serialize message field [Zaccel]
    bufferOffset = _serializer.int64(obj.Zaccel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type nodeData
    let len;
    let data = new nodeData(null);
    // Deserialize message field [tagAddress]
    data.tagAddress = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [rangeNum]
    data.rangeNum = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [timeOfReception]
    data.timeOfReception = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [degrees]
    data.degrees = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Xcoord]
    data.Xcoord = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Ycoord]
    data.Ycoord = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [XcoordFiltered]
    data.XcoordFiltered = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [YcoordFiltered]
    data.YcoordFiltered = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [clockOffset]
    data.clockOffset = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [serviceData]
    data.serviceData = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Xaccel]
    data.Xaccel = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Yaccel]
    data.Yaccel = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [Zaccel]
    data.Zaccel = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.tagAddress.length;
    return length + 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dw_listener/nodeData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33334d719b640f53532bf5bf4909ab5c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string tagAddress
    int64 rangeNum
    int64 timeOfReception
    int64 distance
    int64 degrees
    int64 Xcoord
    int64 Ycoord
    int64 XcoordFiltered
    int64 YcoordFiltered
    int64 clockOffset
    int64 serviceData
    int64 Xaccel
    int64 Yaccel
    int64 Zaccel
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new nodeData(null);
    if (msg.tagAddress !== undefined) {
      resolved.tagAddress = msg.tagAddress;
    }
    else {
      resolved.tagAddress = ''
    }

    if (msg.rangeNum !== undefined) {
      resolved.rangeNum = msg.rangeNum;
    }
    else {
      resolved.rangeNum = 0
    }

    if (msg.timeOfReception !== undefined) {
      resolved.timeOfReception = msg.timeOfReception;
    }
    else {
      resolved.timeOfReception = 0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0
    }

    if (msg.degrees !== undefined) {
      resolved.degrees = msg.degrees;
    }
    else {
      resolved.degrees = 0
    }

    if (msg.Xcoord !== undefined) {
      resolved.Xcoord = msg.Xcoord;
    }
    else {
      resolved.Xcoord = 0
    }

    if (msg.Ycoord !== undefined) {
      resolved.Ycoord = msg.Ycoord;
    }
    else {
      resolved.Ycoord = 0
    }

    if (msg.XcoordFiltered !== undefined) {
      resolved.XcoordFiltered = msg.XcoordFiltered;
    }
    else {
      resolved.XcoordFiltered = 0
    }

    if (msg.YcoordFiltered !== undefined) {
      resolved.YcoordFiltered = msg.YcoordFiltered;
    }
    else {
      resolved.YcoordFiltered = 0
    }

    if (msg.clockOffset !== undefined) {
      resolved.clockOffset = msg.clockOffset;
    }
    else {
      resolved.clockOffset = 0
    }

    if (msg.serviceData !== undefined) {
      resolved.serviceData = msg.serviceData;
    }
    else {
      resolved.serviceData = 0
    }

    if (msg.Xaccel !== undefined) {
      resolved.Xaccel = msg.Xaccel;
    }
    else {
      resolved.Xaccel = 0
    }

    if (msg.Yaccel !== undefined) {
      resolved.Yaccel = msg.Yaccel;
    }
    else {
      resolved.Yaccel = 0
    }

    if (msg.Zaccel !== undefined) {
      resolved.Zaccel = msg.Zaccel;
    }
    else {
      resolved.Zaccel = 0
    }

    return resolved;
    }
};

module.exports = nodeData;
