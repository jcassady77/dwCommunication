# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dw_listener/nodeData.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class nodeData(genpy.Message):
  _md5sum = "33334d719b640f53532bf5bf4909ab5c"
  _type = "dw_listener/nodeData"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """
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
"""
  __slots__ = ['tagAddress','rangeNum','timeOfReception','distance','degrees','Xcoord','Ycoord','XcoordFiltered','YcoordFiltered','clockOffset','serviceData','Xaccel','Yaccel','Zaccel']
  _slot_types = ['string','int64','int64','int64','int64','int64','int64','int64','int64','int64','int64','int64','int64','int64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       tagAddress,rangeNum,timeOfReception,distance,degrees,Xcoord,Ycoord,XcoordFiltered,YcoordFiltered,clockOffset,serviceData,Xaccel,Yaccel,Zaccel

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(nodeData, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.tagAddress is None:
        self.tagAddress = ''
      if self.rangeNum is None:
        self.rangeNum = 0
      if self.timeOfReception is None:
        self.timeOfReception = 0
      if self.distance is None:
        self.distance = 0
      if self.degrees is None:
        self.degrees = 0
      if self.Xcoord is None:
        self.Xcoord = 0
      if self.Ycoord is None:
        self.Ycoord = 0
      if self.XcoordFiltered is None:
        self.XcoordFiltered = 0
      if self.YcoordFiltered is None:
        self.YcoordFiltered = 0
      if self.clockOffset is None:
        self.clockOffset = 0
      if self.serviceData is None:
        self.serviceData = 0
      if self.Xaccel is None:
        self.Xaccel = 0
      if self.Yaccel is None:
        self.Yaccel = 0
      if self.Zaccel is None:
        self.Zaccel = 0
    else:
      self.tagAddress = ''
      self.rangeNum = 0
      self.timeOfReception = 0
      self.distance = 0
      self.degrees = 0
      self.Xcoord = 0
      self.Ycoord = 0
      self.XcoordFiltered = 0
      self.YcoordFiltered = 0
      self.clockOffset = 0
      self.serviceData = 0
      self.Xaccel = 0
      self.Yaccel = 0
      self.Zaccel = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.tagAddress
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_13q().pack(_x.rangeNum, _x.timeOfReception, _x.distance, _x.degrees, _x.Xcoord, _x.Ycoord, _x.XcoordFiltered, _x.YcoordFiltered, _x.clockOffset, _x.serviceData, _x.Xaccel, _x.Yaccel, _x.Zaccel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tagAddress = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tagAddress = str[start:end]
      _x = self
      start = end
      end += 104
      (_x.rangeNum, _x.timeOfReception, _x.distance, _x.degrees, _x.Xcoord, _x.Ycoord, _x.XcoordFiltered, _x.YcoordFiltered, _x.clockOffset, _x.serviceData, _x.Xaccel, _x.Yaccel, _x.Zaccel,) = _get_struct_13q().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.tagAddress
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_13q().pack(_x.rangeNum, _x.timeOfReception, _x.distance, _x.degrees, _x.Xcoord, _x.Ycoord, _x.XcoordFiltered, _x.YcoordFiltered, _x.clockOffset, _x.serviceData, _x.Xaccel, _x.Yaccel, _x.Zaccel))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.tagAddress = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.tagAddress = str[start:end]
      _x = self
      start = end
      end += 104
      (_x.rangeNum, _x.timeOfReception, _x.distance, _x.degrees, _x.Xcoord, _x.Ycoord, _x.XcoordFiltered, _x.YcoordFiltered, _x.clockOffset, _x.serviceData, _x.Xaccel, _x.Yaccel, _x.Zaccel,) = _get_struct_13q().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_13q = None
def _get_struct_13q():
    global _struct_13q
    if _struct_13q is None:
        _struct_13q = struct.Struct("<13q")
    return _struct_13q
