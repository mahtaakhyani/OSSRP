# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from face_pkg/Exp.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class Exp(genpy.Message):
  _md5sum = "482c620b9db5aa1eb01410b3c618ded5"
  _type = "face_pkg/Exp"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint32 seq
time time
string action
string emotion
bool auto_imit"""
  __slots__ = ['seq','time','action','emotion','auto_imit']
  _slot_types = ['uint32','time','string','string','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       seq,time,action,emotion,auto_imit

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Exp, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.seq is None:
        self.seq = 0
      if self.time is None:
        self.time = genpy.Time()
      if self.action is None:
        self.action = ''
      if self.emotion is None:
        self.emotion = ''
      if self.auto_imit is None:
        self.auto_imit = False
    else:
      self.seq = 0
      self.time = genpy.Time()
      self.action = ''
      self.emotion = ''
      self.auto_imit = False

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
      _x = self
      buff.write(_get_struct_3I().pack(_x.seq, _x.time.secs, _x.time.nsecs))
      _x = self.action
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.emotion
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.auto_imit
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.time is None:
        self.time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.time.secs, _x.time.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.action = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.action = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.emotion = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.emotion = str[start:end]
      start = end
      end += 1
      (self.auto_imit,) = _get_struct_B().unpack(str[start:end])
      self.auto_imit = bool(self.auto_imit)
      self.time.canon()
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
      _x = self
      buff.write(_get_struct_3I().pack(_x.seq, _x.time.secs, _x.time.nsecs))
      _x = self.action
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.emotion
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.auto_imit
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.time is None:
        self.time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.seq, _x.time.secs, _x.time.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.action = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.action = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.emotion = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.emotion = str[start:end]
      start = end
      end += 1
      (self.auto_imit,) = _get_struct_B().unpack(str[start:end])
      self.auto_imit = bool(self.auto_imit)
      self.time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
