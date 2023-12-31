# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from infrastructure/AudFeatures.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AudFeatures(genpy.Message):
  _md5sum = "7c6ab6a6d7305a866c023967280bedf4"
  _type = "infrastructure/AudFeatures"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 min_f0 #minimum fundamental frequency
float64 max_f0 #maximum fundamental frequency
float64 mean_f0 #mean fundamental frequency
float64 min_int #minimum intensity
float64 max_int #maximum intensity
float64 mean_int #mean intensity
float64 jitter #local
float64 shimmer #local
float64 hnr #harmonic to noise ratio
float64 speaking_rate #of syllables per second"""
  __slots__ = ['min_f0','max_f0','mean_f0','min_int','max_int','mean_int','jitter','shimmer','hnr','speaking_rate']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       min_f0,max_f0,mean_f0,min_int,max_int,mean_int,jitter,shimmer,hnr,speaking_rate

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AudFeatures, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.min_f0 is None:
        self.min_f0 = 0.
      if self.max_f0 is None:
        self.max_f0 = 0.
      if self.mean_f0 is None:
        self.mean_f0 = 0.
      if self.min_int is None:
        self.min_int = 0.
      if self.max_int is None:
        self.max_int = 0.
      if self.mean_int is None:
        self.mean_int = 0.
      if self.jitter is None:
        self.jitter = 0.
      if self.shimmer is None:
        self.shimmer = 0.
      if self.hnr is None:
        self.hnr = 0.
      if self.speaking_rate is None:
        self.speaking_rate = 0.
    else:
      self.min_f0 = 0.
      self.max_f0 = 0.
      self.mean_f0 = 0.
      self.min_int = 0.
      self.max_int = 0.
      self.mean_int = 0.
      self.jitter = 0.
      self.shimmer = 0.
      self.hnr = 0.
      self.speaking_rate = 0.

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
      buff.write(_get_struct_10d().pack(_x.min_f0, _x.max_f0, _x.mean_f0, _x.min_int, _x.max_int, _x.mean_int, _x.jitter, _x.shimmer, _x.hnr, _x.speaking_rate))
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
      end = 0
      _x = self
      start = end
      end += 80
      (_x.min_f0, _x.max_f0, _x.mean_f0, _x.min_int, _x.max_int, _x.mean_int, _x.jitter, _x.shimmer, _x.hnr, _x.speaking_rate,) = _get_struct_10d().unpack(str[start:end])
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
      buff.write(_get_struct_10d().pack(_x.min_f0, _x.max_f0, _x.mean_f0, _x.min_int, _x.max_int, _x.mean_int, _x.jitter, _x.shimmer, _x.hnr, _x.speaking_rate))
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
      end = 0
      _x = self
      start = end
      end += 80
      (_x.min_f0, _x.max_f0, _x.mean_f0, _x.min_int, _x.max_int, _x.mean_int, _x.jitter, _x.shimmer, _x.hnr, _x.speaking_rate,) = _get_struct_10d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_10d = None
def _get_struct_10d():
    global _struct_10d
    if _struct_10d is None:
        _struct_10d = struct.Struct("<10d")
    return _struct_10d
