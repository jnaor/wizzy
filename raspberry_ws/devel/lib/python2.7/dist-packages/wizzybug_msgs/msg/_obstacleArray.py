# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from wizzybug_msgs/obstacleArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import wizzybug_msgs.msg
import std_msgs.msg

class obstacleArray(genpy.Message):
  _md5sum = "2f379a8004de76e2e7e7b8485dc4f074"
  _type = "wizzybug_msgs/obstacleArray"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

wizzybug_msgs/obstacle[] data

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
MSG: wizzybug_msgs/obstacle
std_msgs/Float64 x
std_msgs/Float64 y
std_msgs/Float64 z
std_msgs/Float64 width
std_msgs/Float64 height
std_msgs/Float64 length

std_msgs/String classification

================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data
"""
  __slots__ = ['header','data']
  _slot_types = ['std_msgs/Header','wizzybug_msgs/obstacle[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(obstacleArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = []
    else:
      self.header = std_msgs.msg.Header()
      self.data = []

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _v1 = val1.x
        buff.write(_get_struct_d().pack(_v1.data))
        _v2 = val1.y
        buff.write(_get_struct_d().pack(_v2.data))
        _v3 = val1.z
        buff.write(_get_struct_d().pack(_v3.data))
        _v4 = val1.width
        buff.write(_get_struct_d().pack(_v4.data))
        _v5 = val1.height
        buff.write(_get_struct_d().pack(_v5.data))
        _v6 = val1.length
        buff.write(_get_struct_d().pack(_v6.data))
        _v7 = val1.classification
        _x = _v7.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = wizzybug_msgs.msg.obstacle()
        _v8 = val1.x
        start = end
        end += 8
        (_v8.data,) = _get_struct_d().unpack(str[start:end])
        _v9 = val1.y
        start = end
        end += 8
        (_v9.data,) = _get_struct_d().unpack(str[start:end])
        _v10 = val1.z
        start = end
        end += 8
        (_v10.data,) = _get_struct_d().unpack(str[start:end])
        _v11 = val1.width
        start = end
        end += 8
        (_v11.data,) = _get_struct_d().unpack(str[start:end])
        _v12 = val1.height
        start = end
        end += 8
        (_v12.data,) = _get_struct_d().unpack(str[start:end])
        _v13 = val1.length
        start = end
        end += 8
        (_v13.data,) = _get_struct_d().unpack(str[start:end])
        _v14 = val1.classification
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v14.data = str[start:end].decode('utf-8')
        else:
          _v14.data = str[start:end]
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _v15 = val1.x
        buff.write(_get_struct_d().pack(_v15.data))
        _v16 = val1.y
        buff.write(_get_struct_d().pack(_v16.data))
        _v17 = val1.z
        buff.write(_get_struct_d().pack(_v17.data))
        _v18 = val1.width
        buff.write(_get_struct_d().pack(_v18.data))
        _v19 = val1.height
        buff.write(_get_struct_d().pack(_v19.data))
        _v20 = val1.length
        buff.write(_get_struct_d().pack(_v20.data))
        _v21 = val1.classification
        _x = _v21.data
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = wizzybug_msgs.msg.obstacle()
        _v22 = val1.x
        start = end
        end += 8
        (_v22.data,) = _get_struct_d().unpack(str[start:end])
        _v23 = val1.y
        start = end
        end += 8
        (_v23.data,) = _get_struct_d().unpack(str[start:end])
        _v24 = val1.z
        start = end
        end += 8
        (_v24.data,) = _get_struct_d().unpack(str[start:end])
        _v25 = val1.width
        start = end
        end += 8
        (_v25.data,) = _get_struct_d().unpack(str[start:end])
        _v26 = val1.height
        start = end
        end += 8
        (_v26.data,) = _get_struct_d().unpack(str[start:end])
        _v27 = val1.length
        start = end
        end += 8
        (_v27.data,) = _get_struct_d().unpack(str[start:end])
        _v28 = val1.classification
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v28.data = str[start:end].decode('utf-8')
        else:
          _v28.data = str[start:end]
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

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
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d