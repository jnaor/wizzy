; Auto-generated. Do not edit!


(cl:in-package wizzybug_msgs-msg)


;//! \htmlinclude ttc.msg.html

(cl:defclass <ttc> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ttc
    :reader ttc
    :initarg :ttc
    :type cl:float
    :initform 0.0)
   (ttc_azimuth
    :reader ttc_azimuth
    :initarg :ttc_azimuth
    :type cl:float
    :initform 0.0))
)

(cl:defclass ttc (<ttc>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ttc>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ttc)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wizzybug_msgs-msg:<ttc> is deprecated: use wizzybug_msgs-msg:ttc instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ttc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:header-val is deprecated.  Use wizzybug_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ttc-val :lambda-list '(m))
(cl:defmethod ttc-val ((m <ttc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:ttc-val is deprecated.  Use wizzybug_msgs-msg:ttc instead.")
  (ttc m))

(cl:ensure-generic-function 'ttc_azimuth-val :lambda-list '(m))
(cl:defmethod ttc_azimuth-val ((m <ttc>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:ttc_azimuth-val is deprecated.  Use wizzybug_msgs-msg:ttc_azimuth instead.")
  (ttc_azimuth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ttc>) ostream)
  "Serializes a message object of type '<ttc>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ttc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ttc_azimuth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ttc>) istream)
  "Deserializes a message object of type '<ttc>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ttc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ttc_azimuth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ttc>)))
  "Returns string type for a message object of type '<ttc>"
  "wizzybug_msgs/ttc")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ttc)))
  "Returns string type for a message object of type 'ttc"
  "wizzybug_msgs/ttc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ttc>)))
  "Returns md5sum for a message object of type '<ttc>"
  "4fd9fba655aceff04f987cabc9046972")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ttc)))
  "Returns md5sum for a message object of type 'ttc"
  "4fd9fba655aceff04f987cabc9046972")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ttc>)))
  "Returns full string definition for message of type '<ttc>"
  (cl:format cl:nil "Header header~%~%float32 ttc~%float32 ttc_azimuth~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ttc)))
  "Returns full string definition for message of type 'ttc"
  (cl:format cl:nil "Header header~%~%float32 ttc~%float32 ttc_azimuth~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ttc>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ttc>))
  "Converts a ROS message object to a list"
  (cl:list 'ttc
    (cl:cons ':header (header msg))
    (cl:cons ':ttc (ttc msg))
    (cl:cons ':ttc_azimuth (ttc_azimuth msg))
))
