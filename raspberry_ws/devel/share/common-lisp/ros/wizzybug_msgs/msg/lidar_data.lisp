; Auto-generated. Do not edit!


(cl:in-package wizzybug_msgs-msg)


;//! \htmlinclude lidar_data.msg.html

(cl:defclass <lidar_data> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dist_to_pitfall
    :reader dist_to_pitfall
    :initarg :dist_to_pitfall
    :type cl:float
    :initform 0.0)
   (dist_to_obstacle
    :reader dist_to_obstacle
    :initarg :dist_to_obstacle
    :type cl:float
    :initform 0.0)
   (dist_to_floor
    :reader dist_to_floor
    :initarg :dist_to_floor
    :type cl:float
    :initform 0.0))
)

(cl:defclass lidar_data (<lidar_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lidar_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lidar_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wizzybug_msgs-msg:<lidar_data> is deprecated: use wizzybug_msgs-msg:lidar_data instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <lidar_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:header-val is deprecated.  Use wizzybug_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dist_to_pitfall-val :lambda-list '(m))
(cl:defmethod dist_to_pitfall-val ((m <lidar_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:dist_to_pitfall-val is deprecated.  Use wizzybug_msgs-msg:dist_to_pitfall instead.")
  (dist_to_pitfall m))

(cl:ensure-generic-function 'dist_to_obstacle-val :lambda-list '(m))
(cl:defmethod dist_to_obstacle-val ((m <lidar_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:dist_to_obstacle-val is deprecated.  Use wizzybug_msgs-msg:dist_to_obstacle instead.")
  (dist_to_obstacle m))

(cl:ensure-generic-function 'dist_to_floor-val :lambda-list '(m))
(cl:defmethod dist_to_floor-val ((m <lidar_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:dist_to_floor-val is deprecated.  Use wizzybug_msgs-msg:dist_to_floor instead.")
  (dist_to_floor m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lidar_data>) ostream)
  "Serializes a message object of type '<lidar_data>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_to_pitfall))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_to_obstacle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dist_to_floor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lidar_data>) istream)
  "Deserializes a message object of type '<lidar_data>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_to_pitfall) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_to_obstacle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist_to_floor) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lidar_data>)))
  "Returns string type for a message object of type '<lidar_data>"
  "wizzybug_msgs/lidar_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lidar_data)))
  "Returns string type for a message object of type 'lidar_data"
  "wizzybug_msgs/lidar_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lidar_data>)))
  "Returns md5sum for a message object of type '<lidar_data>"
  "c19335d47678d872f794c1f089147d5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lidar_data)))
  "Returns md5sum for a message object of type 'lidar_data"
  "c19335d47678d872f794c1f089147d5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lidar_data>)))
  "Returns full string definition for message of type '<lidar_data>"
  (cl:format cl:nil "Header header~%~%float32 dist_to_pitfall~%float32 dist_to_obstacle~%float32 dist_to_floor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lidar_data)))
  "Returns full string definition for message of type 'lidar_data"
  (cl:format cl:nil "Header header~%~%float32 dist_to_pitfall~%float32 dist_to_obstacle~%float32 dist_to_floor~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lidar_data>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lidar_data>))
  "Converts a ROS message object to a list"
  (cl:list 'lidar_data
    (cl:cons ':header (header msg))
    (cl:cons ':dist_to_pitfall (dist_to_pitfall msg))
    (cl:cons ':dist_to_obstacle (dist_to_obstacle msg))
    (cl:cons ':dist_to_floor (dist_to_floor msg))
))
