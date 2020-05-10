; Auto-generated. Do not edit!


(cl:in-package wizzybug_msgs-msg)


;//! \htmlinclude obstacleArray.msg.html

(cl:defclass <obstacleArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector wizzybug_msgs-msg:obstacle)
   :initform (cl:make-array 0 :element-type 'wizzybug_msgs-msg:obstacle :initial-element (cl:make-instance 'wizzybug_msgs-msg:obstacle))))
)

(cl:defclass obstacleArray (<obstacleArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacleArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacleArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wizzybug_msgs-msg:<obstacleArray> is deprecated: use wizzybug_msgs-msg:obstacleArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <obstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:header-val is deprecated.  Use wizzybug_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <obstacleArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:data-val is deprecated.  Use wizzybug_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacleArray>) ostream)
  "Serializes a message object of type '<obstacleArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacleArray>) istream)
  "Deserializes a message object of type '<obstacleArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'wizzybug_msgs-msg:obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacleArray>)))
  "Returns string type for a message object of type '<obstacleArray>"
  "wizzybug_msgs/obstacleArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacleArray)))
  "Returns string type for a message object of type 'obstacleArray"
  "wizzybug_msgs/obstacleArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacleArray>)))
  "Returns md5sum for a message object of type '<obstacleArray>"
  "2f379a8004de76e2e7e7b8485dc4f074")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacleArray)))
  "Returns md5sum for a message object of type 'obstacleArray"
  "2f379a8004de76e2e7e7b8485dc4f074")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacleArray>)))
  "Returns full string definition for message of type '<obstacleArray>"
  (cl:format cl:nil "Header header~%~%wizzybug_msgs/obstacle[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: wizzybug_msgs/obstacle~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 z~%std_msgs/Float64 width~%std_msgs/Float64 height~%std_msgs/Float64 length~%~%std_msgs/String classification~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacleArray)))
  "Returns full string definition for message of type 'obstacleArray"
  (cl:format cl:nil "Header header~%~%wizzybug_msgs/obstacle[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: wizzybug_msgs/obstacle~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 z~%std_msgs/Float64 width~%std_msgs/Float64 height~%std_msgs/Float64 length~%~%std_msgs/String classification~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacleArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacleArray>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacleArray
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
