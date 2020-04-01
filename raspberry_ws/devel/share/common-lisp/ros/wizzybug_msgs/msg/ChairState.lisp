; Auto-generated. Do not edit!


(cl:in-package wizzybug_msgs-msg)


;//! \htmlinclude ChairState.msg.html

(cl:defclass <ChairState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector wizzybug_msgs-msg:obstacle)
   :initform (cl:make-array 0 :element-type 'wizzybug_msgs-msg:obstacle :initial-element (cl:make-instance 'wizzybug_msgs-msg:obstacle)))
   (state
    :reader state
    :initarg :state
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (ttc_msg
    :reader ttc_msg
    :initarg :ttc_msg
    :type wizzybug_msgs-msg:ttc
    :initform (cl:make-instance 'wizzybug_msgs-msg:ttc)))
)

(cl:defclass ChairState (<ChairState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChairState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChairState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wizzybug_msgs-msg:<ChairState> is deprecated: use wizzybug_msgs-msg:ChairState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ChairState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:header-val is deprecated.  Use wizzybug_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <ChairState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:obstacles-val is deprecated.  Use wizzybug_msgs-msg:obstacles instead.")
  (obstacles m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ChairState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:state-val is deprecated.  Use wizzybug_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'ttc_msg-val :lambda-list '(m))
(cl:defmethod ttc_msg-val ((m <ChairState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wizzybug_msgs-msg:ttc_msg-val is deprecated.  Use wizzybug_msgs-msg:ttc_msg instead.")
  (ttc_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChairState>) ostream)
  "Serializes a message object of type '<ChairState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ttc_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChairState>) istream)
  "Deserializes a message object of type '<ChairState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'wizzybug_msgs-msg:obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ttc_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChairState>)))
  "Returns string type for a message object of type '<ChairState>"
  "wizzybug_msgs/ChairState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChairState)))
  "Returns string type for a message object of type 'ChairState"
  "wizzybug_msgs/ChairState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChairState>)))
  "Returns md5sum for a message object of type '<ChairState>"
  "80026f80fdf5c7c2e3ba8933871ba1e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChairState)))
  "Returns md5sum for a message object of type 'ChairState"
  "80026f80fdf5c7c2e3ba8933871ba1e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChairState>)))
  "Returns full string definition for message of type '<ChairState>"
  (cl:format cl:nil "Header header~%~%wizzybug_msgs/obstacle[] obstacles~%std_msgs/String state~%wizzybug_msgs/ttc ttc_msg~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: wizzybug_msgs/obstacle~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 z~%std_msgs/Float64 width~%std_msgs/Float64 height~%std_msgs/Float64 length~%~%std_msgs/String classification~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: wizzybug_msgs/ttc~%Header header~%~%float32 ttc~%float32 ttc_azimuth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChairState)))
  "Returns full string definition for message of type 'ChairState"
  (cl:format cl:nil "Header header~%~%wizzybug_msgs/obstacle[] obstacles~%std_msgs/String state~%wizzybug_msgs/ttc ttc_msg~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: wizzybug_msgs/obstacle~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/Float64 z~%std_msgs/Float64 width~%std_msgs/Float64 height~%std_msgs/Float64 length~%~%std_msgs/String classification~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: wizzybug_msgs/ttc~%Header header~%~%float32 ttc~%float32 ttc_azimuth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChairState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ttc_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChairState>))
  "Converts a ROS message object to a list"
  (cl:list 'ChairState
    (cl:cons ':header (header msg))
    (cl:cons ':obstacles (obstacles msg))
    (cl:cons ':state (state msg))
    (cl:cons ':ttc_msg (ttc_msg msg))
))
