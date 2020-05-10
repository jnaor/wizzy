;; Auto-generated. Do not edit!


(when (boundp 'wizzybug_msgs::ChairState)
  (if (not (find-package "WIZZYBUG_MSGS"))
    (make-package "WIZZYBUG_MSGS"))
  (shadow 'ChairState (find-package "WIZZYBUG_MSGS")))
(unless (find-package "WIZZYBUG_MSGS::CHAIRSTATE")
  (make-package "WIZZYBUG_MSGS::CHAIRSTATE"))

(in-package "ROS")
;;//! \htmlinclude ChairState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wizzybug_msgs::ChairState
  :super ros::object
  :slots (_header _obstacles _state _ttc_msg ))

(defmethod wizzybug_msgs::ChairState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) (let (r) (dotimes (i 0) (push (instance wizzybug_msgs::obstacle :init) r)) r))
    ((:state __state) (instance std_msgs::String :init))
    ((:ttc_msg __ttc_msg) (instance wizzybug_msgs::ttc :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   (setq _state __state)
   (setq _ttc_msg __ttc_msg)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:ttc_msg
   (&rest __ttc_msg)
   (if (keywordp (car __ttc_msg))
       (send* _ttc_msg __ttc_msg)
     (progn
       (if __ttc_msg (setq _ttc_msg (car __ttc_msg)))
       _ttc_msg)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; wizzybug_msgs/obstacle[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ;; std_msgs/String _state
    (send _state :serialization-length)
    ;; wizzybug_msgs/ttc _ttc_msg
    (send _ttc_msg :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; wizzybug_msgs/obstacle[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;; std_msgs/String _state
       (send _state :serialize s)
     ;; wizzybug_msgs/ttc _ttc_msg
       (send _ttc_msg :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; wizzybug_msgs/obstacle[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance wizzybug_msgs::obstacle :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/String _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;; wizzybug_msgs/ttc _ttc_msg
     (send _ttc_msg :deserialize buf ptr-) (incf ptr- (send _ttc_msg :serialization-length))
   ;;
   self)
  )

(setf (get wizzybug_msgs::ChairState :md5sum-) "80026f80fdf5c7c2e3ba8933871ba1e1")
(setf (get wizzybug_msgs::ChairState :datatype-) "wizzybug_msgs/ChairState")
(setf (get wizzybug_msgs::ChairState :definition-)
      "Header header

wizzybug_msgs/obstacle[] obstacles
std_msgs/String state
wizzybug_msgs/ttc ttc_msg

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

================================================================================
MSG: wizzybug_msgs/ttc
Header header

float32 ttc
float32 ttc_azimuth

")



(provide :wizzybug_msgs/ChairState "80026f80fdf5c7c2e3ba8933871ba1e1")


