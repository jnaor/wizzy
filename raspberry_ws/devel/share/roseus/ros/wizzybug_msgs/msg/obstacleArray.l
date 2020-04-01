;; Auto-generated. Do not edit!


(when (boundp 'wizzybug_msgs::obstacleArray)
  (if (not (find-package "WIZZYBUG_MSGS"))
    (make-package "WIZZYBUG_MSGS"))
  (shadow 'obstacleArray (find-package "WIZZYBUG_MSGS")))
(unless (find-package "WIZZYBUG_MSGS::OBSTACLEARRAY")
  (make-package "WIZZYBUG_MSGS::OBSTACLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude obstacleArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wizzybug_msgs::obstacleArray
  :super ros::object
  :slots (_header _data ))

(defmethod wizzybug_msgs::obstacleArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (let (r) (dotimes (i 0) (push (instance wizzybug_msgs::obstacle :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; wizzybug_msgs/obstacle[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; wizzybug_msgs/obstacle[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; wizzybug_msgs/obstacle[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance wizzybug_msgs::obstacle :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get wizzybug_msgs::obstacleArray :md5sum-) "2f379a8004de76e2e7e7b8485dc4f074")
(setf (get wizzybug_msgs::obstacleArray :datatype-) "wizzybug_msgs/obstacleArray")
(setf (get wizzybug_msgs::obstacleArray :definition-)
      "Header header

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

")



(provide :wizzybug_msgs/obstacleArray "2f379a8004de76e2e7e7b8485dc4f074")


