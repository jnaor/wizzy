;; Auto-generated. Do not edit!


(when (boundp 'wizzybug_msgs::lidar_data)
  (if (not (find-package "WIZZYBUG_MSGS"))
    (make-package "WIZZYBUG_MSGS"))
  (shadow 'lidar_data (find-package "WIZZYBUG_MSGS")))
(unless (find-package "WIZZYBUG_MSGS::LIDAR_DATA")
  (make-package "WIZZYBUG_MSGS::LIDAR_DATA"))

(in-package "ROS")
;;//! \htmlinclude lidar_data.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wizzybug_msgs::lidar_data
  :super ros::object
  :slots (_header _dist_to_pitfall _dist_to_obstacle _dist_to_floor ))

(defmethod wizzybug_msgs::lidar_data
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:dist_to_pitfall __dist_to_pitfall) 0.0)
    ((:dist_to_obstacle __dist_to_obstacle) 0.0)
    ((:dist_to_floor __dist_to_floor) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _dist_to_pitfall (float __dist_to_pitfall))
   (setq _dist_to_obstacle (float __dist_to_obstacle))
   (setq _dist_to_floor (float __dist_to_floor))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dist_to_pitfall
   (&optional __dist_to_pitfall)
   (if __dist_to_pitfall (setq _dist_to_pitfall __dist_to_pitfall)) _dist_to_pitfall)
  (:dist_to_obstacle
   (&optional __dist_to_obstacle)
   (if __dist_to_obstacle (setq _dist_to_obstacle __dist_to_obstacle)) _dist_to_obstacle)
  (:dist_to_floor
   (&optional __dist_to_floor)
   (if __dist_to_floor (setq _dist_to_floor __dist_to_floor)) _dist_to_floor)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _dist_to_pitfall
    4
    ;; float32 _dist_to_obstacle
    4
    ;; float32 _dist_to_floor
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _dist_to_pitfall
       (sys::poke _dist_to_pitfall (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dist_to_obstacle
       (sys::poke _dist_to_obstacle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dist_to_floor
       (sys::poke _dist_to_floor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _dist_to_pitfall
     (setq _dist_to_pitfall (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dist_to_obstacle
     (setq _dist_to_obstacle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dist_to_floor
     (setq _dist_to_floor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wizzybug_msgs::lidar_data :md5sum-) "c19335d47678d872f794c1f089147d5f")
(setf (get wizzybug_msgs::lidar_data :datatype-) "wizzybug_msgs/lidar_data")
(setf (get wizzybug_msgs::lidar_data :definition-)
      "Header header

float32 dist_to_pitfall
float32 dist_to_obstacle
float32 dist_to_floor

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

")



(provide :wizzybug_msgs/lidar_data "c19335d47678d872f794c1f089147d5f")


