;; Auto-generated. Do not edit!


(when (boundp 'wizzybug_msgs::ttc)
  (if (not (find-package "WIZZYBUG_MSGS"))
    (make-package "WIZZYBUG_MSGS"))
  (shadow 'ttc (find-package "WIZZYBUG_MSGS")))
(unless (find-package "WIZZYBUG_MSGS::TTC")
  (make-package "WIZZYBUG_MSGS::TTC"))

(in-package "ROS")
;;//! \htmlinclude ttc.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wizzybug_msgs::ttc
  :super ros::object
  :slots (_header _ttc _ttc_azimuth ))

(defmethod wizzybug_msgs::ttc
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ttc __ttc) 0.0)
    ((:ttc_azimuth __ttc_azimuth) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ttc (float __ttc))
   (setq _ttc_azimuth (float __ttc_azimuth))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ttc
   (&optional __ttc)
   (if __ttc (setq _ttc __ttc)) _ttc)
  (:ttc_azimuth
   (&optional __ttc_azimuth)
   (if __ttc_azimuth (setq _ttc_azimuth __ttc_azimuth)) _ttc_azimuth)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _ttc
    4
    ;; float32 _ttc_azimuth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _ttc
       (sys::poke _ttc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ttc_azimuth
       (sys::poke _ttc_azimuth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _ttc
     (setq _ttc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ttc_azimuth
     (setq _ttc_azimuth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wizzybug_msgs::ttc :md5sum-) "4fd9fba655aceff04f987cabc9046972")
(setf (get wizzybug_msgs::ttc :datatype-) "wizzybug_msgs/ttc")
(setf (get wizzybug_msgs::ttc :definition-)
      "Header header

float32 ttc
float32 ttc_azimuth

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



(provide :wizzybug_msgs/ttc "4fd9fba655aceff04f987cabc9046972")


