;; Auto-generated. Do not edit!


(when (boundp 'wizzybug_msgs::obstacle)
  (if (not (find-package "WIZZYBUG_MSGS"))
    (make-package "WIZZYBUG_MSGS"))
  (shadow 'obstacle (find-package "WIZZYBUG_MSGS")))
(unless (find-package "WIZZYBUG_MSGS::OBSTACLE")
  (make-package "WIZZYBUG_MSGS::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude obstacle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass wizzybug_msgs::obstacle
  :super ros::object
  :slots (_x _y _z _width _height _length _classification ))

(defmethod wizzybug_msgs::obstacle
  (:init
   (&key
    ((:x __x) (instance std_msgs::Float64 :init))
    ((:y __y) (instance std_msgs::Float64 :init))
    ((:z __z) (instance std_msgs::Float64 :init))
    ((:width __width) (instance std_msgs::Float64 :init))
    ((:height __height) (instance std_msgs::Float64 :init))
    ((:length __length) (instance std_msgs::Float64 :init))
    ((:classification __classification) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _z __z)
   (setq _width __width)
   (setq _height __height)
   (setq _length __length)
   (setq _classification __classification)
   self)
  (:x
   (&rest __x)
   (if (keywordp (car __x))
       (send* _x __x)
     (progn
       (if __x (setq _x (car __x)))
       _x)))
  (:y
   (&rest __y)
   (if (keywordp (car __y))
       (send* _y __y)
     (progn
       (if __y (setq _y (car __y)))
       _y)))
  (:z
   (&rest __z)
   (if (keywordp (car __z))
       (send* _z __z)
     (progn
       (if __z (setq _z (car __z)))
       _z)))
  (:width
   (&rest __width)
   (if (keywordp (car __width))
       (send* _width __width)
     (progn
       (if __width (setq _width (car __width)))
       _width)))
  (:height
   (&rest __height)
   (if (keywordp (car __height))
       (send* _height __height)
     (progn
       (if __height (setq _height (car __height)))
       _height)))
  (:length
   (&rest __length)
   (if (keywordp (car __length))
       (send* _length __length)
     (progn
       (if __length (setq _length (car __length)))
       _length)))
  (:classification
   (&rest __classification)
   (if (keywordp (car __classification))
       (send* _classification __classification)
     (progn
       (if __classification (setq _classification (car __classification)))
       _classification)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _x
    (send _x :serialization-length)
    ;; std_msgs/Float64 _y
    (send _y :serialization-length)
    ;; std_msgs/Float64 _z
    (send _z :serialization-length)
    ;; std_msgs/Float64 _width
    (send _width :serialization-length)
    ;; std_msgs/Float64 _height
    (send _height :serialization-length)
    ;; std_msgs/Float64 _length
    (send _length :serialization-length)
    ;; std_msgs/String _classification
    (send _classification :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _x
       (send _x :serialize s)
     ;; std_msgs/Float64 _y
       (send _y :serialize s)
     ;; std_msgs/Float64 _z
       (send _z :serialize s)
     ;; std_msgs/Float64 _width
       (send _width :serialize s)
     ;; std_msgs/Float64 _height
       (send _height :serialize s)
     ;; std_msgs/Float64 _length
       (send _length :serialize s)
     ;; std_msgs/String _classification
       (send _classification :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float64 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;; std_msgs/Float64 _z
     (send _z :deserialize buf ptr-) (incf ptr- (send _z :serialization-length))
   ;; std_msgs/Float64 _width
     (send _width :deserialize buf ptr-) (incf ptr- (send _width :serialization-length))
   ;; std_msgs/Float64 _height
     (send _height :deserialize buf ptr-) (incf ptr- (send _height :serialization-length))
   ;; std_msgs/Float64 _length
     (send _length :deserialize buf ptr-) (incf ptr- (send _length :serialization-length))
   ;; std_msgs/String _classification
     (send _classification :deserialize buf ptr-) (incf ptr- (send _classification :serialization-length))
   ;;
   self)
  )

(setf (get wizzybug_msgs::obstacle :md5sum-) "8d2575537d9593df2bf86f5b4318f6e4")
(setf (get wizzybug_msgs::obstacle :datatype-) "wizzybug_msgs/obstacle")
(setf (get wizzybug_msgs::obstacle :definition-)
      "std_msgs/Float64 x
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



(provide :wizzybug_msgs/obstacle "8d2575537d9593df2bf86f5b4318f6e4")


