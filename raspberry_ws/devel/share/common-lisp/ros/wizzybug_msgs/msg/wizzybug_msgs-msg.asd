
(cl:in-package :asdf)

(defsystem "wizzybug_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ChairState" :depends-on ("_package_ChairState"))
    (:file "_package_ChairState" :depends-on ("_package"))
    (:file "lidar_data" :depends-on ("_package_lidar_data"))
    (:file "_package_lidar_data" :depends-on ("_package"))
    (:file "obstacle" :depends-on ("_package_obstacle"))
    (:file "_package_obstacle" :depends-on ("_package"))
    (:file "obstacleArray" :depends-on ("_package_obstacleArray"))
    (:file "_package_obstacleArray" :depends-on ("_package"))
    (:file "ttc" :depends-on ("_package_ttc"))
    (:file "_package_ttc" :depends-on ("_package"))
  ))