
(cl:in-package :asdf)

(defsystem "odometry-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "DisplacementMiddleWheels" :depends-on ("_package_DisplacementMiddleWheels"))
    (:file "_package_DisplacementMiddleWheels" :depends-on ("_package"))
  ))