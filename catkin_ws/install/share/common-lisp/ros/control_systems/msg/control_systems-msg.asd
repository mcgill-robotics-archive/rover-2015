
(cl:in-package :asdf)

(defsystem "control_systems-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Moving" :depends-on ("_package_Moving"))
    (:file "_package_Moving" :depends-on ("_package"))
    (:file "ArmAngles" :depends-on ("_package_ArmAngles"))
    (:file "_package_ArmAngles" :depends-on ("_package"))
    (:file "MotionType" :depends-on ("_package_MotionType"))
    (:file "_package_MotionType" :depends-on ("_package"))
    (:file "PanTiltZoom" :depends-on ("_package_PanTiltZoom"))
    (:file "_package_PanTiltZoom" :depends-on ("_package"))
    (:file "ArmMotion" :depends-on ("_package_ArmMotion"))
    (:file "_package_ArmMotion" :depends-on ("_package"))
    (:file "SetPoints" :depends-on ("_package_SetPoints"))
    (:file "_package_SetPoints" :depends-on ("_package"))
  ))