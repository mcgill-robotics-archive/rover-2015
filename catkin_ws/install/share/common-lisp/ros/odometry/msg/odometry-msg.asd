
(cl:in-package :asdf)

(defsystem "odometry-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Tachometers" :depends-on ("_package_Tachometers"))
    (:file "_package_Tachometers" :depends-on ("_package"))
  ))