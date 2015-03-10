; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude ArmAngles.msg.html

(cl:defclass <ArmAngles> (roslisp-msg-protocol:ros-message)
  ((shoulderOrientation
    :reader shoulderOrientation
    :initarg :shoulderOrientation
    :type cl:float
    :initform 0.0)
   (shoulderElevation
    :reader shoulderElevation
    :initarg :shoulderElevation
    :type cl:float
    :initform 0.0)
   (elbow
    :reader elbow
    :initarg :elbow
    :type cl:float
    :initform 0.0)
   (wristOrientation
    :reader wristOrientation
    :initarg :wristOrientation
    :type cl:float
    :initform 0.0)
   (wristElevation
    :reader wristElevation
    :initarg :wristElevation
    :type cl:float
    :initform 0.0))
)

(cl:defclass ArmAngles (<ArmAngles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmAngles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmAngles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<ArmAngles> is deprecated: use control_systems-msg:ArmAngles instead.")))

(cl:ensure-generic-function 'shoulderOrientation-val :lambda-list '(m))
(cl:defmethod shoulderOrientation-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:shoulderOrientation-val is deprecated.  Use control_systems-msg:shoulderOrientation instead.")
  (shoulderOrientation m))

(cl:ensure-generic-function 'shoulderElevation-val :lambda-list '(m))
(cl:defmethod shoulderElevation-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:shoulderElevation-val is deprecated.  Use control_systems-msg:shoulderElevation instead.")
  (shoulderElevation m))

(cl:ensure-generic-function 'elbow-val :lambda-list '(m))
(cl:defmethod elbow-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:elbow-val is deprecated.  Use control_systems-msg:elbow instead.")
  (elbow m))

(cl:ensure-generic-function 'wristOrientation-val :lambda-list '(m))
(cl:defmethod wristOrientation-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:wristOrientation-val is deprecated.  Use control_systems-msg:wristOrientation instead.")
  (wristOrientation m))

(cl:ensure-generic-function 'wristElevation-val :lambda-list '(m))
(cl:defmethod wristElevation-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:wristElevation-val is deprecated.  Use control_systems-msg:wristElevation instead.")
  (wristElevation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmAngles>) ostream)
  "Serializes a message object of type '<ArmAngles>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shoulderOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shoulderElevation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'elbow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wristOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wristElevation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmAngles>) istream)
  "Deserializes a message object of type '<ArmAngles>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shoulderOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shoulderElevation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'elbow) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wristOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wristElevation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmAngles>)))
  "Returns string type for a message object of type '<ArmAngles>"
  "control_systems/ArmAngles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmAngles)))
  "Returns string type for a message object of type 'ArmAngles"
  "control_systems/ArmAngles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmAngles>)))
  "Returns md5sum for a message object of type '<ArmAngles>"
  "5361e68128bb6d1538c253b719002d51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmAngles)))
  "Returns md5sum for a message object of type 'ArmAngles"
  "5361e68128bb6d1538c253b719002d51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmAngles>)))
  "Returns full string definition for message of type '<ArmAngles>"
  (cl:format cl:nil "float32 shoulderOrientation~%float32 shoulderElevation~%float32 elbow~%float32 wristOrientation~%float32 wristElevation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmAngles)))
  "Returns full string definition for message of type 'ArmAngles"
  (cl:format cl:nil "float32 shoulderOrientation~%float32 shoulderElevation~%float32 elbow~%float32 wristOrientation~%float32 wristElevation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmAngles>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmAngles>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmAngles
    (cl:cons ':shoulderOrientation (shoulderOrientation msg))
    (cl:cons ':shoulderElevation (shoulderElevation msg))
    (cl:cons ':elbow (elbow msg))
    (cl:cons ':wristOrientation (wristOrientation msg))
    (cl:cons ':wristElevation (wristElevation msg))
))
