; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude ArmMotion.msg.html

(cl:defclass <ArmMotion> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (on
    :reader on
    :initarg :on
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmMotion (<ArmMotion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmMotion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmMotion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<ArmMotion> is deprecated: use control_systems-msg:ArmMotion instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ArmMotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:x-val is deprecated.  Use control_systems-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ArmMotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:y-val is deprecated.  Use control_systems-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <ArmMotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:theta-val is deprecated.  Use control_systems-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'on-val :lambda-list '(m))
(cl:defmethod on-val ((m <ArmMotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:on-val is deprecated.  Use control_systems-msg:on instead.")
  (on m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmMotion>) ostream)
  "Serializes a message object of type '<ArmMotion>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'on) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmMotion>) istream)
  "Deserializes a message object of type '<ArmMotion>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'on) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmMotion>)))
  "Returns string type for a message object of type '<ArmMotion>"
  "control_systems/ArmMotion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmMotion)))
  "Returns string type for a message object of type 'ArmMotion"
  "control_systems/ArmMotion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmMotion>)))
  "Returns md5sum for a message object of type '<ArmMotion>"
  "6b1436c74180e80c20cec25884b236eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmMotion)))
  "Returns md5sum for a message object of type 'ArmMotion"
  "6b1436c74180e80c20cec25884b236eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmMotion>)))
  "Returns full string definition for message of type '<ArmMotion>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmMotion)))
  "Returns full string definition for message of type 'ArmMotion"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%bool on~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmMotion>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmMotion>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmMotion
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':on (on msg))
))
