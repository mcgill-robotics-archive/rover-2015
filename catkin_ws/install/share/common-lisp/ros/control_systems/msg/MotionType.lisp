; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude MotionType.msg.html

(cl:defclass <MotionType> (roslisp-msg-protocol:ros-message)
  ((ACKERMANN
    :reader ACKERMANN
    :initarg :ACKERMANN
    :type cl:fixnum
    :initform 0)
   (POINT
    :reader POINT
    :initarg :POINT
    :type cl:fixnum
    :initform 0)
   (TRANSLATORY
    :reader TRANSLATORY
    :initarg :TRANSLATORY
    :type cl:fixnum
    :initform 0)
   (SWERVE
    :reader SWERVE
    :initarg :SWERVE
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotionType (<MotionType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<MotionType> is deprecated: use control_systems-msg:MotionType instead.")))

(cl:ensure-generic-function 'ACKERMANN-val :lambda-list '(m))
(cl:defmethod ACKERMANN-val ((m <MotionType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:ACKERMANN-val is deprecated.  Use control_systems-msg:ACKERMANN instead.")
  (ACKERMANN m))

(cl:ensure-generic-function 'POINT-val :lambda-list '(m))
(cl:defmethod POINT-val ((m <MotionType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:POINT-val is deprecated.  Use control_systems-msg:POINT instead.")
  (POINT m))

(cl:ensure-generic-function 'TRANSLATORY-val :lambda-list '(m))
(cl:defmethod TRANSLATORY-val ((m <MotionType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:TRANSLATORY-val is deprecated.  Use control_systems-msg:TRANSLATORY instead.")
  (TRANSLATORY m))

(cl:ensure-generic-function 'SWERVE-val :lambda-list '(m))
(cl:defmethod SWERVE-val ((m <MotionType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:SWERVE-val is deprecated.  Use control_systems-msg:SWERVE instead.")
  (SWERVE m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionType>) ostream)
  "Serializes a message object of type '<MotionType>"
  (cl:let* ((signed (cl:slot-value msg 'ACKERMANN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'POINT)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'TRANSLATORY)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'SWERVE)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionType>) istream)
  "Deserializes a message object of type '<MotionType>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ACKERMANN) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'POINT) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TRANSLATORY) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'SWERVE) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionType>)))
  "Returns string type for a message object of type '<MotionType>"
  "control_systems/MotionType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionType)))
  "Returns string type for a message object of type 'MotionType"
  "control_systems/MotionType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionType>)))
  "Returns md5sum for a message object of type '<MotionType>"
  "00aa9f900bb945bea6467931de7f6956")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionType)))
  "Returns md5sum for a message object of type 'MotionType"
  "00aa9f900bb945bea6467931de7f6956")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionType>)))
  "Returns full string definition for message of type '<MotionType>"
  (cl:format cl:nil "int8 ACKERMANN~%int8 POINT~%int8 TRANSLATORY~%int8 SWERVE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionType)))
  "Returns full string definition for message of type 'MotionType"
  (cl:format cl:nil "int8 ACKERMANN~%int8 POINT~%int8 TRANSLATORY~%int8 SWERVE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionType>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionType>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionType
    (cl:cons ':ACKERMANN (ACKERMANN msg))
    (cl:cons ':POINT (POINT msg))
    (cl:cons ':TRANSLATORY (TRANSLATORY msg))
    (cl:cons ':SWERVE (SWERVE msg))
))
