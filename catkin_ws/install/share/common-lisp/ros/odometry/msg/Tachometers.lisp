; Auto-generated. Do not edit!


(cl:in-package odometry-msg)


;//! \htmlinclude Tachometers.msg.html

(cl:defclass <Tachometers> (roslisp-msg-protocol:ros-message)
  ((tachoFL
    :reader tachoFL
    :initarg :tachoFL
    :type cl:integer
    :initform 0)
   (tachoML
    :reader tachoML
    :initarg :tachoML
    :type cl:integer
    :initform 0)
   (tachoBL
    :reader tachoBL
    :initarg :tachoBL
    :type cl:integer
    :initform 0)
   (tachoFR
    :reader tachoFR
    :initarg :tachoFR
    :type cl:integer
    :initform 0)
   (tachoMR
    :reader tachoMR
    :initarg :tachoMR
    :type cl:integer
    :initform 0)
   (tachoBR
    :reader tachoBR
    :initarg :tachoBR
    :type cl:integer
    :initform 0))
)

(cl:defclass Tachometers (<Tachometers>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tachometers>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tachometers)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odometry-msg:<Tachometers> is deprecated: use odometry-msg:Tachometers instead.")))

(cl:ensure-generic-function 'tachoFL-val :lambda-list '(m))
(cl:defmethod tachoFL-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoFL-val is deprecated.  Use odometry-msg:tachoFL instead.")
  (tachoFL m))

(cl:ensure-generic-function 'tachoML-val :lambda-list '(m))
(cl:defmethod tachoML-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoML-val is deprecated.  Use odometry-msg:tachoML instead.")
  (tachoML m))

(cl:ensure-generic-function 'tachoBL-val :lambda-list '(m))
(cl:defmethod tachoBL-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoBL-val is deprecated.  Use odometry-msg:tachoBL instead.")
  (tachoBL m))

(cl:ensure-generic-function 'tachoFR-val :lambda-list '(m))
(cl:defmethod tachoFR-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoFR-val is deprecated.  Use odometry-msg:tachoFR instead.")
  (tachoFR m))

(cl:ensure-generic-function 'tachoMR-val :lambda-list '(m))
(cl:defmethod tachoMR-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoMR-val is deprecated.  Use odometry-msg:tachoMR instead.")
  (tachoMR m))

(cl:ensure-generic-function 'tachoBR-val :lambda-list '(m))
(cl:defmethod tachoBR-val ((m <Tachometers>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-msg:tachoBR-val is deprecated.  Use odometry-msg:tachoBR instead.")
  (tachoBR m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tachometers>) ostream)
  "Serializes a message object of type '<Tachometers>"
  (cl:let* ((signed (cl:slot-value msg 'tachoFL)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tachoML)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tachoBL)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tachoFR)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tachoMR)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tachoBR)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tachometers>) istream)
  "Deserializes a message object of type '<Tachometers>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoFL) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoML) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoBL) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoFR) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoMR) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tachoBR) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tachometers>)))
  "Returns string type for a message object of type '<Tachometers>"
  "odometry/Tachometers")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tachometers)))
  "Returns string type for a message object of type 'Tachometers"
  "odometry/Tachometers")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tachometers>)))
  "Returns md5sum for a message object of type '<Tachometers>"
  "fb233116002aa847d65f9b3f6ce21653")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tachometers)))
  "Returns md5sum for a message object of type 'Tachometers"
  "fb233116002aa847d65f9b3f6ce21653")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tachometers>)))
  "Returns full string definition for message of type '<Tachometers>"
  (cl:format cl:nil "int32 tachoFL~%int32 tachoML~%int32 tachoBL~%int32 tachoFR~%int32 tachoMR~%int32 tachoBR~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tachometers)))
  "Returns full string definition for message of type 'Tachometers"
  (cl:format cl:nil "int32 tachoFL~%int32 tachoML~%int32 tachoBL~%int32 tachoFR~%int32 tachoMR~%int32 tachoBR~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tachometers>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tachometers>))
  "Converts a ROS message object to a list"
  (cl:list 'Tachometers
    (cl:cons ':tachoFL (tachoFL msg))
    (cl:cons ':tachoML (tachoML msg))
    (cl:cons ':tachoBL (tachoBL msg))
    (cl:cons ':tachoFR (tachoFR msg))
    (cl:cons ':tachoMR (tachoMR msg))
    (cl:cons ':tachoBR (tachoBR msg))
))
