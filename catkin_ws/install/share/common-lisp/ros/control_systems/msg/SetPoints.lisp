; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude SetPoints.msg.html

(cl:defclass <SetPoints> (roslisp-msg-protocol:ros-message)
  ((thetaFL
    :reader thetaFL
    :initarg :thetaFL
    :type cl:float
    :initform 0.0)
   (thetaFR
    :reader thetaFR
    :initarg :thetaFR
    :type cl:float
    :initform 0.0)
   (thetaRL
    :reader thetaRL
    :initarg :thetaRL
    :type cl:float
    :initform 0.0)
   (thetaRR
    :reader thetaRR
    :initarg :thetaRR
    :type cl:float
    :initform 0.0)
   (speedFL
    :reader speedFL
    :initarg :speedFL
    :type cl:float
    :initform 0.0)
   (speedFR
    :reader speedFR
    :initarg :speedFR
    :type cl:float
    :initform 0.0)
   (speedML
    :reader speedML
    :initarg :speedML
    :type cl:float
    :initform 0.0)
   (speedMR
    :reader speedMR
    :initarg :speedMR
    :type cl:float
    :initform 0.0)
   (speedRL
    :reader speedRL
    :initarg :speedRL
    :type cl:float
    :initform 0.0)
   (speedRR
    :reader speedRR
    :initarg :speedRR
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetPoints (<SetPoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<SetPoints> is deprecated: use control_systems-msg:SetPoints instead.")))

(cl:ensure-generic-function 'thetaFL-val :lambda-list '(m))
(cl:defmethod thetaFL-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:thetaFL-val is deprecated.  Use control_systems-msg:thetaFL instead.")
  (thetaFL m))

(cl:ensure-generic-function 'thetaFR-val :lambda-list '(m))
(cl:defmethod thetaFR-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:thetaFR-val is deprecated.  Use control_systems-msg:thetaFR instead.")
  (thetaFR m))

(cl:ensure-generic-function 'thetaRL-val :lambda-list '(m))
(cl:defmethod thetaRL-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:thetaRL-val is deprecated.  Use control_systems-msg:thetaRL instead.")
  (thetaRL m))

(cl:ensure-generic-function 'thetaRR-val :lambda-list '(m))
(cl:defmethod thetaRR-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:thetaRR-val is deprecated.  Use control_systems-msg:thetaRR instead.")
  (thetaRR m))

(cl:ensure-generic-function 'speedFL-val :lambda-list '(m))
(cl:defmethod speedFL-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedFL-val is deprecated.  Use control_systems-msg:speedFL instead.")
  (speedFL m))

(cl:ensure-generic-function 'speedFR-val :lambda-list '(m))
(cl:defmethod speedFR-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedFR-val is deprecated.  Use control_systems-msg:speedFR instead.")
  (speedFR m))

(cl:ensure-generic-function 'speedML-val :lambda-list '(m))
(cl:defmethod speedML-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedML-val is deprecated.  Use control_systems-msg:speedML instead.")
  (speedML m))

(cl:ensure-generic-function 'speedMR-val :lambda-list '(m))
(cl:defmethod speedMR-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedMR-val is deprecated.  Use control_systems-msg:speedMR instead.")
  (speedMR m))

(cl:ensure-generic-function 'speedRL-val :lambda-list '(m))
(cl:defmethod speedRL-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedRL-val is deprecated.  Use control_systems-msg:speedRL instead.")
  (speedRL m))

(cl:ensure-generic-function 'speedRR-val :lambda-list '(m))
(cl:defmethod speedRR-val ((m <SetPoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:speedRR-val is deprecated.  Use control_systems-msg:speedRR instead.")
  (speedRR m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPoints>) ostream)
  "Serializes a message object of type '<SetPoints>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaFL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaFR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaRL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thetaRR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedFL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedFR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedML))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedMR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedRL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speedRR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPoints>) istream)
  "Deserializes a message object of type '<SetPoints>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaFL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaFR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaRL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thetaRR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedFL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedFR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedML) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedMR) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedRL) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speedRR) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPoints>)))
  "Returns string type for a message object of type '<SetPoints>"
  "control_systems/SetPoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPoints)))
  "Returns string type for a message object of type 'SetPoints"
  "control_systems/SetPoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPoints>)))
  "Returns md5sum for a message object of type '<SetPoints>"
  "8081cfe954eacee416d55d865a3b7a97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPoints)))
  "Returns md5sum for a message object of type 'SetPoints"
  "8081cfe954eacee416d55d865a3b7a97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPoints>)))
  "Returns full string definition for message of type '<SetPoints>"
  (cl:format cl:nil "float32 thetaFL~%float32 thetaFR~%float32 thetaRL~%float32 thetaRR~%float32 speedFL~%float32 speedFR~%float32 speedML~%float32 speedMR~%float32 speedRL~%float32 speedRR~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPoints)))
  "Returns full string definition for message of type 'SetPoints"
  (cl:format cl:nil "float32 thetaFL~%float32 thetaFR~%float32 thetaRL~%float32 thetaRR~%float32 speedFL~%float32 speedFR~%float32 speedML~%float32 speedMR~%float32 speedRL~%float32 speedRR~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPoints>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPoints>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPoints
    (cl:cons ':thetaFL (thetaFL msg))
    (cl:cons ':thetaFR (thetaFR msg))
    (cl:cons ':thetaRL (thetaRL msg))
    (cl:cons ':thetaRR (thetaRR msg))
    (cl:cons ':speedFL (speedFL msg))
    (cl:cons ':speedFR (speedFR msg))
    (cl:cons ':speedML (speedML msg))
    (cl:cons ':speedMR (speedMR msg))
    (cl:cons ':speedRL (speedRL msg))
    (cl:cons ':speedRR (speedRR msg))
))
