; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude PanTiltZoom.msg.html

(cl:defclass <PanTiltZoom> (roslisp-msg-protocol:ros-message)
  ((pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0)
   (zoom
    :reader zoom
    :initarg :zoom
    :type cl:float
    :initform 0.0))
)

(cl:defclass PanTiltZoom (<PanTiltZoom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PanTiltZoom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PanTiltZoom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<PanTiltZoom> is deprecated: use control_systems-msg:PanTiltZoom instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <PanTiltZoom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:pan-val is deprecated.  Use control_systems-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <PanTiltZoom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:tilt-val is deprecated.  Use control_systems-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'zoom-val :lambda-list '(m))
(cl:defmethod zoom-val ((m <PanTiltZoom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:zoom-val is deprecated.  Use control_systems-msg:zoom instead.")
  (zoom m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PanTiltZoom>) ostream)
  "Serializes a message object of type '<PanTiltZoom>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'zoom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PanTiltZoom>) istream)
  "Deserializes a message object of type '<PanTiltZoom>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zoom) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PanTiltZoom>)))
  "Returns string type for a message object of type '<PanTiltZoom>"
  "control_systems/PanTiltZoom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PanTiltZoom)))
  "Returns string type for a message object of type 'PanTiltZoom"
  "control_systems/PanTiltZoom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PanTiltZoom>)))
  "Returns md5sum for a message object of type '<PanTiltZoom>"
  "eef4827ef5b8ba89a8c9fa0810a30294")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PanTiltZoom)))
  "Returns md5sum for a message object of type 'PanTiltZoom"
  "eef4827ef5b8ba89a8c9fa0810a30294")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PanTiltZoom>)))
  "Returns full string definition for message of type '<PanTiltZoom>"
  (cl:format cl:nil "float32 pan~%float32 tilt~%float32 zoom~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PanTiltZoom)))
  "Returns full string definition for message of type 'PanTiltZoom"
  (cl:format cl:nil "float32 pan~%float32 tilt~%float32 zoom~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PanTiltZoom>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PanTiltZoom>))
  "Converts a ROS message object to a list"
  (cl:list 'PanTiltZoom
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':zoom (zoom msg))
))
