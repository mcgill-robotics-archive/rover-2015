; Auto-generated. Do not edit!


(cl:in-package odometry-srv)


;//! \htmlinclude DisplacementMiddleWheels-request.msg.html

(cl:defclass <DisplacementMiddleWheels-request> (roslisp-msg-protocol:ros-message)
  ((diffTachoLeft
    :reader diffTachoLeft
    :initarg :diffTachoLeft
    :type cl:float
    :initform 0.0)
   (diffTachoRight
    :reader diffTachoRight
    :initarg :diffTachoRight
    :type cl:float
    :initform 0.0))
)

(cl:defclass DisplacementMiddleWheels-request (<DisplacementMiddleWheels-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisplacementMiddleWheels-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisplacementMiddleWheels-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odometry-srv:<DisplacementMiddleWheels-request> is deprecated: use odometry-srv:DisplacementMiddleWheels-request instead.")))

(cl:ensure-generic-function 'diffTachoLeft-val :lambda-list '(m))
(cl:defmethod diffTachoLeft-val ((m <DisplacementMiddleWheels-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-srv:diffTachoLeft-val is deprecated.  Use odometry-srv:diffTachoLeft instead.")
  (diffTachoLeft m))

(cl:ensure-generic-function 'diffTachoRight-val :lambda-list '(m))
(cl:defmethod diffTachoRight-val ((m <DisplacementMiddleWheels-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-srv:diffTachoRight-val is deprecated.  Use odometry-srv:diffTachoRight instead.")
  (diffTachoRight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisplacementMiddleWheels-request>) ostream)
  "Serializes a message object of type '<DisplacementMiddleWheels-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'diffTachoLeft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'diffTachoRight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisplacementMiddleWheels-request>) istream)
  "Deserializes a message object of type '<DisplacementMiddleWheels-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'diffTachoLeft) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'diffTachoRight) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisplacementMiddleWheels-request>)))
  "Returns string type for a service object of type '<DisplacementMiddleWheels-request>"
  "odometry/DisplacementMiddleWheelsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisplacementMiddleWheels-request)))
  "Returns string type for a service object of type 'DisplacementMiddleWheels-request"
  "odometry/DisplacementMiddleWheelsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisplacementMiddleWheels-request>)))
  "Returns md5sum for a message object of type '<DisplacementMiddleWheels-request>"
  "298fb6689bea16a3681240b79db77220")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisplacementMiddleWheels-request)))
  "Returns md5sum for a message object of type 'DisplacementMiddleWheels-request"
  "298fb6689bea16a3681240b79db77220")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisplacementMiddleWheels-request>)))
  "Returns full string definition for message of type '<DisplacementMiddleWheels-request>"
  (cl:format cl:nil "float32 diffTachoLeft~%float32 diffTachoRight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisplacementMiddleWheels-request)))
  "Returns full string definition for message of type 'DisplacementMiddleWheels-request"
  (cl:format cl:nil "float32 diffTachoLeft~%float32 diffTachoRight~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisplacementMiddleWheels-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisplacementMiddleWheels-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DisplacementMiddleWheels-request
    (cl:cons ':diffTachoLeft (diffTachoLeft msg))
    (cl:cons ':diffTachoRight (diffTachoRight msg))
))
;//! \htmlinclude DisplacementMiddleWheels-response.msg.html

(cl:defclass <DisplacementMiddleWheels-response> (roslisp-msg-protocol:ros-message)
  ((diffPosition
    :reader diffPosition
    :initarg :diffPosition
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass DisplacementMiddleWheels-response (<DisplacementMiddleWheels-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DisplacementMiddleWheels-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DisplacementMiddleWheels-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odometry-srv:<DisplacementMiddleWheels-response> is deprecated: use odometry-srv:DisplacementMiddleWheels-response instead.")))

(cl:ensure-generic-function 'diffPosition-val :lambda-list '(m))
(cl:defmethod diffPosition-val ((m <DisplacementMiddleWheels-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odometry-srv:diffPosition-val is deprecated.  Use odometry-srv:diffPosition instead.")
  (diffPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DisplacementMiddleWheels-response>) ostream)
  "Serializes a message object of type '<DisplacementMiddleWheels-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'diffPosition) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DisplacementMiddleWheels-response>) istream)
  "Deserializes a message object of type '<DisplacementMiddleWheels-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'diffPosition) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DisplacementMiddleWheels-response>)))
  "Returns string type for a service object of type '<DisplacementMiddleWheels-response>"
  "odometry/DisplacementMiddleWheelsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisplacementMiddleWheels-response)))
  "Returns string type for a service object of type 'DisplacementMiddleWheels-response"
  "odometry/DisplacementMiddleWheelsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DisplacementMiddleWheels-response>)))
  "Returns md5sum for a message object of type '<DisplacementMiddleWheels-response>"
  "298fb6689bea16a3681240b79db77220")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DisplacementMiddleWheels-response)))
  "Returns md5sum for a message object of type 'DisplacementMiddleWheels-response"
  "298fb6689bea16a3681240b79db77220")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DisplacementMiddleWheels-response>)))
  "Returns full string definition for message of type '<DisplacementMiddleWheels-response>"
  (cl:format cl:nil "geometry_msgs/Pose diffPosition~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DisplacementMiddleWheels-response)))
  "Returns full string definition for message of type 'DisplacementMiddleWheels-response"
  (cl:format cl:nil "geometry_msgs/Pose diffPosition~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DisplacementMiddleWheels-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'diffPosition))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DisplacementMiddleWheels-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DisplacementMiddleWheels-response
    (cl:cons ':diffPosition (diffPosition msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DisplacementMiddleWheels)))
  'DisplacementMiddleWheels-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DisplacementMiddleWheels)))
  'DisplacementMiddleWheels-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DisplacementMiddleWheels)))
  "Returns string type for a service object of type '<DisplacementMiddleWheels>"
  "odometry/DisplacementMiddleWheels")