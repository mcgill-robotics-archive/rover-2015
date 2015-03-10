; Auto-generated. Do not edit!


(cl:in-package control_systems-msg)


;//! \htmlinclude Moving.msg.html

(cl:defclass <Moving> (roslisp-msg-protocol:ros-message)
  ((move
    :reader move
    :initarg :move
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Moving (<Moving>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Moving>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Moving)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_systems-msg:<Moving> is deprecated: use control_systems-msg:Moving instead.")))

(cl:ensure-generic-function 'move-val :lambda-list '(m))
(cl:defmethod move-val ((m <Moving>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_systems-msg:move-val is deprecated.  Use control_systems-msg:move instead.")
  (move m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Moving>) ostream)
  "Serializes a message object of type '<Moving>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'move) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Moving>) istream)
  "Deserializes a message object of type '<Moving>"
    (cl:setf (cl:slot-value msg 'move) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Moving>)))
  "Returns string type for a message object of type '<Moving>"
  "control_systems/Moving")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Moving)))
  "Returns string type for a message object of type 'Moving"
  "control_systems/Moving")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Moving>)))
  "Returns md5sum for a message object of type '<Moving>"
  "52d810f536f472d08719561a224ff00c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Moving)))
  "Returns md5sum for a message object of type 'Moving"
  "52d810f536f472d08719561a224ff00c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Moving>)))
  "Returns full string definition for message of type '<Moving>"
  (cl:format cl:nil "bool move~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Moving)))
  "Returns full string definition for message of type 'Moving"
  (cl:format cl:nil "bool move~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Moving>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Moving>))
  "Converts a ROS message object to a list"
  (cl:list 'Moving
    (cl:cons ':move (move msg))
))
