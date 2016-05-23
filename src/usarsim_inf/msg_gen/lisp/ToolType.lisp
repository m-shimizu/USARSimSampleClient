; Auto-generated. Do not edit!


(cl:in-package usarsim_inf-msg)


;//! \htmlinclude ToolType.msg.html

(cl:defclass <ToolType> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ToolType (<ToolType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToolType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToolType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usarsim_inf-msg:<ToolType> is deprecated: use usarsim_inf-msg:ToolType instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ToolType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:type-val is deprecated.  Use usarsim_inf-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ToolType>)))
    "Constants for message type '<ToolType>"
  '((:UNKNOWN . 0)
    (:GRIPPER . 1)
    (:VACUUM . 2)
    (:TOOLCHANGER . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ToolType)))
    "Constants for message type 'ToolType"
  '((:UNKNOWN . 0)
    (:GRIPPER . 1)
    (:VACUUM . 2)
    (:TOOLCHANGER . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToolType>) ostream)
  "Serializes a message object of type '<ToolType>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToolType>) istream)
  "Deserializes a message object of type '<ToolType>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToolType>)))
  "Returns string type for a message object of type '<ToolType>"
  "usarsim_inf/ToolType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToolType)))
  "Returns string type for a message object of type 'ToolType"
  "usarsim_inf/ToolType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToolType>)))
  "Returns md5sum for a message object of type '<ToolType>"
  "9d49e02052e33b674525426c5e7bd1f6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToolType)))
  "Returns md5sum for a message object of type 'ToolType"
  "9d49e02052e33b674525426c5e7bd1f6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToolType>)))
  "Returns full string definition for message of type '<ToolType>"
  (cl:format cl:nil "uint8 type~%uint8 UNKNOWN=0~%uint8 GRIPPER=1~%uint8 VACUUM=2~%uint8 TOOLCHANGER=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToolType)))
  "Returns full string definition for message of type 'ToolType"
  (cl:format cl:nil "uint8 type~%uint8 UNKNOWN=0~%uint8 GRIPPER=1~%uint8 VACUUM=2~%uint8 TOOLCHANGER=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToolType>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToolType>))
  "Converts a ROS message object to a list"
  (cl:list 'ToolType
    (cl:cons ':type (type msg))
))
