; Auto-generated. Do not edit!


(cl:in-package usarsim_inf-msg)


;//! \htmlinclude EffectorCommand.msg.html

(cl:defclass <EffectorCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass EffectorCommand (<EffectorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EffectorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EffectorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usarsim_inf-msg:<EffectorCommand> is deprecated: use usarsim_inf-msg:EffectorCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EffectorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:header-val is deprecated.  Use usarsim_inf-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <EffectorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:state-val is deprecated.  Use usarsim_inf-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EffectorCommand>)))
    "Constants for message type '<EffectorCommand>"
  '((:OPEN . 0)
    (:CLOSE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EffectorCommand)))
    "Constants for message type 'EffectorCommand"
  '((:OPEN . 0)
    (:CLOSE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EffectorCommand>) ostream)
  "Serializes a message object of type '<EffectorCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EffectorCommand>) istream)
  "Deserializes a message object of type '<EffectorCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EffectorCommand>)))
  "Returns string type for a message object of type '<EffectorCommand>"
  "usarsim_inf/EffectorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EffectorCommand)))
  "Returns string type for a message object of type 'EffectorCommand"
  "usarsim_inf/EffectorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EffectorCommand>)))
  "Returns md5sum for a message object of type '<EffectorCommand>"
  "98e54041312d158639ac0a333985e48c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EffectorCommand)))
  "Returns md5sum for a message object of type 'EffectorCommand"
  "98e54041312d158639ac0a333985e48c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EffectorCommand>)))
  "Returns full string definition for message of type '<EffectorCommand>"
  (cl:format cl:nil "Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EffectorCommand)))
  "Returns full string definition for message of type 'EffectorCommand"
  (cl:format cl:nil "Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EffectorCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EffectorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'EffectorCommand
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
