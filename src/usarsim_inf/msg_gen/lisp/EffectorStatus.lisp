; Auto-generated. Do not edit!


(cl:in-package usarsim_inf-msg)


;//! \htmlinclude EffectorStatus.msg.html

(cl:defclass <EffectorStatus> (roslisp-msg-protocol:ros-message)
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

(cl:defclass EffectorStatus (<EffectorStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EffectorStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EffectorStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usarsim_inf-msg:<EffectorStatus> is deprecated: use usarsim_inf-msg:EffectorStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EffectorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:header-val is deprecated.  Use usarsim_inf-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <EffectorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:state-val is deprecated.  Use usarsim_inf-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EffectorStatus>)))
    "Constants for message type '<EffectorStatus>"
  '((:OPEN . 0)
    (:CLOSE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EffectorStatus)))
    "Constants for message type 'EffectorStatus"
  '((:OPEN . 0)
    (:CLOSE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EffectorStatus>) ostream)
  "Serializes a message object of type '<EffectorStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EffectorStatus>) istream)
  "Deserializes a message object of type '<EffectorStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EffectorStatus>)))
  "Returns string type for a message object of type '<EffectorStatus>"
  "usarsim_inf/EffectorStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EffectorStatus)))
  "Returns string type for a message object of type 'EffectorStatus"
  "usarsim_inf/EffectorStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EffectorStatus>)))
  "Returns md5sum for a message object of type '<EffectorStatus>"
  "98e54041312d158639ac0a333985e48c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EffectorStatus)))
  "Returns md5sum for a message object of type 'EffectorStatus"
  "98e54041312d158639ac0a333985e48c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EffectorStatus>)))
  "Returns full string definition for message of type '<EffectorStatus>"
  (cl:format cl:nil "Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EffectorStatus)))
  "Returns full string definition for message of type 'EffectorStatus"
  (cl:format cl:nil "Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EffectorStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EffectorStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'EffectorStatus
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
