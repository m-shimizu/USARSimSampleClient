; Auto-generated. Do not edit!


(cl:in-package usarsim_inf-msg)


;//! \htmlinclude ToolchangerStatus.msg.html

(cl:defclass <ToolchangerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (effector_status
    :reader effector_status
    :initarg :effector_status
    :type usarsim_inf-msg:EffectorStatus
    :initform (cl:make-instance 'usarsim_inf-msg:EffectorStatus))
   (tool_type
    :reader tool_type
    :initarg :tool_type
    :type usarsim_inf-msg:ToolType
    :initform (cl:make-instance 'usarsim_inf-msg:ToolType)))
)

(cl:defclass ToolchangerStatus (<ToolchangerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToolchangerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToolchangerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usarsim_inf-msg:<ToolchangerStatus> is deprecated: use usarsim_inf-msg:ToolchangerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ToolchangerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:header-val is deprecated.  Use usarsim_inf-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'effector_status-val :lambda-list '(m))
(cl:defmethod effector_status-val ((m <ToolchangerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:effector_status-val is deprecated.  Use usarsim_inf-msg:effector_status instead.")
  (effector_status m))

(cl:ensure-generic-function 'tool_type-val :lambda-list '(m))
(cl:defmethod tool_type-val ((m <ToolchangerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:tool_type-val is deprecated.  Use usarsim_inf-msg:tool_type instead.")
  (tool_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToolchangerStatus>) ostream)
  "Serializes a message object of type '<ToolchangerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'effector_status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tool_type) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToolchangerStatus>) istream)
  "Deserializes a message object of type '<ToolchangerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'effector_status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tool_type) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToolchangerStatus>)))
  "Returns string type for a message object of type '<ToolchangerStatus>"
  "usarsim_inf/ToolchangerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToolchangerStatus)))
  "Returns string type for a message object of type 'ToolchangerStatus"
  "usarsim_inf/ToolchangerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToolchangerStatus>)))
  "Returns md5sum for a message object of type '<ToolchangerStatus>"
  "8b5a06cc589e1ab2b44a5eec0fd708b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToolchangerStatus)))
  "Returns md5sum for a message object of type 'ToolchangerStatus"
  "8b5a06cc589e1ab2b44a5eec0fd708b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToolchangerStatus>)))
  "Returns full string definition for message of type '<ToolchangerStatus>"
  (cl:format cl:nil "Header header~%EffectorStatus effector_status~%ToolType tool_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: usarsim_inf/EffectorStatus~%Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: usarsim_inf/ToolType~%uint8 type~%uint8 UNKNOWN=0~%uint8 GRIPPER=1~%uint8 VACUUM=2~%uint8 TOOLCHANGER=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToolchangerStatus)))
  "Returns full string definition for message of type 'ToolchangerStatus"
  (cl:format cl:nil "Header header~%EffectorStatus effector_status~%ToolType tool_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: usarsim_inf/EffectorStatus~%Header header~%uint8 state~%uint8 OPEN=0~%uint8 CLOSE=1~%~%================================================================================~%MSG: usarsim_inf/ToolType~%uint8 type~%uint8 UNKNOWN=0~%uint8 GRIPPER=1~%uint8 VACUUM=2~%uint8 TOOLCHANGER=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToolchangerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'effector_status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tool_type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToolchangerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ToolchangerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':effector_status (effector_status msg))
    (cl:cons ':tool_type (tool_type msg))
))
