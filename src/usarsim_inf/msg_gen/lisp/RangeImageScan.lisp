; Auto-generated. Do not edit!


(cl:in-package usarsim_inf-msg)


;//! \htmlinclude RangeImageScan.msg.html

(cl:defclass <RangeImageScan> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dummy_param
    :reader dummy_param
    :initarg :dummy_param
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RangeImageScan (<RangeImageScan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RangeImageScan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RangeImageScan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usarsim_inf-msg:<RangeImageScan> is deprecated: use usarsim_inf-msg:RangeImageScan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RangeImageScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:header-val is deprecated.  Use usarsim_inf-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dummy_param-val :lambda-list '(m))
(cl:defmethod dummy_param-val ((m <RangeImageScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usarsim_inf-msg:dummy_param-val is deprecated.  Use usarsim_inf-msg:dummy_param instead.")
  (dummy_param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RangeImageScan>) ostream)
  "Serializes a message object of type '<RangeImageScan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dummy_param)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RangeImageScan>) istream)
  "Deserializes a message object of type '<RangeImageScan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dummy_param)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RangeImageScan>)))
  "Returns string type for a message object of type '<RangeImageScan>"
  "usarsim_inf/RangeImageScan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RangeImageScan)))
  "Returns string type for a message object of type 'RangeImageScan"
  "usarsim_inf/RangeImageScan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RangeImageScan>)))
  "Returns md5sum for a message object of type '<RangeImageScan>"
  "2c8ee61fc395f79ed39913d793b03a68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RangeImageScan)))
  "Returns md5sum for a message object of type 'RangeImageScan"
  "2c8ee61fc395f79ed39913d793b03a68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RangeImageScan>)))
  "Returns full string definition for message of type '<RangeImageScan>"
  (cl:format cl:nil "Header header~%uint8 dummy_param~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RangeImageScan)))
  "Returns full string definition for message of type 'RangeImageScan"
  (cl:format cl:nil "Header header~%uint8 dummy_param~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RangeImageScan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RangeImageScan>))
  "Converts a ROS message object to a list"
  (cl:list 'RangeImageScan
    (cl:cons ':header (header msg))
    (cl:cons ':dummy_param (dummy_param msg))
))
