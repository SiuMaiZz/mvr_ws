; Auto-generated. Do not edit!


(cl:in-package mvr_robot_control-msg)


;//! \htmlinclude MotorData.msg.html

(cl:defclass <MotorData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (kp_
    :reader kp_
    :initarg :kp_
    :type cl:float
    :initform 0.0)
   (kd_
    :reader kd_
    :initarg :kd_
    :type cl:float
    :initform 0.0)
   (ff_
    :reader ff_
    :initarg :ff_
    :type cl:float
    :initform 0.0)
   (pos_des_
    :reader pos_des_
    :initarg :pos_des_
    :type cl:float
    :initform 0.0)
   (vel_des_
    :reader vel_des_
    :initarg :vel_des_
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotorData (<MotorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mvr_robot_control-msg:<MotorData> is deprecated: use mvr_robot_control-msg:MotorData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:header-val is deprecated.  Use mvr_robot_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:id-val is deprecated.  Use mvr_robot_control-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'kp_-val :lambda-list '(m))
(cl:defmethod kp_-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:kp_-val is deprecated.  Use mvr_robot_control-msg:kp_ instead.")
  (kp_ m))

(cl:ensure-generic-function 'kd_-val :lambda-list '(m))
(cl:defmethod kd_-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:kd_-val is deprecated.  Use mvr_robot_control-msg:kd_ instead.")
  (kd_ m))

(cl:ensure-generic-function 'ff_-val :lambda-list '(m))
(cl:defmethod ff_-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:ff_-val is deprecated.  Use mvr_robot_control-msg:ff_ instead.")
  (ff_ m))

(cl:ensure-generic-function 'pos_des_-val :lambda-list '(m))
(cl:defmethod pos_des_-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:pos_des_-val is deprecated.  Use mvr_robot_control-msg:pos_des_ instead.")
  (pos_des_ m))

(cl:ensure-generic-function 'vel_des_-val :lambda-list '(m))
(cl:defmethod vel_des_-val ((m <MotorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:vel_des_-val is deprecated.  Use mvr_robot_control-msg:vel_des_ instead.")
  (vel_des_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorData>) ostream)
  "Serializes a message object of type '<MotorData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kp_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'kd_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ff_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_des_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel_des_))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorData>) istream)
  "Deserializes a message object of type '<MotorData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp_) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd_) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ff_) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_des_) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel_des_) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorData>)))
  "Returns string type for a message object of type '<MotorData>"
  "mvr_robot_control/MotorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorData)))
  "Returns string type for a message object of type 'MotorData"
  "mvr_robot_control/MotorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorData>)))
  "Returns md5sum for a message object of type '<MotorData>"
  "6bed2913051bec1ee3cdf057ad08a302")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorData)))
  "Returns md5sum for a message object of type 'MotorData"
  "6bed2913051bec1ee3cdf057ad08a302")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorData>)))
  "Returns full string definition for message of type '<MotorData>"
  (cl:format cl:nil "Header header~%    int16 id~%    float32 kp_~%    float32 kd_~%    float32 ff_~%    float32 pos_des_ ~%    float32 vel_des_~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorData)))
  "Returns full string definition for message of type 'MotorData"
  (cl:format cl:nil "Header header~%    int16 id~%    float32 kp_~%    float32 kd_~%    float32 ff_~%    float32 pos_des_ ~%    float32 vel_des_~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorData>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorData
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':kp_ (kp_ msg))
    (cl:cons ':kd_ (kd_ msg))
    (cl:cons ':ff_ (ff_ msg))
    (cl:cons ':pos_des_ (pos_des_ msg))
    (cl:cons ':vel_des_ (vel_des_ msg))
))
