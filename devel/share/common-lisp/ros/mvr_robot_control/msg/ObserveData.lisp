; Auto-generated. Do not edit!


(cl:in-package mvr_robot_control-msg)


;//! \htmlinclude ObserveData.msg.html

(cl:defclass <ObserveData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (imu_angular_vel
    :reader imu_angular_vel
    :initarg :imu_angular_vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (joint_pos
    :reader joint_pos
    :initarg :joint_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 1 :element-type 'cl:float :initial-element 0.0))
   (joint_vel
    :reader joint_vel
    :initarg :joint_vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 1 :element-type 'cl:float :initial-element 0.0))
   (commands
    :reader commands
    :initarg :commands
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (quat_float
    :reader quat_float
    :initarg :quat_float
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ObserveData (<ObserveData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObserveData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObserveData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mvr_robot_control-msg:<ObserveData> is deprecated: use mvr_robot_control-msg:ObserveData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:header-val is deprecated.  Use mvr_robot_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'imu_angular_vel-val :lambda-list '(m))
(cl:defmethod imu_angular_vel-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:imu_angular_vel-val is deprecated.  Use mvr_robot_control-msg:imu_angular_vel instead.")
  (imu_angular_vel m))

(cl:ensure-generic-function 'joint_pos-val :lambda-list '(m))
(cl:defmethod joint_pos-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:joint_pos-val is deprecated.  Use mvr_robot_control-msg:joint_pos instead.")
  (joint_pos m))

(cl:ensure-generic-function 'joint_vel-val :lambda-list '(m))
(cl:defmethod joint_vel-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:joint_vel-val is deprecated.  Use mvr_robot_control-msg:joint_vel instead.")
  (joint_vel m))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:commands-val is deprecated.  Use mvr_robot_control-msg:commands instead.")
  (commands m))

(cl:ensure-generic-function 'quat_float-val :lambda-list '(m))
(cl:defmethod quat_float-val ((m <ObserveData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:quat_float-val is deprecated.  Use mvr_robot_control-msg:quat_float instead.")
  (quat_float m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObserveData>) ostream)
  "Serializes a message object of type '<ObserveData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'imu_angular_vel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_pos))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_vel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'commands))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'quat_float))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObserveData>) istream)
  "Deserializes a message object of type '<ObserveData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'imu_angular_vel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'imu_angular_vel)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'joint_pos) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'joint_pos)))
    (cl:dotimes (i 1)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'joint_vel) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'joint_vel)))
    (cl:dotimes (i 1)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'commands) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'commands)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'quat_float) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quat_float)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObserveData>)))
  "Returns string type for a message object of type '<ObserveData>"
  "mvr_robot_control/ObserveData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObserveData)))
  "Returns string type for a message object of type 'ObserveData"
  "mvr_robot_control/ObserveData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObserveData>)))
  "Returns md5sum for a message object of type '<ObserveData>"
  "50eb62f4305c2c959b5563826e09f256")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObserveData)))
  "Returns md5sum for a message object of type 'ObserveData"
  "50eb62f4305c2c959b5563826e09f256")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObserveData>)))
  "Returns full string definition for message of type '<ObserveData>"
  (cl:format cl:nil "Header header~%    float32[3] imu_angular_vel~%    float32[1] joint_pos~%    float32[1] joint_vel~%    float32[3] commands~%    float32[4] quat_float~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObserveData)))
  "Returns full string definition for message of type 'ObserveData"
  (cl:format cl:nil "Header header~%    float32[3] imu_angular_vel~%    float32[1] joint_pos~%    float32[1] joint_vel~%    float32[3] commands~%    float32[4] quat_float~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObserveData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'imu_angular_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat_float) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObserveData>))
  "Converts a ROS message object to a list"
  (cl:list 'ObserveData
    (cl:cons ':header (header msg))
    (cl:cons ':imu_angular_vel (imu_angular_vel msg))
    (cl:cons ':joint_pos (joint_pos msg))
    (cl:cons ':joint_vel (joint_vel msg))
    (cl:cons ':commands (commands msg))
    (cl:cons ':quat_float (quat_float msg))
))
