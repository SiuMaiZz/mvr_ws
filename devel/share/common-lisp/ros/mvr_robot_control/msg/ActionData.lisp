; Auto-generated. Do not edit!


(cl:in-package mvr_robot_control-msg)


;//! \htmlinclude ActionData.msg.html

(cl:defclass <ActionData> (roslisp-msg-protocol:ros-message)
  ((joint_pos
    :reader joint_pos
    :initarg :joint_pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 1 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ActionData (<ActionData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mvr_robot_control-msg:<ActionData> is deprecated: use mvr_robot_control-msg:ActionData instead.")))

(cl:ensure-generic-function 'joint_pos-val :lambda-list '(m))
(cl:defmethod joint_pos-val ((m <ActionData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mvr_robot_control-msg:joint_pos-val is deprecated.  Use mvr_robot_control-msg:joint_pos instead.")
  (joint_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionData>) ostream)
  "Serializes a message object of type '<ActionData>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'joint_pos))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionData>) istream)
  "Deserializes a message object of type '<ActionData>"
  (cl:setf (cl:slot-value msg 'joint_pos) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'joint_pos)))
    (cl:dotimes (i 1)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionData>)))
  "Returns string type for a message object of type '<ActionData>"
  "mvr_robot_control/ActionData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionData)))
  "Returns string type for a message object of type 'ActionData"
  "mvr_robot_control/ActionData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionData>)))
  "Returns md5sum for a message object of type '<ActionData>"
  "a07255ea0da1785d1c58f5945b919bcb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionData)))
  "Returns md5sum for a message object of type 'ActionData"
  "a07255ea0da1785d1c58f5945b919bcb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionData>)))
  "Returns full string definition for message of type '<ActionData>"
  (cl:format cl:nil "float32[1] joint_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionData)))
  "Returns full string definition for message of type 'ActionData"
  (cl:format cl:nil "float32[1] joint_pos~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionData>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionData>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionData
    (cl:cons ':joint_pos (joint_pos msg))
))
