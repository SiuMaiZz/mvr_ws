;; Auto-generated. Do not edit!


(when (boundp 'mvr_robot_control::MotorData)
  (if (not (find-package "MVR_ROBOT_CONTROL"))
    (make-package "MVR_ROBOT_CONTROL"))
  (shadow 'MotorData (find-package "MVR_ROBOT_CONTROL")))
(unless (find-package "MVR_ROBOT_CONTROL::MOTORDATA")
  (make-package "MVR_ROBOT_CONTROL::MOTORDATA"))

(in-package "ROS")
;;//! \htmlinclude MotorData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mvr_robot_control::MotorData
  :super ros::object
  :slots (_header _id _kp_ _kd_ _ff_ _pos_des_ _vel_des_ ))

(defmethod mvr_robot_control::MotorData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:kp_ __kp_) 0.0)
    ((:kd_ __kd_) 0.0)
    ((:ff_ __ff_) 0.0)
    ((:pos_des_ __pos_des_) 0.0)
    ((:vel_des_ __vel_des_) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _kp_ (float __kp_))
   (setq _kd_ (float __kd_))
   (setq _ff_ (float __ff_))
   (setq _pos_des_ (float __pos_des_))
   (setq _vel_des_ (float __vel_des_))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:kp_
   (&optional __kp_)
   (if __kp_ (setq _kp_ __kp_)) _kp_)
  (:kd_
   (&optional __kd_)
   (if __kd_ (setq _kd_ __kd_)) _kd_)
  (:ff_
   (&optional __ff_)
   (if __ff_ (setq _ff_ __ff_)) _ff_)
  (:pos_des_
   (&optional __pos_des_)
   (if __pos_des_ (setq _pos_des_ __pos_des_)) _pos_des_)
  (:vel_des_
   (&optional __vel_des_)
   (if __vel_des_ (setq _vel_des_ __vel_des_)) _vel_des_)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _id
    2
    ;; float32 _kp_
    4
    ;; float32 _kd_
    4
    ;; float32 _ff_
    4
    ;; float32 _pos_des_
    4
    ;; float32 _vel_des_
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _id
       (write-word _id s)
     ;; float32 _kp_
       (sys::poke _kp_ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _kd_
       (sys::poke _kd_ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ff_
       (sys::poke _ff_ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pos_des_
       (sys::poke _pos_des_ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _vel_des_
       (sys::poke _vel_des_ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _kp_
     (setq _kp_ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _kd_
     (setq _kd_ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ff_
     (setq _ff_ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pos_des_
     (setq _pos_des_ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _vel_des_
     (setq _vel_des_ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mvr_robot_control::MotorData :md5sum-) "6bed2913051bec1ee3cdf057ad08a302")
(setf (get mvr_robot_control::MotorData :datatype-) "mvr_robot_control/MotorData")
(setf (get mvr_robot_control::MotorData :definition-)
      "Header header
    int16 id
    float32 kp_
    float32 kd_
    float32 ff_
    float32 pos_des_ 
    float32 vel_des_
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :mvr_robot_control/MotorData "6bed2913051bec1ee3cdf057ad08a302")


