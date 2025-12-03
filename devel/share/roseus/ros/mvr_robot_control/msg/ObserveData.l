;; Auto-generated. Do not edit!


(when (boundp 'mvr_robot_control::ObserveData)
  (if (not (find-package "MVR_ROBOT_CONTROL"))
    (make-package "MVR_ROBOT_CONTROL"))
  (shadow 'ObserveData (find-package "MVR_ROBOT_CONTROL")))
(unless (find-package "MVR_ROBOT_CONTROL::OBSERVEDATA")
  (make-package "MVR_ROBOT_CONTROL::OBSERVEDATA"))

(in-package "ROS")
;;//! \htmlinclude ObserveData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mvr_robot_control::ObserveData
  :super ros::object
  :slots (_header _imu_angular_vel _joint_pos _joint_vel _commands _quat_float ))

(defmethod mvr_robot_control::ObserveData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:imu_angular_vel __imu_angular_vel) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:joint_pos __joint_pos) (make-array 1 :initial-element 0.0 :element-type :float))
    ((:joint_vel __joint_vel) (make-array 1 :initial-element 0.0 :element-type :float))
    ((:commands __commands) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:quat_float __quat_float) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _imu_angular_vel __imu_angular_vel)
   (setq _joint_pos __joint_pos)
   (setq _joint_vel __joint_vel)
   (setq _commands __commands)
   (setq _quat_float __quat_float)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:imu_angular_vel
   (&optional __imu_angular_vel)
   (if __imu_angular_vel (setq _imu_angular_vel __imu_angular_vel)) _imu_angular_vel)
  (:joint_pos
   (&optional __joint_pos)
   (if __joint_pos (setq _joint_pos __joint_pos)) _joint_pos)
  (:joint_vel
   (&optional __joint_vel)
   (if __joint_vel (setq _joint_vel __joint_vel)) _joint_vel)
  (:commands
   (&optional __commands)
   (if __commands (setq _commands __commands)) _commands)
  (:quat_float
   (&optional __quat_float)
   (if __quat_float (setq _quat_float __quat_float)) _quat_float)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32[3] _imu_angular_vel
    (* 4    3)
    ;; float32[1] _joint_pos
    (* 4    1)
    ;; float32[1] _joint_vel
    (* 4    1)
    ;; float32[3] _commands
    (* 4    3)
    ;; float32[4] _quat_float
    (* 4    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32[3] _imu_angular_vel
     (dotimes (i 3)
       (sys::poke (elt _imu_angular_vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[1] _joint_pos
     (dotimes (i 1)
       (sys::poke (elt _joint_pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[1] _joint_vel
     (dotimes (i 1)
       (sys::poke (elt _joint_vel i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _commands
     (dotimes (i 3)
       (sys::poke (elt _commands i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _quat_float
     (dotimes (i 4)
       (sys::poke (elt _quat_float i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32[3] _imu_angular_vel
   (dotimes (i (length _imu_angular_vel))
     (setf (elt _imu_angular_vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[1] _joint_pos
   (dotimes (i (length _joint_pos))
     (setf (elt _joint_pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[1] _joint_vel
   (dotimes (i (length _joint_vel))
     (setf (elt _joint_vel i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _commands
   (dotimes (i (length _commands))
     (setf (elt _commands i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _quat_float
   (dotimes (i (length _quat_float))
     (setf (elt _quat_float i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get mvr_robot_control::ObserveData :md5sum-) "50eb62f4305c2c959b5563826e09f256")
(setf (get mvr_robot_control::ObserveData :datatype-) "mvr_robot_control/ObserveData")
(setf (get mvr_robot_control::ObserveData :definition-)
      "Header header
    float32[3] imu_angular_vel
    float32[1] joint_pos
    float32[1] joint_vel
    float32[3] commands
    float32[4] quat_float
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



(provide :mvr_robot_control/ObserveData "50eb62f4305c2c959b5563826e09f256")


