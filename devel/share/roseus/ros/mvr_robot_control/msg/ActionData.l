;; Auto-generated. Do not edit!


(when (boundp 'mvr_robot_control::ActionData)
  (if (not (find-package "MVR_ROBOT_CONTROL"))
    (make-package "MVR_ROBOT_CONTROL"))
  (shadow 'ActionData (find-package "MVR_ROBOT_CONTROL")))
(unless (find-package "MVR_ROBOT_CONTROL::ACTIONDATA")
  (make-package "MVR_ROBOT_CONTROL::ACTIONDATA"))

(in-package "ROS")
;;//! \htmlinclude ActionData.msg.html


(defclass mvr_robot_control::ActionData
  :super ros::object
  :slots (_joint_pos ))

(defmethod mvr_robot_control::ActionData
  (:init
   (&key
    ((:joint_pos __joint_pos) (make-array 1 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_pos __joint_pos)
   self)
  (:joint_pos
   (&optional __joint_pos)
   (if __joint_pos (setq _joint_pos __joint_pos)) _joint_pos)
  (:serialization-length
   ()
   (+
    ;; float32[1] _joint_pos
    (* 4    1)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[1] _joint_pos
     (dotimes (i 1)
       (sys::poke (elt _joint_pos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[1] _joint_pos
   (dotimes (i (length _joint_pos))
     (setf (elt _joint_pos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get mvr_robot_control::ActionData :md5sum-) "a07255ea0da1785d1c58f5945b919bcb")
(setf (get mvr_robot_control::ActionData :datatype-) "mvr_robot_control/ActionData")
(setf (get mvr_robot_control::ActionData :definition-)
      "float32[1] joint_pos
")



(provide :mvr_robot_control/ActionData "a07255ea0da1785d1c58f5945b919bcb")


