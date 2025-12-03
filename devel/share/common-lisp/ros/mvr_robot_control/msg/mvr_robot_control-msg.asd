
(cl:in-package :asdf)

(defsystem "mvr_robot_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActionData" :depends-on ("_package_ActionData"))
    (:file "_package_ActionData" :depends-on ("_package"))
    (:file "MotorData" :depends-on ("_package_MotorData"))
    (:file "_package_MotorData" :depends-on ("_package"))
    (:file "ObserveData" :depends-on ("_package_ObserveData"))
    (:file "_package_ObserveData" :depends-on ("_package"))
  ))