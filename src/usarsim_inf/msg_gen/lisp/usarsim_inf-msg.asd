
(cl:in-package :asdf)

(defsystem "usarsim_inf-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ToolType" :depends-on ("_package_ToolType"))
    (:file "_package_ToolType" :depends-on ("_package"))
    (:file "ToolchangerStatus" :depends-on ("_package_ToolchangerStatus"))
    (:file "_package_ToolchangerStatus" :depends-on ("_package"))
    (:file "RangeImageScan" :depends-on ("_package_RangeImageScan"))
    (:file "_package_RangeImageScan" :depends-on ("_package"))
    (:file "EffectorCommand" :depends-on ("_package_EffectorCommand"))
    (:file "_package_EffectorCommand" :depends-on ("_package"))
    (:file "SenseObject" :depends-on ("_package_SenseObject"))
    (:file "_package_SenseObject" :depends-on ("_package"))
    (:file "EffectorStatus" :depends-on ("_package_EffectorStatus"))
    (:file "_package_EffectorStatus" :depends-on ("_package"))
  ))