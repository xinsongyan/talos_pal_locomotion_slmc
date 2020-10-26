#rostopic pub /biped_walking_dcm_controller/com_position_goal pal_control_msgs/OperationalSpaceGoal -1 -f file.msg
header:
  stamp: now
  frame_id: "ankle_midpoint"
duration:
  secs: 10
  nsecs: 0
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.71
