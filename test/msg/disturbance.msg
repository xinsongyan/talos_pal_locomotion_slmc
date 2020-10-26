#rostopic pub /simulator/external_wrench pal_simulation_msgs/ExternalWrench -1 -f disturbance_example.msg
header:
  stamp: now
  frame_id: "base_link"
link_name: "torso_2_link"
wrench:
  force:
    x: 170
    y: 170
    z: 0
  torque:
    x: 0
    y: 0
    z: 0
application_point:
  x: 0.0
  y: 0.0
  z: 0.0
duration:
  secs: 0
  nsecs: 30000000
