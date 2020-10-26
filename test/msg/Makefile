LSS = 10
RSS = 11
DS = 21
SS = 22

LINVEL=0.0
ANGVEL=0.0

step:
	rosservice call /biped_walking_dcm_controller/push_velocity_commands \
        "{commands: [{  hasSide: true, \
                        usePose: false, \
                        lin_vel: [${LINVEL}, 0, 0], \
                        ang_vel: [0, 0, ${ANGVEL}], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 10, nsecs: 0}, \
                        supportType: {supportType: ${SS}}, \
                        side: {side: ${LSS}} \
                     }, \
                     {  hasSide: false, \
                        usePose: false, \
                        lin_vel: [0, 0, 0], \
                        ang_vel: [0, 0, 0], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 3, nsecs: 0}, \
                        supportType: {supportType: ${DS}}, \
                        side: {side: 0} \
                     }], \
                        resetList: 0}"

walk:
	rosservice call /biped_walking_dcm_controller/push_velocity_commands \
        "{commands: [{  hasSide: true, \
                        usePose: false, \
                        lin_vel: [${LINVEL}, 0, 0], \
                        ang_vel: [0, 0, ${ANGVEL}], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 10, nsecs: 0}, \
                        supportType: {supportType: ${SS}}, \
                        side: {side: ${RSS}} \
                     }, \
                     {  hasSide: true, \
                        usePose: false, \
                        lin_vel: [${LINVEL}, 0, 0], \
                        ang_vel: [0, 0, ${ANGVEL}], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 10, nsecs: 0}, \
                        supportType: {supportType: ${SS}}, \
                        side: {side: ${LSS}} \
                     }, \
                     {  hasSide: true, \
                        usePose: false, \
                        lin_vel: [${LINVEL}, 0, 0], \
                        ang_vel: [0, 0, ${ANGVEL}], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 10, nsecs: 0}, \
                        supportType: {supportType: ${SS}}, \
                        side: {side: ${RSS}} \
                     }, \
                     {  hasSide: true, \
                        usePose: false, \
                        lin_vel: [${LINVEL}, 0, 0], \
                        ang_vel: [0, 0, ${ANGVEL}], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 10, nsecs: 0}, \
                        supportType: {supportType: ${SS}}, \
                        side: {side: ${LSS}} \
                     }, \
                     {  hasSide: false, \
                        usePose: false, \
                        lin_vel: [0, 0, 0], \
                        ang_vel: [0, 0, 0], \
                        switchTime: {secs: 1, nsecs: 0}, \
                        stanceTime: {secs: 3, nsecs: 0}, \
                        supportType: {supportType: ${DS}}, \
                        side: {side: 0} \
                     }], \
                        resetList: 0}"

dist:
	rostopic pub /simulator/external_wrench pal_simulation_msgs/ExternalWrench -1 -f disturbance.msg
