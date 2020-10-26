^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package talos_pal_locomotion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.37 (2018-09-13)
-------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  fixing tests
  See merge request control/talos_pal_locomotion!40
* fixed tests for lower body
* fixing tests
* Contributors: Hilario Tome

0.0.36 (2018-09-12)
-------------------
* Merge branch 'gains_access' into 'erbium-devel'
  made gains accesible
  See merge request control/talos_pal_locomotion!38
* added time out in test
* more tunning
* Merge branch 'as_threaded_estimator' of gitlab:control/talos_pal_locomotion into gains_access
* tried changing solver type
* tunning parameters
* gains making the robot walk
* more gains
* separated roll pitch yaw gains
* made gains accesible
* configs: add spawn_estimator_thread -> false
* Contributors: Hilario Tome, alexandersherikov

0.0.35 (2018-09-10)
-------------------
* Merge branch 'as_static_walk' into 'erbium-devel'
  Add static walk test.
  See merge request control/talos_pal_locomotion!37
* Static walk: step bounds
* Replace deprecated ariless calls.
* Fixed static_walk tests in gazebo, enable extra tests
  Also, use kinematic_estimator_params by default when DISABLE_EXTENDED_TESTS=YES
* static walk test: use angular velocity
* Add static_walk.launch + script to send commands.
* test/CMakeLists: Readd accidentally deleted static_walk_test
* Add static walk test.
* Contributors: alexandersherikov

0.0.34 (2018-09-05)
-------------------
* Merge branch 'erbium-devel' of gitlab:control/talos_pal_locomotion into erbium-devel
* Merge branch 'cop_friction_cone_constraint' into 'erbium-devel'
  Cop friction cone constraint
  See merge request control/talos_pal_locomotion!36
* fixed gains for full
* more tunning paramters
* added more logging of individual cop
* Merge branch 'erbium-devel' of gitlab:control/talos_pal_locomotion into erbium-devel
* Contributors: Hilario Tome

0.0.33 (2018-08-29)
-------------------
* Merge branch 'as_balancing_orientation_fix' into 'erbium-devel'
  balancing: change support foot
  See merge request control/talos_pal_locomotion!33
* balancing: add safety params
* Update swing foot position in balancing configs
* balancing: change support foot
* Contributors: alexandersherikov

0.2.15 (2020-08-01)
-------------------
* Merge branch 'fix_safety_override_torso' into 'erbium-devel'
  remove the brackets for proper safety override functionality for torso joints
  See merge request control/talos_pal_locomotion!102
* remove the brackets for proper safety override functionality for torso joints
* Contributors: saikishor, victor

0.2.14 (2020-07-27)
-------------------
* Merge branch 'kinematic_chain' into 'erbium-devel'
  Create empty robot_model_chain
  See merge request control/talos_pal_locomotion!101
* Merge branch 'jenkins_tests' into 'erbium-devel'
  Fix static walk test
  See merge request control/talos_pal_locomotion!100
* Create empty robot_model_chain
* Fix standing test
* Fix static walk test
* Update README
* Contributors: Adria Roig, victor

0.2.13 (2020-07-16)
-------------------
* Merge branch 'lateral_offset' into 'erbium-devel'
  Reduce slightly CoM lateral offset
  See merge request control/talos_pal_locomotion!99
* Reduce slightly CoM lateral offset
* Contributors: Adria Roig, victor

0.2.12 (2020-07-15)
-------------------
* Merge branch 'extra_biped_param' into 'erbium-devel'
  Add extra biped parameters files for debrise + balancing
  See merge request control/talos_pal_locomotion!98
* renamed file debrise to debris
* Add extra biped parameters files for debrise + balancing
* Merge branch 'documentation_review' into 'erbium-devel'
  Review done
  See merge request control/talos_pal_locomotion!97
* Review done
* Contributors: Adria Roig, Sai Kishor Kothakota, narcismiguel, victor

0.2.11 (2020-07-10)
-------------------
* Merge branch 'robot_experiments' into 'erbium-devel'
  Robot experiments
  See merge request control/talos_pal_locomotion!96
* Merge branch 'robot_experiments' of gitlab:control/talos_pal_locomotion into robot_experiments
* Add hybrid factor for static walk action
* Extend README
* Modify biped parameters
* Merge branch 'robot_experiments' of gitlab:control/talos_pal_locomotion into robot_experiments
* Extend README
* Modify biped parameters
* Modify biped parameters
* Contributors: Adria Roig, victor

0.2.10 (2020-07-08)
-------------------
* Merge branch 'talos_6_fixes' into 'erbium-devel'
  Talos 6 fixes
  See merge request control/talos_pal_locomotion!94
* Apply suggestion to README.md
* added z_height to swing interpolator test as the common parameters effect the test
* Load IMU specific parameters and remove the kvh_imu argument from launch
* Remove integral correction
* Add static walk fixes parameters
* Add KVH imu as argument
* added README
* fixed the safety_override of torso transmission on full_v2
* added a way to disable safety for only a set of joints
* Contributors: Adria Roig, Sai Kishor Kothakota, saikishor, victor

0.2.9 (2020-06-05)
------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  fix the contact monitor test
  See merge request control/talos_pal_locomotion!93
* fix the contact monitor test
* Merge branch 'swing_interp' into 'erbium-devel'
  Add simulation biped parameters
  See merge request control/talos_pal_locomotion!91
* Remove by default gui and log_introspection_args
* Add fix for last DS step
* Add DS step at the end
* Print messages
* Fix mmissing args swing interp test
* Add ARG gui equal to true for debugging
* Add gazebo logs to swing interpolator test
* Modifed params test
* Increase swing leg tolerances
* Increase swing interp tolerance
* Fix swing interp test
* Add sleeps in swing interpolator test
* Increse tolerance to fix test on jenkins
* Fix swing interpolator test
* Force GUI equal to true for swing traj test
* Increase timeout swing interpolator test
* Add swing interpolator test
* Add simulation biped parameters
* Contributors: Adria Roig, Adrià Roig, Sai Kishor Kothakota, victor

0.2.8 (2020-05-05)
------------------
* Merge branch 'wait-for-walk-action' into 'erbium-devel'
  Wait until walk action has been completely pushed
  See merge request control/talos_pal_locomotion!88
* Wait until walk action has been completely pushed
* Contributors: Victor Lopez, victor

0.2.7 (2020-03-20)
------------------
* Merge branch 'static_stepping' into 'erbium-devel'
  Fix static stepping tests
  See merge request control/talos_pal_locomotion!90
* Increase static_stepping duration for gazebo contact issue
* Fix static stepping tests
* Contributors: Adrià Roig, victor

0.2.6 (2020-03-19)
------------------
* Merge branch 'static_test' into 'erbium-devel'
  Modified biped_parameters
  See merge request control/talos_pal_locomotion!89
* Modified biped_parameters
* Contributors: Adrià Roig, victor

0.2.5 (2020-03-16)
------------------
* Merge branch 'hiqh_freq' into 'erbium-devel'
  added 2kHz run frequency to pal physics simulator launches
  See merge request control/talos_pal_locomotion!84
* added 2kHz run frequency to pal physics simulator launches
* Fix changed orientation commited by mistake
* Contributors: Sai Kishor Kothakota, Victor Lopez, victor

0.2.4 (2020-03-11)
------------------
* Fix launch order in contact monitor test
* Contributors: Victor Lopez

0.2.3 (2020-03-11)
------------------
* Merge branch 'fix-dcm-monitor-test' into 'erbium-devel'
  Add sleep before pushing the robot to give it some time to stabilize
  See merge request control/talos_pal_locomotion!87
* Add sleep before pushing the robot to give it some time to stabilize
* Contributors: Victor Lopez, victor

0.2.2 (2020-03-08)
------------------
* Merge branch 'fix-test-controller-wait' into 'erbium-devel'
  Fix wait for controller on tests
  See merge request control/talos_pal_locomotion!86
* Save RAIIPopen until the end of test, or the controllers are removed
* Fix wait for controller on tests
* Contributors: Victor Lopez, victor

0.2.1 (2020-03-06)
------------------
* Reduce timeout of 10k seconds to 1k
* Contributors: Victor Lopez

0.2.0 (2020-03-03)
------------------
* Merge branch 'static_walking' into 'erbium-devel'
  Static walking
  See merge request control/talos_pal_locomotion!79
* Fixed tests
* Fix tests
* Improve gui param
* Fix roslaunch exception tests
* Modify biped parameters after some experiments
* fix missing parameters
* Remove controller_parameters files
* Increase z_height
* high frequency
* Contributors: Adria Roig, Hilario Tome, Victor Lopez, victor

0.1.28 (2020-02-27)
-------------------
* Merge branch 'high_freq' into 'erbium-devel'
  high frequency
  See merge request control/talos_pal_locomotion!81
* Increase frequency to 2khz and add estimator thread
* Contributors: Victor Lopez, victor

0.1.27 (2020-01-17)
-------------------
* Merge branch 'dcm_monitor' into 'erbium-devel'
  Add test for DCM monitor
  See merge request control/talos_pal_locomotion!78
* Add safety by default
* Add dcm_monitor to dynamic balancing
* Remove duplicated function
* Fix tests
* Fix contact monitor test
* Remove safety for balancing dynamic support
* Reduce contact threeshold
* Add contact monitor test
* Check for foot contact
* Fix dcm monitor test
* Remove DCM monitor for dynamic walking
* Add test for DCM monitor
* Contributors: Adria Roig, Adrià Roig, Victor Lopez

0.1.26 (2019-11-19)
-------------------
* fixed typo gain
* Contributors: Hilario Tome

0.1.25 (2019-11-19)
-------------------
* Merge branch 'relative_gains' into 'erbium-devel'
  removed relative gains
  See merge request control/talos_pal_locomotion!76
* removed relative gains
* Contributors: Hilario Tome

0.1.24 (2019-11-19)
-------------------
* Merge branch 'test_tolerances' into 'erbium-devel'
  increased test tolerances
  See merge request control/talos_pal_locomotion!75
* increased test tolerances
* Contributors: Hilario Tome

0.1.23 (2019-10-15)
-------------------
* Fix shadowed variable
* Contributors: Victor Lopez

0.1.22 (2019-08-01)
-------------------
* dissabled tests in gui
* Contributors: Hilario Tome

0.1.21 (2019-08-01)
-------------------
* Merge branch 'fix_test' into 'erbium-devel'
  more testing
  See merge request control/talos_pal_locomotion!73
* increased tolerances
* increased test tolerances
* 0.1.20
* updated changelog
* Merge branch 'fix_yaw' into 'erbium-devel'
  enabled fixing yaw
  See merge request control/talos_pal_locomotion!74
* more testing
* enabled fixing yaw
* Contributors: Hilario Tome

0.1.19 (2019-07-30)
-------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  deleted old tests
  See merge request control/talos_pal_locomotion!72
* deleted old tests
* Contributors: Hilario Tome

0.1.20 (2019-08-01)
-------------------
* Merge branch 'fix_yaw' into 'erbium-devel'
  enabled fixing yaw
  See merge request control/talos_pal_locomotion!74
* 0.1.19
* updated changelog
* Merge branch 'fix_tests' into 'erbium-devel'
  deleted old tests
  See merge request control/talos_pal_locomotion!72
* deleted old tests
* enabled fixing yaw
* Contributors: Hilario Tome

0.1.18 (2019-07-29)
-------------------
* Merge branch 'memmo' into 'erbium-devel'
  Memmo
  See merge request control/talos_pal_locomotion!71
* unfreezed kinematic estimator params
* api change interpoaltor
* Contributors: Hilario Tome

0.1.17 (2019-07-24)
-------------------
* Merge branch 'inertia_id' into 'erbium-devel'
  Inertia
  See merge request control/talos_pal_locomotion!69
* changed default height lower body torso robot
* changed from ankle to sole in estimator
* modified some commom parameters
* Contributors: Hilario Tome

0.1.16 (2019-07-21)
-------------------
* Merge branch 'extended_actuator_test' into 'erbium-devel'
  added actuator tests when dissabling extended tests
  See merge request control/talos_pal_locomotion!68
* added actuator tests when dissabling extended tests
* Contributors: Hilario Tome

0.1.15 (2019-07-17)
-------------------
* Merge branch 'safe_initialization' into 'erbium-devel'
  added safe initialization
  See merge request control/talos_pal_locomotion!66
* added safe controller initialization executable
* added inertia shaping as depend
* added safe initialization with position launch
* added while spin
* fixing safe intializer
* added safe initialization
* Contributors: Hilario Tome

0.1.14 (2019-07-16)
-------------------
* Merge branch 'stance_actuators_test' into 'erbium-devel'
  Stance actuators test
  See merge request control/talos_pal_locomotion!67
* changed definitiion of safety override
* fixed bug in wrist control with actuators sim
* updated test
* standing test launches
* removed gui from tests
* initial test with actuators working
* added stance actuators test
* Contributors: Hilario Tome

0.1.13 (2019-05-30)
-------------------
* Merge branch 'head_effort' into 'erbium-devel'
  changed head to effort control in real robot
  See merge request control/talos_pal_locomotion!64
* changed head to effort control in real robot
* Contributors: Hilario Tome

0.1.12 (2019-05-20)
-------------------
* Merge branch 'moved_actions' into 'erbium-devel'
  moved actions into a separated package
  See merge request control/talos_pal_locomotion!63
* moved actions into a separated package
* Contributors: Hilario Tome

0.1.11 (2019-05-15)
-------------------
* Merge branch 'license-refactor' into 'erbium-devel'
  Update pal license
  See merge request control/talos_pal_locomotion!62
* Update PAL licenses
* Contributors: Victor Lopez

0.1.10 (2019-04-23)
-------------------
* Merge branch 'new-statistics-msgs' into 'erbium-devel'
  New topic name
  See merge request control/talos_pal_locomotion!61
* New topic name
* Contributors: Victor Lopez

0.1.9 (2019-03-24)
------------------
* Merge branch 'wrist_test' into 'erbium-devel'
  fixed wrist test
  See merge request control/talos_pal_locomotion!60
* fixed wrist test
* Contributors: Hilario Tome

0.1.8 (2019-03-22)
------------------
* Merge branch 'wrist_local_control' into 'erbium-devel'
  added wrist local joint control type parameter
  See merge request control/talos_pal_locomotion!59
* moved com link offseter params
* added wrist local joint control type parameter
* Contributors: Hilario Tome

0.1.7 (2019-03-19)
------------------
* Merge branch 'fixed_upper_body' into 'erbium-devel'
  fixed upper body fixed
  See merge request control/talos_pal_locomotion!57
* reduced icp gain
* added link com offset
* added link com offset helper
* Contributors: Hilario Tome

0.1.6 (2019-01-29)
------------------
* Merge branch 'as_fix_launch_files' into 'erbium-devel'
  Minor fixes in launch files.
  See merge request control/talos_pal_locomotion!56
* Do not add tasks with zero gains, cleanups.
* Fix various issues with WBC parameters.
* Minor fixes & updates in launch files.
* modified for real robot
* fixed upper body fixed
* Contributors: Hilario Tome, alexandersherikov

0.1.5 (2018-12-12)
------------------
* Merge branch 'limits' into 'erbium-devel'
  reenabled joint and torque limits
  See merge request control/talos_pal_locomotion!55
* formating
* added end effector stabilization
* first iteracion of wbc action working in talos
* reenabled joint and torque limits
* Contributors: Hilario Tome

0.1.4 (2018-11-21)
------------------
* Merge branch 'as_vel_limits' into 'erbium-devel'
  Disable joint vel bounds for lower_body_torso_head and fixed_upper_body
  See merge request control/talos_pal_locomotion!53
* Disable joint vel bounds for lower_body_torso_head and fixed_upper_body
* Contributors: alexandersherikov

0.1.3 (2018-11-16)
------------------
* Merge branch 'as_new_state_all_in_base' into 'erbium-devel'
  As new state all in base
  See merge request control/talos_pal_locomotion!52
* Migrate to new rbcomposite::State.
* Contributors: alexandersherikov

0.1.2 (2018-11-12)
------------------
* Merge branch 'as_update' into 'erbium-devel'
  Ariles compatibility fix.
  See merge request control/talos_pal_locomotion!50
* Ariles compatibility fix.
* Contributors: alexandersherikov

0.1.1 (2018-11-08)
------------------
* Merge branch 'as_cfg_fix' into 'erbium-devel'
  Fix open_loop_com_tracking issue.
  See merge request control/talos_pal_locomotion!51
* Fix open_loop_com_tracking issue.
* Contributors: Hilario Tome, alexandersherikov

0.1.0 (2018-11-05)
------------------
* Merge branch 'as_vel_supp' into 'erbium-devel'
  Reworked velocity suppression task + fixed configs
  See merge request control/talos_pal_locomotion!39
* tests: Tolerances & timeouts.
* tests: disable legged_switching_state_link_position_estimator for stepping
* Added static stepping test.
* Enable stepping test.
* adjust test tolerances; readd config which is used by other pkgs
* Minor fixes in test launch files.
* test cleanup, stepping.test (not working yet)
* Fix bug in dynamic_moving_support test.
* test static_walk: increase waiting time
* Do not set initial_state_machine_action in config files.
* Sync WBC parameters.
* Deduplicate configuration parameters.
* Comment out unused tasks.
* Minimize diffs from master.
* Constraint fixes & tuning
* walking_wbc_base: do not initialize stance foot gains
* Fewer balancing tests with DISABLE_EXTENDED_TESTS.
* Initialize stance foot gains.
* Adjust torsion friction, fix bug in stance foot orientaion gain
* Reworked velocity suppression task + fixed configs
* Contributors: alexandersherikov

0.0.47 (2018-10-25)
-------------------
* Merge branch 'migrate-to-statistics' into 'erbium-devel'
  Migrate to statistics
  See merge request control/talos_pal_locomotion!49
* Migrate to statistics
* Contributors: Victor Lopez

0.0.46 (2018-10-17)
-------------------
* changed referece posture gain
* Merge branch 'head' into 'erbium-devel'
  adding back the head
  See merge request control/talos_pal_locomotion!48
* open loop parameters
* adding back the head
* Contributors: Hilario Tome

0.0.45 (2018-10-10)
-------------------
* Merge branch 'output_diff_type' into 'erbium-devel'
  Output diff type
  See merge request control/talos_pal_locomotion!47
* fixed moving support launch file
* test fixes
* tests: fix configs for balancing and static walking
* Fix broken tests.
* Contributors: Hilario Tome, alexandersherikov

0.0.44 (2018-09-28)
-------------------
* fixed tests
* Contributors: Hilario Tome

0.0.43 (2018-09-28)
-------------------
* changed waiting time tests
* Contributors: Hilario Tome

0.0.42 (2018-09-28)
-------------------
* Merge branch 'more_tests' into 'erbium-devel'
  fixing more tests
  See merge request control/talos_pal_locomotion!45
* fixing more tests
* Contributors: Hilario Tome

0.0.41 (2018-09-28)
-------------------
* Merge branch 'fix_tests' into 'erbium-devel'
  fixing more tests
  See merge request control/talos_pal_locomotion!44
* fixing more tests
* Contributors: Hilario Tome

0.0.40 (2018-09-27)
-------------------
* Merge branch 'torso_torque' into 'erbium-devel'
  balancing with upper body joints, no toros
  See merge request control/talos_pal_locomotion!43
* more test fixing
* fixing tests
* changed bias estimator
* fixed repeated name tasks
* balancing with upper body joints, no toros
* Contributors: Hilario Tome

0.0.39 (2018-09-25)
-------------------
* Merge branch 'remove_dt' into 'erbium-devel'
  Remove dt
  See merge request control/talos_pal_locomotion!41
* added bias estimation
* fixed PhysicsConstraintsParameters enum
* more tunning
* tunning
* added back default configuration
* working in simulation thread controller
* unified parameters in single struct
* added cop tracking
* added logging of local quantities, removed dt from parameters, added fixed upper body
* Contributors: Hilario Tome

0.0.38 (2018-09-14)
-------------------
* removed margin cop from full model
* 0.0.37
* updated changelog
* Merge branch 'fix_tests' into 'erbium-devel'
  fixing tests
  See merge request control/talos_pal_locomotion!40
* fixed tests for lower body
* fixing tests
* 0.0.36
* updated changelog
* Merge branch 'gains_access' into 'erbium-devel'
  made gains accesible
  See merge request control/talos_pal_locomotion!38
* added time out in test
* more tunning
* Merge branch 'as_threaded_estimator' of gitlab:control/talos_pal_locomotion into gains_access
* tried changing solver type
* tunning parameters
* gains making the robot walk
* more gains
* separated roll pitch yaw gains
* made gains accesible
* 0.0.35
* Updated changelog
* Merge branch 'as_static_walk' into 'erbium-devel'
  Add static walk test.
  See merge request control/talos_pal_locomotion!37
* Static walk: step bounds
* Replace deprecated ariless calls.
* Fixed static_walk tests in gazebo, enable extra tests
  Also, use kinematic_estimator_params by default when DISABLE_EXTENDED_TESTS=YES
* static walk test: use angular velocity
* Add static_walk.launch + script to send commands.
* test/CMakeLists: Readd accidentally deleted static_walk_test
* Add static walk test.
* 0.0.34
* updated changelog
* Merge branch 'erbium-devel' of gitlab:control/talos_pal_locomotion into erbium-devel
* Merge branch 'cop_friction_cone_constraint' into 'erbium-devel'
  Cop friction cone constraint
  See merge request control/talos_pal_locomotion!36
* fixed gains for full
* more tunning paramters
* added more logging of individual cop
* configs: add spawn_estimator_thread -> false
* 0.0.33
* Updated changelog
* Merge branch 'as_balancing_orientation_fix' into 'erbium-devel'
  balancing: change support foot
  See merge request control/talos_pal_locomotion!33
* Merge branch 'erbium-devel' of gitlab:control/talos_pal_locomotion into erbium-devel
* balancing: add safety params
* Update swing foot position in balancing configs
* balancing: change support foot
* Contributors: Hilario Tome, alexandersherikov

0.0.32 (2018-08-29)
-------------------
* Merge branch 'fixing_tests' into 'erbium-devel'
  fixing tolerances for tests
  See merge request control/talos_pal_locomotion!32
* fixing tolerances for tests
* Contributors: Hilario Tome

0.0.31 (2018-08-28)
-------------------
* Merge branch 'reduced_friction_cone' into 'erbium-devel'
  Reduced friction cone
  See merge request control/talos_pal_locomotion!31
* gains ported from the robot
* added kinematic state estimator depend
* removed deprecated header
* reenabled ds force distribution
* fixed merge
* added kinematic estimator test
* Fix force distribution & foot width.
* correct processing of solution from reduced dynamics friction cone
* Some helpers for balancing tests on the robot.
* Balancing on one foot: use ankle_left as reference.
* Blancing on two feet: use ankle_midpoint as reference.
* Contributors: Hilario Tome, alexandersherikov

0.0.30 (2018-08-24)
-------------------
* Merge branch 'pid_rate' into 'erbium-devel'
  Pid rate
  See merge request control/talos_pal_locomotion!29
* fixing tests
* Updated configs
* removed hardcoded cop
* added support for dynamics reduced friction cone formulation
* cleanup gains
* added extra parameters for estimator
* Contributors: Hilario Tome, alexandersherikov

0.0.29 (2018-08-13)
-------------------
* Merge branch 'as_gazebo_test' into 'erbium-devel'
  Reenable one Gazebo test when DISABLE_EXTENDED_TESTS=true
  See merge request control/talos_pal_locomotion!24
* Cleanup after rebase.
* Reenable one Gazebo test when DISABLE_EXTENDED_TESTS=true
  Otherwise we might accidentally break Gazebo support by modifying
  dependency.
* Contributors: alexandersherikov

0.0.28 (2018-08-01)
-------------------
* Merge branch 'parameters_rebased' into 'erbium-devel'
  Parameters rebased
  See merge request control/talos_pal_locomotion!23
* modified final tolerance
* added missing torque scaling gain
* migrated balancing base tests to new config struct
* removed gui from test
* fixed balancing parameters
* fixed balancing parameters
* increased filter cuttoffs to make simulator tests pass
* fixing tests
* allocated physics tasks
* fixed rebase
* modified some parameters
* modified to use ariles parameters
* separated parameters
* Solver type is now configurable.
* lower_body_torso_head: no control
* Contributors: Hilario Tome, alexandersherikov

0.0.27 (2018-08-01)
-------------------
* Merge branch 'wbc_grasp_demo' into 'erbium-devel'
  Fixed griper trajectory controllers
  See merge request control/talos_pal_locomotion!22
* Fixed griper trajectory controllers
* Contributors: Adrià Roig, Hilario Tome

0.0.26 (2018-07-25)
-------------------
* Merge branch 'as_init_lib' into 'erbium-devel'
  tests: Use pal_configuration_initializer
  See merge request control/talos_pal_locomotion!21
* Disable collision checks in configuration_initializer
  Initial configuration is colliding in Gazebo.
* bringup controllers in pal_physics_simulator.launch
* tests: Use pal_configuration_initializer
* Contributors: alexandersherikov

0.0.25 (2018-07-24)
-------------------
* Merge branch 'as_test_tuning' into 'erbium-devel'
  Tune test parameters
  See merge request control/talos_pal_locomotion!19
* Tune test parameters
* Contributors: alexandersherikov

0.0.24 (2018-07-23)
-------------------
* Merge branch 'moving_support' into 'erbium-devel'
  added gazebo tests
  See merge request control/talos_pal_locomotion!15
* Switch to qpmad solver by default.
* Cleanup after rebase
* Solver type is now configurable.
* lower_body_torso_head: no control
* Contributors: alexandersherikov

0.0.23 (2018-07-20)
-------------------
* Merge branch 'blending_rebased' into 'erbium-devel'
  Blending rebased
  See merge request control/talos_pal_locomotion!18
* started tasks in lower body stack
* added start task
* fixed merge
* migrating api
* A few fixes related to control of grippers.
* Test tuning
* Make use of DISABLE_EXTENDED_TESTS
* Headless simulator, test tuning.
* Cleanup after rebase, refactoring, common style.
* changed default robot in dcm launch
* especified thin box in moving support test
* added moving support test for lower_body torso head
* Resolve issue with multiple model loading.
* faster movements in balancing test
* added balancing tests
* added head
* added gazebo tests
* Contributors: Hilario Tome, alexandersherikov

0.0.22 (2018-07-13)
-------------------
* Merge branch 'as_headless_simulation' into 'erbium-devel'
  headless -> true for pal_physics_simulator
  See merge request control/talos_pal_locomotion!16
* headless -> true for pal_physics_simulator
* Contributors: alexandersherikov

0.0.21 (2018-07-13)
-------------------
* Merge branch 'as_commander_api' into 'erbium-devel'
  trajectory_dispatcher::Commander -- new API
  See merge request control/talos_pal_locomotion!17
* trajectory_dispatcher::Commander -- new API
* Contributors: alexandersherikov

0.0.20 (2018-07-11)
-------------------
* Merge branch 'as_balancing_tests' into 'erbium-devel'
  Working on balancing tests.
  See merge request control/talos_pal_locomotion!13
* Re-add dummy test launchers to prevent false positives
* Balancing on one leg
* More balancing tests + refactoring
* Add test for standing balancing
* Contributors: alexandersherikov

0.0.19 (2018-07-10)
-------------------
* Merge branch 'as_plus_head' into 'erbium-devel'
  Add head to arm-less configuration
  See merge request control/talos_pal_locomotion!14
* Add head to arm-less configuration
* Contributors: alexandersherikov

0.0.18 (2018-07-10)
-------------------
* Merge branch 'new_estimator' into 'erbium-devel'
  New estimator
  See merge request control/talos_pal_locomotion!12
* dynamic_walk_simple: use position control to reach initial conf
* Contributors: Hilario Tome, alexandersherikov

0.0.17 (2018-07-09)
-------------------
* Merge branch 'new_estimator' into 'erbium-devel'
  added extra parameters for estimator
  See merge request control/talos_pal_locomotion!10
* dynamic_walk_simple.test: disable recording of rosbag
* dynamic_walk_simple: start position controllers first.
* dynamic_walk_simple: record introspection data
* Prevent overriding of test results in dynamic_walk_simple.test
* Rebased and cleaned up.
* dissabled rviz in tests
* fixed estimator
* added extra parameters for estimator
* Contributors: Hilario Tome, alexandersherikov

0.0.16 (2018-07-04)
-------------------
* Merge branch 'as_selective_loading' into 'erbium-devel'
  Launch script rename in talos_controller_configuration
  See merge request control/talos_pal_locomotion!11
* Added missing dependency on pal_test_utils.
* Add controller & tests for partial talos, refactoring.
* delete walking_controller_spawner
* Launch script rename in talos_controller_configuration
* Contributors: alexandersherikov

0.0.15 (2018-07-02)
-------------------
* Merge branch 'as_talos_locomotion_fix' into 'erbium-devel'
  change test parameters and contact constraints
  See merge request control/talos_pal_locomotion!9
* locomotion test: tuning, disable grippers.
* Gripper reference vpositions and cleanups
* change test parameters and contact constraints
* Contributors: Hilario Tome, alexandersherikov

0.0.14 (2018-06-21)
-------------------
* Fix wrong launch path
* Contributors: Victor Lopez

0.0.13 (2018-06-21)
-------------------
* Merge branch 'launch-utilities' into 'erbium-devel'
  Launch utilities
  See merge request control/talos_pal_locomotion!8
* Start walking when launching walking controller
* Add talos steering
* Merge branch 'tunning_gazebo' into 'erbium-devel'
  Tunning gazebo
  See merge request control/talos_pal_locomotion!7
* changed default step time
* Contributors: Hilario Tome, Victor Lopez

0.0.12 (2018-06-21)
-------------------
* Merge branch 'tunning_gazebo' into 'erbium-devel'
  tunning gazebo
  See merge request control/talos_pal_locomotion!6
* tunning gazebo
* Contributors: Hilario Tome

0.0.11 (2018-06-19)
-------------------
* Merge branch 'tunning_hardware' into 'erbium-devel'
  fixed compatibility with new local joint control
  See merge request control/talos_pal_locomotion!5
* added verification to walking test
* fixed local joint control
* fixed compatibility with new local joint control
* Contributors: Hilario Tome

0.0.10 (2018-05-09)
-------------------
* increased test time
* Contributors: Hilario Tome

0.0.9 (2018-04-19)
------------------
* Merge branch 'test2' into 'erbium-devel'
  reenabled tests
  See merge request control/talos_pal_locomotion!4
* reenabled tests
* Contributors: Hilario Tome

0.0.8 (2018-04-16)
------------------
* removed changelog
* updated changelog
* updated changelog
* comenting out test until jenkins cpu speed is fixed
* 0.0.7
* updated changelog
* Merge branch 'fix_test' into 'erbium-devel'
  extended time test
  See merge request control/talos_pal_locomotion!2
* extended time test
* Contributors: Hilario Tome

0.0.6 (2018-04-13 16:17)
------------------------
* updated changelog
* Update dynamic_walking_speed_simulation_test.test
* Contributors: Hilario Tome

0.0.5 (2018-04-13 10:51)
------------------------
* updated changelog
* removed changelog
* Merge branch 'release' into 'erbium-devel'
  added missing talos_wbc depend
  See merge request control/talos_pal_locomotion!1
* commented out estimator test
* commented out rqt steering
* added missing talos_wbc depend
* added missing xacro test
* changed to imu link for walking
* reenabled test
* walking test working expect segfault in destructor
* changed to more reasonable parameters for legged state estimator
* first try with full state estimator
* reenabled force distribution:
* changes for ariles and absolute time
* migration
* fixed compilation
* pal_robot_tools migration
* moved test utils to experimental walking
* working, (stops randomly), moving support dynamic test
* splitting pal robot tools
* migration from pal robot tools
* separated talos physics simulator
* separated talos physics simulator
* rough terrain working with new bullet version
* Merge branch 'erbium-devel' of gitlab:control/talos_experimental_walking into erbium-devel
* removed hardcoded z 0, and changed leg penetration ground
* Update package.xml
* Update CMakeLists.txt
* changed to package 2
* renamed and deleted files
* cleaned package
* cleanup
* fixed dynamic step options
* moved to kinematic simulator
* fixed step adjustment tests
* fixed compatibility with new reference
* Merge branch 'erbium-devel' of gitlab:control/talos_experimental_walking into erbium-devel
* added support for linear cop inside foot
* fixed merge conflict
* fixed merge
* added pal collision namespace
* fixed tests
* step adjustment test working again
* Merge branch 'increase-gains' into 'erbium-devel'
  Increase gains so robot is able to lift srcsim panel
  See merge request !2
* Increase gains so robot is able to lift srcsim panel
* added separate dynamic reconfigure for grippers
* Merge branch 'erbium-devel' of gitlab:control/talos_experimental_walking into erbium-devel
* fixed compilation tests
* changed upper body reference to ddynamic reconfigure, useless because it getts overridden
* changed default walking params
* dynamic step adjustment tests working again, added hierarquical wbc (not working)
* added biped inverse dynamics monitor
* Merge branch 'erbium-devel' of gitlab:control/talos_experimental_walking into erbium-devel
* added initial floating base to tests
* replannin working, had to comend out ds force distribution there is a bug
* fixed pal robot tools visualization namespace
* fixed compilation
* migrated to new wbc
* added stand alone pal physics simulator launch file
* added timer introspection
* added extra parameters that where hardcoded
* fixed api
* Merge branch 'erbium-devel' of gitlab:control/talos_experimental_walking into erbium-devel
* chaged default swing height
* full controlled joints configured
* Merge branch 'no-gui-launch' into 'erbium-devel'
  Allow launch without gui
  See merge request !1
* cleanup and changed to vector reference
* fixed compilation error foot step replanning test
* trying to implement foot step adjustment with cubic polynomial
* added rought terrain simulation
* realistic terrain generated
* rough terrain generation
* changeg bag names
* added talos speed walking test
* initial refactor of refactor using pluginlib
* migrated to push by id
* Allow launch without gui
* fixed api
* reduced the time in dynamic foot step adjustment test from 0.001 to 0.005
* added future step pose publishing and sm status
* working with parametrized foot step adjustment
* lowered a bit gains wbc to used upper body to balance
* working step adjustment test
* added foot step adjustment kinematic test
* added kinematic test
* added kinemtic step adjustment test
* changed default talos walking height
* clean up, and added basic walking test
* Changes in src/footstep_replanning_test.cpp
* Stop replanning at the end of the test
* Added configuration of rviz and modified test to add noise at each step
* working on footstep replanning
* added physics simulation stand alone and angular momentum test double support and single support
* added standalone simulator executable with walking
* Merge branch 'dubnium-devel' of gitlab:control/talos_experimental_walking into dubnium-devel
* removed missing header
* fixed api
* Clean up launch files and added pal physics simulator
* 0.0.4
* Updated changelog
* Roll back to simulator estimator until support for out of base_link imu working
* Changed estimator to simple legged ekf
* Fixed gripper name and removed joint torque sensor state controller
* 0.0.3
* Updated changelog
* Added missing install rules
* 0.0.2
* Updated changelog
* Added missing install rule
* 0.0.1
* Updated changelo
* Renamed tor to talos
* Fixed launch file
* Added namespace
* Fixed merge
* Added namespace
* Merge branch 'master' of gitlab:control/tor_development
* Added paremeters to choose between effort and position joints in generic base controller
* Changed tor default height
* Added momentum reference
* Added pal_wbc namespace
* Finished adding torso and upperbody joint reference
* Added tor wbc standalone
* Created tor kinematic, dynamic and walking force control tor
* Added joint state controller that uses torque sensor
* Added individual filter gains
* First experiments tor hardware
* Added skeleton for joint state controller with real torque sensor feedback
* Initial commit
* Contributors: Adrià Roig, Hilario Tome, Hilario Tomé, Victor Lopez, davidfernandez, sergi.garcia
