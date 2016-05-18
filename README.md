# rtt-gazebo-ur5-integration
RTT-Gazebo components for Universal Robots UR5

* Based on: http://cogimon.github.io/software/gettingstarted.html
* Consider `bash configurations` for environment variables
* Don't forget to check or set up `rsb.conf` settings in `$HOME/.config`


## start gz server
`./gzserver -s $PREFIX/lib/orocos/gnulinux/rtt_gazebo_system/librtt_gazebo_system.so $PREFIX/etc/cogimon-scenarios/scenario-wipe-board/world/scn-wipe-board-empty.world`

## start gz client
`./gzclient`

## load robot urdf model
`$PREFIX/bin/rsb-toolscl0.13 call 'socket:/GazeboDeployerWorldPlugin/spawnModel("/homes/yourName/workspace/cogimon-gazebo-models/universal-robot-ur5/ur5robot.urdf")'`

## start orocos component
`./rsb-toolscl0.13 call -l $PREFIX/share/rst0.13/proto/sandbox/rst/cogimon/ModelComponentConfig.proto 'socket:/GazeboDeployerWorldPlugin/deployRTTComponentWithModel(pb:.rst.cogimon.ModelComponentConfig:{component_name:"ur5_gazebo" component_type:"UR5RttGazeboComponent" component_package:"rtt-gazebo-ur5-integration" model_name:"universal-robot-ur5" script:"/homes/yourName/workspace/rtt-gazebo-ur5-integration/scripts/ur5.ops"})'`

* `component_name:"ur5_gazebo"`	 any name you use in your (ops) script
* `component_type:"UR5RttGazeboComponent"`  component class name 
* `component_package:"rtt-gazebo-ur5-integration"`  package containing the component, e.g. project folder name
* `model_name:"universal-robot-ur5"`  name of the pre-loaded (gazebo)model to attach
* `script:"...."`  ops or lua script to apply further configurations
