# robotis_op_utility
## odom_pub.cpp (robotis_op_odom_pub)
### 概要
移動量の指令値から自己位置を推定することで、
推定のワールド座標をtfとtopicに配信するノード。

### 使用方法
`$ rosrun robotis_op_utility robotis_op_odom_pub (cmd_vel or Pose2D)`

### 必要要件
- Robotis-OPの動作指令が、/robotis_op/cmd_vel、もしくは/robotis_op/Pose2Dのいずれかのトピックで配信されている
- /base_linkをロボットのローカル座標としたRobotis_opのtf treeが構築されている
    - joint_state_publisherノードが実行されていればよい

### TODO
- 動作指令をgeometry_msgs::Poseにも対応
- 推定式の改良
    - 特にPose2Dが不安
