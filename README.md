# robotis_op_utility
## odom_pub.cpp (robotis_op_odom_pub)
### 概要
移動量の指令値から自己位置を推定することで、
推定のワールド座標をtfとtopicに配信するノード

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


## robotis_body.cpp
### 概要
Robotis-OP2の操作、運動学計算、初期姿勢移行などを行う関数群。

### 使用方法
具体的な使用例はrobotis_op_cpwalk/main.cppを参考。
```c++
#include <robotis_op_utility/robotis_body.h>
```

### 必要条件
- moveit!ライブラリが使用可能な状態であること
    - 公式で配布されているrobotis_op_moveitパッケージに、さらに両足それぞれのグループを作成する
- Eigen3がインストールされていること
- robot_descriptionパラメータにRobotis-OP2のURDFファイルが読み込まれていること

### TODO
- CoM計算が間違ってる可能性がある（特にx, y軸方向）
    - 初期姿勢の時点で中心から数cmずれる