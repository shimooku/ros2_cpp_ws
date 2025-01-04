#  ros2_cpp_ws

## What I have learned about ROS 2

Udemy の **ROS2 C++ Robotics Developer Courses - Using ROS2 in C++** という講座(適当なものとして英語版しか無かった)を通してROS2を学習しています。

最後までこの講座を学習したものの、記憶や頭の整理が不十分なので、2回目を見始めています。

> 自分の手順に自信が持てないため、正常に動作しないとすべてを最初からインストールし直すような有り様で、進捗が悪いのが実情....

以下は、学習時に作成したプログラムの説明(ビルド方法と動かし方を含む)です。

> 講師の説明どおりにコーディングしたプログラムです。

## フォルダ構成

```
udemy_ros2_pkg/
├── action
│   └── Navigate.action
├── CMakeLists.txt
├── images
│   ├── 0.png
│   ├── -15.png
│   ├── 15.png
│   ├── -30.png
│   └── 30.png
├── include
│   └── udemy_ros2_pkg
├── launch
│   ├── launch_project.launch.py
│   ├── rpm_node.launch.py
│   ├── simulation_project.launch.py
│   └── wheeled_model_simulation.launch.py
├── models
│   ├── ground_plane
│   │   ├── ground_plane.sdf
│   │   └── model.config
│   ├── project_robot_model
│   │   ├── model.config
│   │   └── model.sdf
│   └── wheeled_model
│       ├── model.config
│       └── model.sdf
├── package.xml
├── src
│   ├── action_client.cpp
│   ├── action_server.cpp
│   ├── publisher.cpp
│   ├── rpm_pub.cpp
│   ├── service_client.cpp
│   ├── service_server.cpp
│   ├── speed_calc.cpp
│   ├── subscriber.cpp
│   ├── turn_camera_client.cpp
│   └── turn_camera_service.cpp
├── srv
│   ├── OddEvenCheck.srv
│   └── TurnCamera.srv
└── worlds
    ├── project_world.sdf
    ├── test_world.sdf
    └── wheeled_model_world.sdf

```

## ビルド方法

作成したプログラムすべての設定が CMakeLists.txt, package.xml に入っています。

1. ros2_cpp_ws のフォルダに移動
2. 以下のコマンドを実施
   ```
   colcon build
   source install/setup.bash
   ```

## Publisher and Subscriber

#### 学習したこと

* Pub-Subの通信方式と、パラメータのハンドリング
* Pythonを使った各処理の起動

#### 作成コード

| コード |node | **topic** |param|内容|
| ------------- | --|---- |--|--|
| rpm_pub.cpp |rpm_pub_node |rpm   |rpm_val  |rpm_valをgetして、rpmでpub|
| speed_calc.cpp |speed_calc_node |rpm, speed   | wheel_radius |rpmをsubして、wheel_radiusをgetし、それらで計算した結果をspeedでpub|


| laucher|内容|
|--|--|
| launch_project.launch.py|rpm_pub_nodeをrpm_val指定付きで起動、speed_calc_nodeをwheel_radius指定付きで起動し、計算結果のspeedを監視する|

#### 実行方法

1. Terminal#1で以下コマンドを実行
   ```
   ros2 launch udemy_ros2_pkg launch_project.launch.py
   ```
   * スクリプト内で指定したwheel_radius値に対して、計算したspeedを1Hzで画面出力し始める

2. Terminal#2️で以下コマンドを実行
   ```
   ros2 pram set /speed_calc_node wheel_radius 0.5
   ```
   * これによってTerminal#1の数値表示が 0.2617993877991494 に変わる

## Server

#### 学習したこと

* rosidlを使ったサーバ・クライアントの実装
* Req-Resの通信方式

#### 作成コード

| コード |node | **service** |内容|
|--|--|--|--|
| service_client.cpp |odd_even_check<br/>_client_node |odd_even_check|Userが入力した数値をサーバに非同期Reqし、そのResを表示|
| service_server.cpp |odd_even_check<br/>_service_node |odd_even_check|Reqで送信された数値の奇数偶数判断をした結果をRes|

|ファイル|内容|
|-|-|
|srv/OffEvenCheck.srv|Req(数値), Res(文字列)のインターフェース定義（C++コード自動生成用）|

#### 実行方法

1. Terminal#1で以下を実行
   ```
   ros2 run udemy_ros2_pkg service_server
   ```

2. Terminal#2で以下を実行
   ```
   ros2 run udemy_ros2_pkg service_client
   ```
   * Terminal#2に出てくるプロンプトに対して数値を入力すると、その回答（偶数か奇数か）が同じ画面に表示される

## Action

#### 学習したこと

* Actionによる通信 (clientのアクション関数をserverが呼び出す)

#### 作成コード

| コード |node | **action** |内容|
|--|--|--|--|
| action_client.cpp |navigate_action<br/>_client_node |navigate|Userが入力したゴール地点と一緒にfeedback, secceed時のコールバックを指定して、サーバに処理を依頼(Req)。|
| action_server.cpp |navigate_action<br/>_service_node |navigate|action_clientから送られてきたgoal_positionと、subしているrobot_positionとの距離を計算して、指定距離以下になるまで、その距離をパラメータにしてaction_clientのfeedbackを呼び出す。指定以下になったらsucceedを呼び出す。|

#### 実行方法

1. Terminal#1で以下を実行
   ```
   ros2 run udemy_ros2_pkg action_server
   ```

2. Terminal#2で以下を実行
   ```
   ros2 run udemy_ros2_pkg action_client
   ```
   * Terminal#2に出てくるプロンプトに対して数値(ゴール地点XYZ)を入力すると、その回答（距離）が同じ画面に表示される。
   * 距離が十分に近い(0.1m以下)と終了。

3. Terminal#3で以下のフォーマットでrobot_positionを指定
   ```
   ros2 topic pub /robot_position --once geometry_msgs/msg/Point "{ x: 1.0, y: 1.0, x: 0.0}"
   ```
## Bridge

Gazeboの使い方を通してBridgeを学習
> Gazebo classicをインストールする前に Ignition Gazebo をインストールしないと Ignition Gazebo が動作しないので要注意 (これにも嵌まった...)

### 車輪ロボットを動かす#1

Ignition Gazeboロボットを動かす (Bridgeは使ってません)

#### 学習したこと

* Ignition Gazebo上にロボットを配置して、Ignitionコマンドでロボットを動かす
  * **ROS2**コマンドではなく**Ignition**コマンドで動かす

* 使用した環境
  * Gazebo classic
  * Ingition Gazebo
   > VMware上のUbuntuでIgnition Gazeboを表示する際には `--render-engine ogre` オプションを追加する必要あり。これをしないと正常に表示できない。これに嵌まった...

#### 作成コード

|ファイル名|内容|
|-|-|
|models/ground_plane|https://app.gazebosim.org/fuel/models から平面モデルをダウンロード|
|models/wheeled_model|Gazeboクラシックを使って車輪ロボットをデザイン。前方に滑るだけのキャスタ、後方に2輪がある形状。|
|worlds/wheeled_model_world.sdf|ground_planeとwheeled_modelをincludeしたもの|

#### 実行方法

1. Termintal#1で以下を実行
   ```
   ign gazebo --render-engin ogra wheeled_model_world.sdf
   ```
   * 画面左下の▶ボタンを押して動作状態にする (これにも嵌まった...)

2. Termintal#2でignコマンドを使って車輪ロボットを制御
   ```
   ign topic -t "/model/wheeled_model/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
   ```
   * `{x: 0.0}` と `{z: 0.0}` の数値を0以外にすると動き出す。負の値を設定する逆方向に。

### 車輪ロボットを動かす#2

* Ignition Gazeboロボットをrqt(ROS2)で動かす

#### 学習したこと

* ros_ign_bridgeノードを使ってGazeboとROS2をコネクションし、qrtでignition Gazeboのロボット制御
* 検証
  * ROS2の/cnd_velがingnitionの/model/wheeled_model/cmd_velに変換されるのか...
  1. コマンドライン操作(1行)でブリッジを起動
    ```
    ros2 run ros_ign_bridge parameter_bridge /model/wheeled_model/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist --ros-args -r /model/wheeled_model/cmd_vel:=cmd_vel
    ```
  2. rqtを起動してlinearのx、anmularのzの値を変更してignition Gazeboロボットの動作を確認
    * Plugins -> Topics -> Message Publisher を表示し、Topicから /cmd_vel を選択して表示しておく

#### 作成コード

|ファイル名|内容|
|-|-|
|launch/wheeled_model_simulation.launch.py|ignition Gazeboとros_ign_bridgeとrqtを同時立上げするスクリプト|

#### 実行方法

* Terminal#1で以下を実行
  ```
  ros2 launch udemy_ros2_pkg wheeled_model_simulation.launch.py
  ```
  * 表示される rqt の画面でlinearのx、anmularのzの値を変更してignition Gazeboロボットの動作を確認
    * Plugins -> Topics -> Message Publisher を表示し、Topicから /cmd_vel を選択して表示しておく

### 車輪ロボットを動かす#3

* カメラセンサを搭載したIgnition Gazeboロボットをrqt(ROS2)で動かす
* 映像のリアルタイム取得

#### 学習したこと

* sdfデータ拡充によるロボット機能強化

#### 作成コード

|ファイル名|内容|
|-|-|
|models/project_robot_model|4輪車にグレードアップ。Gazeboクラシックでデザインしたsdfにcameraセンサー定義を追加。|
|worlds/project_world.sdf|ground_planeとproject_robot_modelを配置|
|launch/simulation_project.launch.py|ignition Gazeboとros_ign_bridgeとrqtを同時立上げするスクリプト|

#### 実行方法

* Terminal#1で以下を実行
  ```
  ros2 launch udemy_ros2_pkg simulation_project.launch.py
  ```
  * 表示される rqt の画面でlinearのx、anmularのzの値を変更してignition Gazeboロボットの動作を確認
    * Plugins -> Topics -> Message Publisher を表示し、
      * Topicから /cmd_vel を選択して表示して、linear/x angular/z のパラメータ変更による動作変化を確認
      * Topicから /camera_rod_pos_cmd を選択して表示して、data のパラメータ変更による動作変化を確認
    * Plugins -> Visualization -> Image View を選択して表示して、
      * 物体を新規に配置したり、
      * ロボットを動かした時のカメラ映像が変わる事を確認

* 別途開発したmyshでこのロボットをマニュアル制御
  * 詳しいところは https://github.com/shimooku/mysh の説明に含めました
