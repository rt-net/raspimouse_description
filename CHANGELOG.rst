^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspimouse_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-11-08)
------------------
* Support ROS 2 Jazzy (`#55 <https://github.com/rt-net/raspimouse_description/issues/55>`_)
  Co-authored-by: Kazushi Kurasawa <Kurasawa@rt-net.jp>
* Replace `ign` to `gz`
* Updated the remapping for subscription to reflect the removal of `unstamped` from `ros2_controllers <https://github.com/ros-controls/ros2_controllers/blob/57c50e584e33b316dd64801916cf6d951e0cff5b/tricycle_controller/CHANGELOG.rst#4100-2024-07-01>`_
* Contributors: YusukeKato

1.2.0 (2024-03-05)
------------------
* シミュレータ環境でscanトピックをpublish (`#52 <https://github.com/rt-net/raspimouse_description/issues/52>`_)
* camera_downwardがtrueのときRGBカメラが斜め下を向くように変更 (`#50 <https://github.com/rt-net/raspimouse_description/issues/50>`_)
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* READMEにRGBカメラを表示するコマンドを追記 (`#47 <https://github.com/rt-net/raspimouse_description/issues/47>`_)
* Gazebo上で画像トピックを配信できるように変更 (`#46 <https://github.com/rt-net/raspimouse_description/issues/46>`_)
* RGBカメラのモデルを表示できるように変更 (`#45 <https://github.com/rt-net/raspimouse_description/issues/45>`_)
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* cmd_velとodomのトピック名をremapping (`#44 <https://github.com/rt-net/raspimouse_description/issues/44>`_)
* controller managerが起動するように変更 (`#43 <https://github.com/rt-net/raspimouse_description/issues/43>`_)
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* robot_description_loaderを実装 (`#42 <https://github.com/rt-net/raspimouse_description/issues/42>`_)
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Contributors: YusukeKato

1.1.0 (2023-11-07)
------------------
* READMEにRGBカメラを表示するコマンドを追記 (`#47 <https://github.com/rt-net/raspimouse_description/issues/47>`_)
* Gazebo上で画像トピックを配信できるように変更 (`#46 <https://github.com/rt-net/raspimouse_description/issues/46>`_)
* RGBカメラのモデルを表示できるように変更 (`#45 <https://github.com/rt-net/raspimouse_description/issues/45>`_)
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* cmd_velとodomのトピック名をremapping (`#44 <https://github.com/rt-net/raspimouse_description/issues/44>`_)
* controller managerが起動するように変更 (`#43 <https://github.com/rt-net/raspimouse_description/issues/43>`_)
  Co-authored-by: Shota Aoki <s.aoki@rt-net.jp>
* robot_description_loaderを実装 (`#42 <https://github.com/rt-net/raspimouse_description/issues/42>`_)
  Co-authored-by: Daisuke Sato <daisuke.sato@rt-net.jp>
* Contributors: YusukeKato

1.0.1 (2023-01-31)
------------------
* Updates README
* Update the license year to 2022
* Adds LiDAR (`#38 <https://github.com/rt-net/raspimouse_description/issues/38>`_)
* Update library to rviz2 (`#36 <https://github.com/rt-net/raspimouse_description/issues/36>`_)
* Update tread (ROS 2) (`#32 <https://github.com/rt-net/raspimouse_description/issues/32>`_)
* Update README for ROS 2 (`#26 <https://github.com/rt-net/raspimouse_description/issues/26>`_)
* Support ROS 2 Foxy (`#24 <https://github.com/rt-net/raspimouse_description/issues/24>`_)
* Create LICENSE
* Contributors: Daisuke Sato, Shota Aoki, Shuhei Kozasa
