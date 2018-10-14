# WRS-2018-DisplayDisporsal
陳列廃棄タスク用のプログラム.  
わからない部分はREADMEか筆者本人に聞いて.  

## HowToUse
`ls /dev/`で`ttyUSB`を確認.  
`wheel/src/main.cpp`の`PORT_NAME`を変更.  
以下のコマンドを実行すると、依存するパッケージを順に時間差で実行していく.  

```
$ roslaunch o_display_disporsal_2018 display_disporsal.launch
```

## Insall
- o_display_disporsal_2018  

```
git clone https://github.com/OkanoShogo0903/o_display_disporsal_2018
```

- realsense  
[demura.netの記事](http://demura.net/athome/14741.html)を参考にしてinstallしてください.  
もしFarmwareの更新で躓いているようならば、このREADMEと同じディレクトリにあるfarmware_update_diaryを参照してみてください.  

- aruco_ros  
```
$ git clone https://github.com/pal-robotics/aruco_ros.git
```
aruco_rosのlaunchファイルは製作者のシミュレータ上で動かす用にデフォルト変数が設定されていたため、それを設定する必要がある.  
`marker_publisher.launch`の該当行を以下のように書き換える.  

```
<arg name="ref_frame"       default="0"/>

<remap from="/camera_info" to="/camera/color/camera_info" />
<!-- <remap from="/camera_info" to="/cameras/$(arg side)_hand_camera/camera_info" /> -->
<remap from="/image" to="/camera/color/image_raw" />
<!-- <remap from="/image" to="/cameras/$(arg side)_hand_camera/image" /> -->
```

## 実行環境
動作が確認されている環境を以下に示します.

- LabNotePC.No2  
```
Ubuntu16.04
Ros, Kinetic
python2.7.12
```

- MyNotePC
```
4.15.0-34-generic
Python 2.7.12
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=16.04
DISTRIB_CODENAME=xenial
DISTRIB_DESCRIPTION="Ubuntu 16.04.5 LTS"
kinetic
```

## packageの依存
- [realsense2_camera](http://wiki.ros.org/realsense2_camera)  
Realsenseの起動用.

- [aruco_ros](https://github.com/pal-robotics/aruco_ros)  
ArUcoマーカの認識系をros化させたもの.  
[このサイト](http://chev.me/arucogen/)で生成できる.

- [wheel](https://github.com/OkanoShogo0903/wheel)  
研究室で作られた2010年ぐらいのプログラム.  

## ChangeLog
- v1.0.0  
launchで動作  

## Note  
realsenseのノードを立ち上げるときにエラーが出やすく、そこでエラーが出た場合は起動し直すことを推奨.  

## CheckList
### Hard
緊急停止スイッチ、ジョイスティック、ホイール切り替えスイッチの確認.
グラグラさせてカチッとさせる. <-- 重要

### Soft
ls /devでPORT_NAME確認
cm
rosrun wheel wheel

