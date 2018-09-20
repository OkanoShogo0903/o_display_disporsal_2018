# 2018WRSの陳列廃棄用
わからない部分はREADMEか筆者本人に聞いて.  

## 使い方
以下のコマンドを実行すると、依存するパッケージを順に時間差で実行していく.  

```
$ roslaunch o_display_disporsal_2018 display_disporsal.launch
```

## Insall
- start.sh  
script/start.shに実行権を付与する(?)  

- realsense  
[demura.netの記事](http://demura.net/athome/14741.html)を参考にしてinstallしてください.  
もしFarmwareの更新で躓いているようならば、このREADMEと同じディレクトリにあるfarmware_update_diaryを参照してみてください.  

- aruco_ros  
```
$ git clone https://github.com/pal-robotics/aruco_ros.git
```
aruco_rosのlaunchファイルは製作者のシミュレータ上で動かす用にデフォルト変数が設定されていたため、それを設定する必要がある.  
marker_publisher.launchを以下のように書き換える.  

```
<remap from="/camera_info" to="/camera/color/camera_info" />
<!-- <remap from="/camera_info" to="/cameras/$(arg side)_hand_camera/camera_info" /> -->
<remap from="/image" to="/camera/color/image_raw" />
<!-- <remap from="/image" to="/cameras/$(arg side)_hand_camera/image" /> -->
```

## 実行環境

研究室ノートPC,No2  
- Ubuntu16.04
- Ros, Kinetic
- python2.7.12

## packageの依存
- [realsense2_camera](http://wiki.ros.org/realsense2_camera)  
Realsenseの起動用.

- [aruco_ros](https://github.com/pal-robotics/aruco_ros)
ArUcoマーカの認識系をros化させたもの.  
[このサイト](http://chev.me/arucogen/)で生成できる.

- [wheel](https://github.com/OkanoShogo0903/wheel)  
研究室で作られた2010年ぐらいのプログラム.  

## Note  
realsenseのノードを立ち上げるときにエラーが出やすく、そこでエラーが出た場合は起動し直すことを推奨.  

