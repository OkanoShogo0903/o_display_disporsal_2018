## 2018WRSの陳列廃棄用

## TODO

## Insall
aruco_rosのlaunchファイルは製作者のシミュレータ上で動かす用にデフォルト変数が設定されていたため、それを設定する必要がある.
marker_publisher.launchを以下のように書き換える.

```
<remap from="/camera_info" to="/camera/color/camera_info" />
<!-- <remap from="/camera_info" to="/cameras/$(arg side)_hand_camera/camera_info" /> -->
<remap from="/image" to="/camera/color/image_raw" />
<!-- <remap from="/image" to="/cameras/$(arg side)_hand_camera/image" /> -->
```

## packageの依存
- realsense2_camera
- aruco_ros
ArUcoマーカの認識系をros化させたもの.
- wheel
研究室で作られた2010年ぐらいのプログラム.

## 使い方
```
$ sh ./start.sh
```

## 実行環境
研究室ノートPC,No2
- Ubuntu16.04
- Ros, Kinetic
- python2.7.12
