9/18 (火) 10:00 ~ 21:00

#ToDo
RealsenseD435の環境設定

- Firmwareの書き換え

# Do
## Demura.net編
[demura.net](http://demura.net/athome/14741.html)の記事に沿って環境設定を行っていた.  
道中でAnacondaを入れていたためにエラーが出たが、使用するpythonをsystemのものに切り替えて回避.
catkin_makeとinstallが通るところまでは済んだ.
しかし、rosrunで動かそうとするとエラーを吐く.
**原因をFarmwareのヴァージョンが古いためであると考えた**

```
Current firmware version: 05.08.15.00
Minimal firmware version: 05.09.14.00
```

## Farmware更新編
[IntelのFarmware更新記事](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Linux-RealSense-D400-DFU-Guide.pdf)を見て、更新を試みる.
しかし、`sudo apt-get update`で以下のエラーがでた.

```
無視:26 http://realsense-hwpublic.s3.amazonaws.com/Debian/apt-repo xenial/main DEP-11 64x64 Icons                                                                                                                  
無視:31 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main DEP-11 64x64 Icons                                                                                                                  
エラー:20 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main amd64 Packages
  403  Forbidden
無視:22 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main i386 Packages
無視:25 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main all Packages
無視:27 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main Translation-ja_JP
無視:28 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main Translation-ja
無視:29 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main Translation-en
無視:30 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main amd64 DEP-11 Metadata
無視:31 http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial/main DEP-11 64x64 Icons
323 kB を 15秒 で取得しました (21.4 kB/s)
パッケージリストを読み込んでいます... 完了
W: リポジトリ http://realsense-hwpublic.s3.amazonaws.com/Debian/apt-repo xenial Release には Release ファイルがありません。
N: このようなリポジトリから取得したデータは認証できないので、データの使用は潜在的に危険です。
N: リポジトリの作成とユーザ設定の詳細は、apt-secure(8) man ページを参照してください。
W: リポジトリ http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo xenial Release には Release ファイルがありません。
N: このようなリポジトリから取得したデータは認証できないので、データの使用は潜在的に危険です。
N: リポジトリの作成とユーザ設定の詳細は、apt-secure(8) man ページを参照してください。
E: http://realsense-hwpublic.s3.amazonaws.com/Debian/apt-repo/dists/xenial/main/binary-amd64/Packages の取得に失敗しました  404  Not Found
E: http://realsense-hw-public.s3.amazonaws.com/Debian/aptrepo/dists/xenial/main/binary-amd64/Packages の取得に失敗しました  403  Forbidden
E: いくつかのインデックスファイルのダウンロードに失敗しました。これらは無視されるか、古いものが代わりに使われます。
```

これはファイルが見つからないと言われている以上よくわからん.
そのため、以下の方法によって解決を図る.

1. Windowsでインストールを試す.
2. Firmware更新を別のPC（エイリアン）で試す

## Windowsでインストールを試す案

[Interの指示どおりにやってみた](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/FW_Update_for_RealSense_D400_Guide.pdf).
管理者権限で実行時にエラーでた.
[RealsenseD435FarmWareをWindows7でインストールしたときのエラーをおいておく](https://i.gyazo.com/610966952ffdbbe952ea3526ae5154f5.png) 

エラーコードで検索してもそれらしいものはヒットせず.
おそらく、Intelの推奨がWindowns10なのに対して自分のWindowsが7であるために出たエラーかと考えられる.
そのため、Windows10のPCを持っている人を探して同じエラーが出るか確かめる.

Windows10だとインストーラ自体は正常に動作することを確認.
しかし、2台のWindows１０PCで試したがどちらもエラーを吐く.
Intel公式の掲示板曰く、そのエラーは何かしらのソフトウェアの競合が理由かと考えられる.
これは解決が難しいと考えて次のエイリアンでFirmwareを更新する方を試す.

## エイリアンでFirmwareを更新
自分のノートPCと同じ.
ただ、エラーコードで調べると何回も抜き差しするとUSBの認識形式（USB2か3か)が変わるためにFirmwareの更新がうまくいくかもしれないというヒントを得る.
ただ、抜き差ししてみるも、特に変化はなかった.
これによってエイリアンで使っていたマウスが使えなくなった.

## 別のケーブルを試す.
私はこれでlinuxから最新バージョンのファームウェア書き換えがうまくいった.
一応環境の情報を載せておく.

```
4.15.0-34-generic
Python 2.7.15
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=16.04
DISTRIB_CODENAME=xenial
DISTRIB_DESCRIPTION="Ubuntu 16.04.5 LTS"
kinetic
```
