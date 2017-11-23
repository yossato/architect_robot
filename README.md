
# architect_robot
このパッケージはライントレースロボットがコースを周回するタイムを競う「ロボトレース」競技のROSとGazeboによるシミュレータです。    
![image](https://github.com/yossato/architect_robot/blob/master/materials/gazebo.png)    
[youtubeで見る](https://youtu.be/G1btov_RhXU)    
このシミュレータには、ロボトレースのコースとサンプルとなるロボットが含まれています。    
ロボットはステレオカメラと、IMUを搭載しています。いまのところ自律移動やSLAMは行えません。

ROSのバージョンはKineticに対応します。    
ロボットの操作はGUIからでも、PS3コントローラからでも可能です。

# 使い方  
## シミュレータの起動    
### PS3 コントローラを使用する場合    
```    
$ roslaunch architect_gazebo robotrace_ps3.launch    
```    
### rqtで操作する場合      
```    
$ roslaunch architect_gazebo robotrace_cource.launch    
```    
rqtのTopic名に/diff_drive_controller/cmd_velを入力する。    
# 関連ライブラリのインストールと初期設定    
## 共通のライブラリ    
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control    
## PS3コントローラを使用する場合のライブラリのインストールと初期設定
PS3 コントローラを使用しない場合は本手順は不要    
```    
$ roslaunch architect_gazebo robotrace_cource.launch    
$ sudo apt-get update && sudo apt-get install git dialog build-essential pyqt4-dev-tools libusb-dev libjack-dev libbluetooth-dev python-dbus -y    
$ git clone --depth 1 https://github.com/falkTX/qtsixa.git    
$ cd qtsixa    
$ make -j4    
$ sudo make install    
    
$ sudo apt-get install ros-kinetic-joy    
$ sudo apt-get install ros-kinetic-teleop-twist-joy    
```    
最初にUSBケーブルでPS3コントローラとPCを接続する。次に以下のコマンドを打つ。    
```    
$ sudo sixpair    
```    
USBケーブルをコントローラから抜く    
```    
$ sixad --start    
```    
以下のような表示がされたら多分成功している。    
```    
[ ok ] Starting bluetooth (via systemctl): bluetooth.service.    
sixad-bin[7337]: started    
sixad-bin[7337]: sixad started, press the PS button now    
sixad-sixaxis[7343]: started    
sixad-sixaxis[7343]: Connected 'PLAYSTATION(R)3 Controller (XX:XX:XX:XX:XX:XX)' [Battery 04]    
```    
再起動してもsixpairを起動するにはrc.localの'exit 0'の直前にコマンドを追加    
```    
$ sudo nano /etc/rc.local     #rc.localを開く    
```    
以下のコマンドをexit 0の前に追記    
```    
sixad --start    
```    

# 作者
Yoshiaki Sato    
    
本パッケージは以下のリポジトリを非常に参考にし作成しました。    
https://github.com/CIR-KIT/fourth_robot_pkg
