
# architect_robot
このパッケージはライントレースロボットがコースを周回するタイムを競う「ロボトレース」競技のROSとGazeboによるシミュレータです。    
〈ここに競技のYouTube 貼る〉    
〈ここにシミュレーターのYouTube 貼る〉    
このシミュレータには、ロボトレースのコースとサンプルとなるロボットが含まれています。    
シミュレータはROS KineticとGazeboで動作します。    

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
# ROSのインストール    
# 関連ライブラリのインストール  
## PS3コントローラを使用する場合
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
sixad-sixaxis[7343]: Connected 'PLAYSTATION(R)3 Controller (28:A1:83:4A:36:E6)' [Battery 04]    
```    
再起動してもsixpairを起動するにはrc.localの'exit 0'の直前にコマンドを追加    
```    
$ sudo nano /etc/rc.local     #rc.localを開く    
```    
以下のコマンドをexit 0の前に追記    
```    
sixad --start    
```    

