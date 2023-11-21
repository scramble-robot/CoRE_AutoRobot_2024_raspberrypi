# SDカード

1. Raspberry Pi ImagerでRaspberry Pi OS Lite (64-bit,bullseye)をMicro SDに書き込む。Imagerの設定でユーザ名**pi**とパスワードを予め設定すること。
2. Micro SDに作成された"bootfs"ドライブ直下に`ssh`という名前のファイルを配置してSSHを有効化する。
3. OS起動後、本レポジトリの`ros2_ws`及び`service`ディレクトリをホームディレクトリ(`/home/pi/`)にコピーする。

# C620とのCAN通信

MCP2515を用いてSPIからCANを制御

- (参考) [RaspberryPiとMCP2515でCAN通信をやってみる](https://qiita.com/h-kiyo/items/d8583af13768ad67bcd0)

## ハードウェア
1Mbpsの通信を行うために、水晶振動子を**16MHz**に交換すること

## CAN-BUSの有効化
1. `/boot/config.txt`を編集してSPIとMCP2515を有効化する
    ```bash
    sudo vi /boot/config.txt
    ```

    ```
    # Uncomment some or all of these to enable the optional hardware interfaces
    #dtparam=i2c_arm=on
    #dtparam=i2s=on
    dtparam=spi=on

    # MCP2515
    dtparam=spi=on
    dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
    dtoverlay=spi-bcm2835-overlay
    ```

2. 初期化できているか確認
    ```
    dmesg | grep mcp251x
    ```

## can-utilsのインストール
```bash
sudo apt install can-utils
```

## 通信テスト

1Mbpsでlink up

```bash
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 1000
sudo ifconfig
```

### 受信テスト
```bash
candump can0
```

C620と通信できていると、以下のようなデータが受信される。
```text
  can0  201   [8]  0E 16 00 00 FF FD 21 00
  can0  201   [8]  0E 16 00 00 FF FD 21 00
  can0  201   [8]  0E 16 00 00 FF BE 21 00
  can0  201   [8]  0E 16 00 00 FF FD 21 00
  can0  201   [8]  0E 16 00 00 00 3D 21 00
```

### 送信
モータ回るはず
```bash
cansend can0 200#0200000000000000
```

## systemdによるCANバスの自動有効化

1. systemdの設定
    ```bash
    sudo vi /etc/systemd/network/80-can.network
    ```

    ```text
    [Match]
    Name=can*

    [CAN]
    BitRate=1000000
    ```

    設定後、以下コマンド実行
    ```bash
    sudo systemctl enable systemd-networkd
    sudo systemctl start systemd-networkd
    ```

2. txqueuelenの自動設定
    ```bash
    sudo vi /etc/udev/rules.d/71-can-txqueuelen.rules
    ```

    ```
    SUBSYSTEM=="net", ACTION=="add|change", KERNEL=="can*", ATTR{tx_queue_len}="1000"
    ```

3. 再起動後、`ifconfig`でlink upしていることを確認

    ```
    can0: flags=193<UP,RUNNING,NOARP>  mtu 16
            unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 1000  (UNSPEC)
            RX packets 0  bytes 0 (0.0 B)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 0  bytes 0 (0.0 B)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
    ```

## Pythonからの制御

```bash
pip install python-can
```

```python
import can
import struct
import time


bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=1000000)

def set_current(bus, currents):
    d = [0]*8
    raw_array = [0]*4
    for i in range(4):
        v = int(round(currents[i]/20 * 16384))
        v = min(v, 16384)
        v = max(v, -16384)
        raw_array[i] = v
    b = struct.pack('>4h', *raw_array)
    msg = can.Message(arbitration_id=0x200, is_extended_id=False, data=b)
    bus.send(msg)

def parse_c620_data(msg: can.Message):
    data = struct.unpack('>hhhbb', msg.data)
    degree = data[0]/ 8192 * 360
    rpm = data[1]
    current = data[2]/16384*20
    temperature = data[3]
    return degree, rpm, current, temperature

def can_receive_callback(msg: can.Message) -> None:
    #print(msg)
    if msg.arbitration_id >= 0x201 and msg.arbitration_id < 0x204:
        degree, rpm, current, temperature = parse_c620_data(msg)
        print(f'{degree:.1f}\t{rpm:+5d}\t{current:.1f}\t{temperature:2d}')

notifier = can.Notifier(bus, [can_receive_callback])

set_current(bus, [1, -1, 0, 0])
time.sleep(1)

set_current(bus, [-1, 1, 0, 0])
time.sleep(1)

for i in range(4):
    set_current(bus, [0, 0, 0, 0])
    time.sleep(1)

notifier.stop()
bus.shutdown()
```

# サーボモータ
## ハードウェア
- 信号線はGPIO17に接続
- サーボモータの5Vは外部電源から取ること

## requirements
```bash
sudo apt install -y pigpio python3-pigpio
sudo pigpiod
sudo systemctl enable pigpiod
```

## 動作確認

```python
import pigpio

output_pin = 17


def setServoDegree(pi, output_pin, degree):
    min_width = 500
    max_width = 2500
    deg_range = 270
    output = degree / deg_range
    output *= max_width - min_width
    output += min_width
    output = int(output)
    pi.set_servo_pulsewidth(output_pin, output)


pi = pigpio.pi()
pi.set_mode(output_pin, pigpio.OUTPUT)
pi.set_servo_pulsewidth(output_pin, 0)

while True:
    print("input Duty Cyle (0 - 270)")
    degree = float(input())
    setServoDegree(pi, output_pin, degree)
```

# ROS2のインストール

Raspbian OS用の、[非公式のdebパッケージ](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)を使ってインストールする

```bash
wget https://github.com/Ar-Ray-code/rpi-bullseye-ros2/releases/download/ros2-0.3.1/ros-humble-desktop-0.3.1_20221218_arm64.deb
sudo apt install ./ros-humble-desktop-0.3.1_20221218_arm64.deb
sudo pip install vcstool colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ROS2パッケージのビルド
```
cd ~/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## ROS2ノードの実行
```bash
ros2 launch scramble_auto_robot hardware.launch.py
```

## 確認
```bash
ros2 topic pub /can_node/c620_0/target_current std_msgs/Float64 "{data: 1.0}"
ros2 topic pub /can_node/c620_0/target_current std_msgs/Float64 "{data: 0.0}"
ros2 topic pub /servo0/degree std_msgs/Float64 "{data: 0}"
ros2 topic pub /servo1/degree std_msgs/Float64 "{data: 0}"
ros2 topic echo /can_node/c620_0/rpm
```

## 自動起動
```bash
sudo ln -s /home/pi/service/scramble_auto_robot_ros2.service /etc/systemd/system
chmod 755 /home/pi/service/ros2_launch.sh
sudo systemctl enable scramble_auto_robot_ros2.service
sudo systemctl start scramble_auto_robot_ros2.service
systemctl status scramble_auto_robot_ros2.service
```

# 起動オプションの変更
1. ターミナルで`sudo raspi-config`を実行
2. "1. System Options"を選択
3. "S6 Network as Boot"でネットワーク確率までBootを待つように設定
4. "S5 Boot / Auto Login"で"Console Autologin"を選択

# SDカードのRead only化
SDカードへの書き込み中に電源を抜くとSDカードが壊れる可能性があるので、read-onlyに設定する。

参考: [Raspberry Pi で Overlay File System (read-only file system) を試す](https://qiita.com/nanbuwks/items/d5d0cfc5f94177515a6a)

1. ターミナルで`sudo raspi-config`を実行
2. "4 Performance Options Configure performance settings"
3. "P3 Overlay File System Enable/disable read-only file system"
4. 有効化して再起動


# その他
- ros2_launch.shにて`ROS_DOMAIN_ID=0`に設定しています。
- デフォルトの設定ではIPアドレスの割当はDHCPで行われます。

# Third-party license
本手順で作成した場合、以下のサードパーティ製ソフトウェアがインストールされます。
(本レポジトリに直接は含まれていません)

- Raspbian OS
- ROS2 Humble
- [rpi-bullseye-ros2](https://github.com/Ar-Ray-code/rpi-bullseye-ros2/blob/main/LICENSE) (MIT License)
- [python-can](https://pypi.org/project/python-can/) (LGPL v3)
- [pigpio](https://github.com/joan2937/pigpio/blob/master/UNLICENCE) (Unlicense)

デバッグ用
- [can-utils](https://github.com/linux-can/can-utils/tree/master/LICENSES)
