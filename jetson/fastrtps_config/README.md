# fastrtps_config
- [confluence: DiscoveryServerV2](https://www.tool.sony.biz/confluence/pages/viewpage.action?pageId=3507048902)
- [confluence: Loaned Msg and True Zero Copy](https://www.tool.sony.biz/confluence/pages/viewpage.action?pageId=3507054503)

# 前提知識
- Discovery Server v2 : 全 Participant 間で1:1通信するのではなく、Discovery Server 経由で必要な通信のみ行うことで、通信量・計算負荷を軽減。詳細はコンフル参照。
- CLIENT : SERVER に Discovery 情報を送信し、Pub/Sub など自分のペアの Discovery 情報のみを受信。
- SUPER_CLIENT : SERVER に Discovery 情報を送信し、全 Discovery 情報を受信。

# 注意
- ros2 topic list などの不特定多数と通信する ROS2 CLI は、Discovery Server 使用時は SUPER_CLIENT に設定しないと正常に通信できません。各スクリプトで ROS2 CLI を SUPER_CLIENT に設定するものを仕込んでいますが注意してください。
- 例えば ros-tools/Relay で使用されている get_publishers_info_by_topic も同様に SUPER_CLIENT に設定しないと正常に通信できません。一部機能が制限されるため、通信が正常動作しない場合にはこの点も疑ってください。

# 各スクリプトの役割
## reset_simple_discovery.sh
- Default の Simple Discovery Server に戻すためのスクリプト。

## run_discovery_server_local.sh
- Discovery Server v2 を使用するときはこのスクリプトを実行して、SERVER を起動する。他の CLIENT もすべてこの SERVER に接続する。他のデバイスのサーバと直接通信できないので注意。

## run_discovery_server_remote.sh
- 開発PCと実機の通信など分散システムで使用する場合にこのスクリプトを実行して、他のデバイスのサーバとローカルサーバを接続するサーバを起動する。接続先サーバの GUID, ID, IP address, PORT を記載する。
- 直接接続されるサーバの GUID (ID) が重複すると正常に通信できないので注意。

## set_discovery_server_client.sh
- Discovery Server v2 に CLIENT として接続するための設定をするスクリプト。CLIENT に設定してから各ノードを起動する。

## set_discovery_server_super_client.sh
- ROS2 CLI など全ノードと通信する必要があるときのための設定スクリプト。CLIENT 設定で通信がうまくいかないときは、SUPER_CLIENT を試してください。

# Usage
## デフォルトサーバを起動
```
$ . run_discovery_server_local.sh

Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration.
The daemon has been stopped
The daemon has been started
Run Default Discovery Server.
### Server is running ###
  Participant Type:   SERVER
  Security:           NO
  Server ID:          0
  Server GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
  Server Addresses:   UDPv4:[127.0.0.1]:11811
```

## デフォルトサーバ・リモートサーバと通信するサーバを起動
```
$ . run_discovery_server_remote.sh

Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration.
The daemon has been stopped
The daemon has been started
Detected host IP address: 43.21.213.65
Using fixed RemoteServer port: 11811
Using Discovery Server port: 11812
Using participant GUID: 44.53.01.5f.45.50.52.4f.53.49.4d.41
Creating XML configuration file...
Please enter the number of additional remote servers (>= 0):
0
XML configuration file custom_discovery_server.xml has been created.
Running Default Discovery Server with custom XML configuration.
### Server is running ###
  Participant Type:   SERVER
  Security:           NO
  Server ID:          1
  Server GUID prefix: 44.53.01.5f.45.50.52.4f.53.49.4d.41
  Server Addresses:   UDPv4:[43.21.213.65]:11812
```

## デフォルトサーバと通信するクライアント設定で ROS2 ノードを起動
```
$ . ./fastrtps_config/set_discovery_client.sh

Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration.
The daemon has been stopped
The daemon has been started
Setting Fast-RTPS default to use Discovery CLIENT configuration.

$ ros2 run demo_node_cpp talker
```
```
$ . ./fastrtps_config/set_discovery_client.sh

Setting ROS2 CLI to use Discovery SUPER_CLIENT configuration.
The daemon has been stopped
The daemon has been started
Setting Fast-RTPS default to use Discovery CLIENT configuration.

$ ros2 run demo_node_cpp listener
```

## ROS2 CLI を使用
```
$ . ./fastrtps_config/set_discovery_super_client.sh

Setting Fast-RTPS default to use Discovery SUPER_CLIENT configuration, and restart daemon for CLI.
The daemon is not running
The daemon has been started

$ ros2 topic list
```
