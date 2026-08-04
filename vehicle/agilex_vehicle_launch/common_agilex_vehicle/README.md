# common_agilex_vehicle

AgileX ScoutをゲームパッドまたはAutowareから操作するためのROS 2パッケージです。
手動操作では`sensor_msgs/msg/Joy`、自動運転では`autoware_control_msgs/msg/Control`を
`geometry_msgs/msg/Twist`へ変換し、`scout_base_node`へ渡します。

## Autowareインターフェース

- subscribe: `/control/command/control_cmd` (`autoware_control_msgs/msg/Control`)
- publish: `/vehicle/status/control_mode` (`autoware_vehicle_msgs/msg/ControlModeReport`)
- デッドマンボタンを押している間はMANUAL、それ以外はAUTONOMOUSを通知します。
- Autowareのステアリングタイヤ角は`wheel_base`を用いてScoutの旋回角速度へ変換します。

## 安全機能

- デッドマンボタンを押している間だけ走行指令を出します。
- ボタンを離したとき、Joy入力が途絶えたとき、不正なJoy配列を受信したときは停止指令を出します。
- 起動時の速度上限は50%です。設定したボタンの立ち上がりごとに10%ずつ変更できます。

## 起動

CANインターフェースを準備した後、次を実行します。

```bash
ros2 launch common_agilex_vehicle manual_control.launch.xml can_port:=can0
```

`/dev/input/js1`を使用する場合は`joy_device_id:=1`も指定します。

既定の割り当ては、左スティック上下が前後、右スティック左右が旋回、Xボタンが
デッドマン、RB/LBが速度上限の増減です。実機のコントローラに合わせて
`config/manual_control.param.yaml`を変更してください。
