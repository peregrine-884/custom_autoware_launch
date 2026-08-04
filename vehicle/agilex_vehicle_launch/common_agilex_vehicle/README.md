# common_agilex_vehicle

AgileX ScoutとAutowareの間で指令・車両状態を変換するROS 2 vehicle interfaceです。
`scout_base_node`との通信には`geometry_msgs/msg/Twist`と`scout_msgs/msg/ScoutStatus`を
使用します。ゲームパッドによる手動オーバーライドにも対応します。

## Nodes

- `agilex_joy_controller`: Autoware/Joyの走行指令と制御モードを管理し、Scout向け
  `/cmd_vel`を生成します。
- `agilex_vehicle_interface`: Gear/Hazard指令と`/scout_status`を扱い、`/light_control`、
  `VelocityReport`、`SteeringReport`、`GearReport`、`HazardLightsReport`を生成します。

## Autoware inputs

| Topic | Type | Behavior |
|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | 速度と操舵角をScoutの速度・旋回速度へ変換 |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | 仮想ギア状態を設定 |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | 前後ライトの同時点滅を設定 |

## Autoware outputs

| Topic | Source |
|---|---|
| `/vehicle/status/control_mode` | 制御モード要求と手動オーバーライド状態 |
| `/vehicle/status/gear_status` | GearCommandに対応する仮想ギア状態 |
| `/vehicle/status/hazard_lights_status` | Scoutの前後ライト実状態 |
| `/vehicle/status/velocity_status` | Scoutの実測並進速度・旋回角速度 |
| `/vehicle/status/steering_status` | 実測速度から換算した仮想操舵角 |

Scoutには自動車の物理ギアがないため、GearReportはAutoware内部で使用する論理状態です。
Hazard LightsのENABLEは前後ライトの`LIGHT_BREATH`へ変換します。DISABLE時は点滅開始前の
ライト状態へ戻します。HazardLightsReportは指令値ではなく`/scout_status`の前後ライトが
両方とも点滅中かどうかから生成します。Turn indicators、Actuation status、door statusは
対応する実機情報がないため処理しません。

## Safety behavior

- 起動時はMANUALかつPARKです。
- AUTONOMOUSへの切り替えにはcontrol mode serviceの成功応答が必要です。
- Joyのデッドマンボタンを押している間は手動操作が最優先です。
- JoyまたはAutoware指令が設定時間以上途絶えると停止します。
- 手動解除または制御モード変更時には一度停止し、新しい指令を待ちます。

## Launch

CANインターフェースを準備した後、次を実行します。

```bash
ros2 launch common_agilex_vehicle vehicle_interface.launch.xml can_port:=can0
```

`/dev/input/js1`を使用する場合は`joy_device_id:=1`も指定します。パラメータは
`config/vehicle_interface.param.yaml`で変更できます。
