# NUCLEO-F446RE用 ローラー射出・M2006装填制御

このプロジェクトは、ギヤで機械的に同期した2台のセンサレスBLDCモーターで射出ローラーを回し、C610に接続した3台のM2006で装填機構を動かします。全モーターはCAN1（1 Mbit/s）を共有します。VESCは拡張CAN ID、C610は標準CAN IDを使うため、同じCANバスで共存できます。`kimura_roller`にあったM3508制御は移植していません。

射出側はAMT102-VのA相とB相をPC6/PC7からTIM8で直接読み取り、1つのPID制御器がVESC ID 105と112へ同符号の電流指令を送信します。

PC3に接続した外付けボタンを押している間、またはCAN2で標準ID 200の`data[0]=1`を受信している間、最初に共通電流をランプ状に増加させて2台を始動し、次に共通dutyをランプ状に増加させます。実測回転数が目標±300 RPM以内へ入り、その状態を100 ms維持すると、`ROLLER_FEEDFORWARD_CURRENT_A`で設定した固定フィードフォワード電流を加えてPID回転数制御へ切り替わります。PC3を離す、またはCAN2で`data[0]=0`を受信すると0 RPMを目標とする制動PIDへ切り替わり、停止後は両モーターへ0 Aを送信します。

## 初期設定

| 設定項目 | 設定値 |
|---|---:|
| VESC CAN ID | モーター1: 105、モーター2: 112 |
| C610 CAN ID | M2006: 4、5、6 |
| CAN1通信速度 | 1 Mbit/s |
| CAN2遠隔指令 | PB12=RX、PB13=TX、1 Mbit/s、標準ID 200 |
| CAN2遠隔指令タイムアウト | 最後のID 200受信から100 msで解放扱い |
| 射出/装填共通ボタン | PC3、内部プルアップ、押下時LOW |
| 装填操作入力 | PC3、PA1、PB10、PB1、PB0。すべて内部プルアップ、作動時LOW |
| スイッチのチャタリング対策 | 全操作・リミット入力を30 msデバウンス |
| エンコーダー入力 | PC6/TIM8_CH1=A相、PC7/TIM8_CH2=B相。現在はコード側でA/B入れ替え相当に符号反転 |
| エンコーダー計数 | 2048 PPR、X4=8192カウント/回転 |
| ローラー目標回転数 | 4500 RPM |
| 起動電流 | 300 msで0から3.0 A/台まで増加 |
| 電流制御からduty制御への切替条件 | 指令方向へ250 RPM以上 |
| 起動duty | 2台の平均入力電圧と目標RPMから自動計算し、1800 msで増加 |
| duty制御からPIDへの切替条件 | 目標回転数±300 RPM以内を100 ms維持 |
| PID切替時のフィードフォワード | `ROLLER_FEEDFORWARD_CURRENT_A`で固定設定。現在は3.5 A/台 |
| PID切替後の補正電流上限ランプ | 150 msで0 Aから許容範囲まで増加 |
| 起動タイムアウト | 起動開始から5000 ms |
| 回転用PID | Kp=0.003、Ki=0.0005、Kd=0.00001 |
| 各VESCへの電流上限 | ±30 A/台（フィードフォワード＋PID補正） |
| 制動用PID | Kp=0.0005、Ki=0、Kd=0 |
| 制動用PID電流上限 | ±2 A/台 |
| 制動タイムアウト | 1000 ms |
| RPM/PID更新周期 | 10 ms |
| VESC指令周期 | 10 ms |
| C610指令周期 | 10 ms |
| 装填M2006開始条件 | PC3またはCAN2共通入力の作動中にローラー回転数が一度4000 RPM以上へ到達 |
| M2006フィードバックタイムアウト | 起動後500 ms猶予、その後1000 msで警告 |
| M2006 CAN送信異常判定 | 100回連続失敗で警告 |

制御値を調整する場合は、`Core/Src/main.c`の先頭付近にある`ROLLER_...`定数を変更してください。逆回転させる場合は`ROLLER_TARGET_RPM`を負の値にします。起動電流と起動dutyの符号は目標RPMから自動的に決まります。

### 装填用M2006の設定

`kimura_roller`の装填部分を次の設定で移植しています。

| M2006 | C610 ID | 操作時の目標回転数 | 用途 |
|---|---:|---:|---|
| ID4 | 4 | PC3/CAN2/PA1で+300 RPM、PB10で-1500 RPM | 旧ブラシ付きモーター機構の置換 |
| ID5 | 5 | -3750 RPM | 装填モーター1 |
| ID6 | 6 | -10000 RPM | 装填モーター2 |

ID5とID6は、PC3またはCAN2共通入力の作動中に射出ローラーが一度4000 RPM以上へ到達してから、3:8の回転数比で同方向へ回ります。装填許可は共通入力がなくなるまで保持されます。同時にID4も正転設定速度で回ります。ID4は正転と逆転で別々に速度を調整できます。調整値は`Core/Src/main.c`の`M2006_...`定数にまとめています。

```c
#define M2006_FEED_2_TARGET_RPM       10000.0f
#define M2006_FEED_1_TARGET_RPM       (M2006_FEED_2_TARGET_RPM * 3.0f / 8.0f)
#define M2006_REPLACEMENT_FORWARD_TARGET_RPM    300.0f
#define M2006_REPLACEMENT_REVERSE_TARGET_RPM    1500.0f
#define M2006_AUTO_REVERSE_STOP_MS       3000U
#define M2006_LOAD_START_ROLLER_RPM   4000.0f
#define M2006_MAX_CURRENT_COMMAND      4000.0f
#define M2006_FEEDBACK_TIMEOUT_MS      1000U
#define M2006_CAN_TX_FAILURE_LIMIT      100U
```

`M2006_MAX_CURRENT_COMMAND`の4000はC610プロトコルの電流指令値で、4000 Aという意味ではありません。`M2006_ID5_CURRENT`や`M2006_ID6_CURRENT`が±4000付近に張り付いたままRPMが目標に届かない場合は、PIDは最大まで出力しているため、電流上限・機構負荷・回転方向・C610/モーター配線を確認してください。

動作開始直後はC610フィードバックを待つため500 msだけ猶予し、その後に各M2006の速度フィードバックが1000 ms以上途絶れた場合は`M2006=feedback-timeout`を出します。C610へのCAN送信は、瞬間的なメールボックス混雑では停止せず、100回連続で失敗した場合に`M2006=CAN-TX-fault`を出します。初期設定では、これらはM2006を即0指令に落とす安全停止ではなく、装填回転が途中で落ちる原因を避けるための警告です。強制停止へ戻したい場合は`M2006_FEEDBACK_FAULT_STOP_ENABLE`または`M2006_CAN_TX_FAULT_STOP_ENABLE`を1にします。

起動dutyは、VESC ID 105と112から受信した入力電圧の平均値に一次関数の補正を行い、次式で自動計算します。

```text
補正電圧[V] = 平均入力電圧[V] × ROLLER_BATTERY_VOLTAGE_GAIN
              + ROLLER_BATTERY_VOLTAGE_OFFSET_V

理論duty = 目標ローラーRPM / (560 × 補正電圧[V] × 13 / 25)
duty = 理論duty × 1.14
```

片方のSTATUS_5だけが新しい場合は受信できた側の電圧を使い、両方とも未受信の場合は公称電圧24.5 Vを使います。Kv式は無負荷理論値なので、3950 RPMで高止まりした実測結果から負荷補正係数1.14を掛けています。計算結果は安全のため±0.95に制限されます。したがって補正係数を一度校正した後は、通常`ROLLER_TARGET_RPM`だけを変更すれば、回転方向を含めた起動dutyが決まります。

### 電圧補正係数の求め方

バッテリー電圧が異なる2つの状態で、Teleplotの平均入力電圧を`x1`、`x2`、同時にテスターで測った実電圧を`y1`、`y2`として記録します。単位はすべてVです。

```text
GAIN = (y2 - y1) / (x2 - x1)
OFFSET = y1 - GAIN × x1
```

求めた値を`Core/Src/main.c`の次の定数へ設定します。

```c
#define ROLLER_BATTERY_VOLTAGE_GAIN        (0.83f / 0.85f)
#define ROLLER_BATTERY_VOLTAGE_OFFSET_V    0.92f
```

現在は、Teleplot平均23.8 V／テスター実測24.16 Vと、Teleplot平均24.65 V／テスター実測24.99 Vの2点で校正しています。設定値は`GAIN=(24.99-24.16)/(24.65-23.8)=0.83/0.85（約0.976471）`、`OFFSET=0.92 V`です。

例えば、Teleplotが20.0 Vのときテスターが20.4 V、Teleplotが25.0 Vのときテスターが25.2 Vなら、`GAIN=0.96`、`OFFSET=1.2 V`です。2点はできるだけ離れた電圧で測定し、モーター停止中など電圧が安定した状態で同時に記録してください。

## 配線

STM32のCAN端子はロジックレベル信号です。SN65HVD230などの外付けCANトランシーバーが必要です。

| NUCLEO-F446RE | 接続先 |
|---|---|
| PA12 / CAN1_TX | CANトランシーバーのTXD |
| PA11 / CAN1_RX | CANトランシーバーのRXD |
| PB13 / CAN2_TX | 遠隔指令用CANトランシーバーのTXD |
| PB12 / CAN2_RX | 遠隔指令用CANトランシーバーのRXD |
| GND | CANトランシーバー、VESC、C610のCAN GND |
| CANトランシーバーのCANH | VESC 2台とC610 3台のCANH |
| CANトランシーバーのCANL | VESC 2台とC610 3台のCANL |

CANバスの物理的な両端に120 Ωの終端抵抗を取り付けてください。

CAN2もマイコンへ直接CANH/CANLを接続せず、別のCANトランシーバーを介して接続してください。送信側は、標準ID 200（10進数、16進数では`0x0C8`）のデータフレームを送信します。DLCは1以上にして、操作ボタンを押している間は`data[0]=1`、離している間は`data[0]=0`を50 ms以下の周期で送信してください。受信側は`1`を受け取ると押下状態、`0`を受け取ると解放状態として扱います。通信が途絶えた場合も最後の受信から100 ms後に解放扱いへ戻ります。保持時間は`CAN2_REMOTE_TRIGGER_TIMEOUT_MS`で変更できます。PC3は従来どおり使用でき、`PC3押下 OR CAN2 data[0]=1`で共通動作が有効になります。

AMT102-Vは次のように直接接続します。

| AMT102-V | NUCLEO-F446RE |
|---|---|
| A相 | PC6 / TIM8_CH1 |
| B相 | PC7 / TIM8_CH2 |
| GND | GND |
| V+ | エンコーダーの仕様範囲内の電源 |

操作ボタンとリミットスイッチは次のピンへ接続します。各スイッチのもう片方の端子はGNDへ接続してください。すべて内部プルアップ入力で、LOWを押下・作動として扱います。各入力にはソフトウェアで30 msのチャタリング対策を入れています。

| NUCLEO-F446RE | Nucleo端子名 | 役割 |
|---|---|---|
| PC3 | - | 射出/装填共通。押している間VESC ID105/112とM2006 ID4/5/6を同時に制御 |
| PA1 | A1 | M2006 ID4を正転 |
| PB10 | D6 | M2006 ID4を逆転。初期値ではPA1正転より速い |
| PB1 | D7 | 逆転側リミット。自動逆転中に作動するとID4を停止 |
| PB0 | D8 | 正転側リミット。PC3/PA1正転中に作動するとID4を自動逆転 |

D7とD8が同時に作動した場合、異常な入力状態としてID4を停止します。PC3や各入力ピンへ5 Vを接続しないでください。青色のUSERボタン（PC13）は使用しません。

## 物理スイッチによる動作

| 操作 | 動作 |
|---|---|
| PC3を押す | VESC ID105/112で射出ローラーを回す。ローラーが一度4000 RPM以上になると、M2006 ID5/6を3:8の回転数比で回し、ID4も正転する |
| CAN2で標準ID 200、`data[0]=1`を受信 | PC3を押している場合と同じ動作 |
| CAN2で標準ID 200、`data[0]=0`を受信 | PC3を離した場合と同じ動作。通信途絶時も最終受信から100 ms後に解放扱い |
| PC3を離す | 射出ローラーに弱い制動をかけ、その後0 Aにする。ID5/6も停止する。ID4の自動逆転待ち/自動逆転中だけはPB1到達まで継続する |
| A1を押す | M2006 ID4だけを正転する |
| D6を押す | M2006 ID4だけを逆転する |
| PC3/CAN2/PA1正転中にPB0が作動 | ID4を`M2006_AUTO_REVERSE_STOP_MS`だけ停止し、その後PB10と同じ速度で自動逆転する。共通入力がなくなってもPB1が作動するまで待機/逆転を継続 |
| 自動逆転中にPB1が作動 | ID4を停止する |

A1、D6、共通入力が同時に入力された場合、通常時のID4の方向はA1、D6、共通入力の順で優先します。ただしPB0到達後の停止待ち/自動逆転中は、PB1が作動するまで自動動作を優先します。ID5/6はPC3またはCAN2共通入力の作動中にローラーが一度4000 RPM以上へ到達してから制御されます。

A相、B相、GNDの配線は短くし、VESCとモーターの相線から離してください。TIM8はハードウェアX4直交デコードを使用します。`g_roller_encoder.continuous_count`には16 bitタイマーの折り返しを補正した連続位置が入り、`g_roller_encoder.rpm`は10 msごとに更新されます。現在は`ROLLER_ENCODER_DIRECTION`を`-1`にして、コード側でA/Bを入れ替えたのと同じ符号にしています。

TeleplotでRPMの符号が意図した方向と逆になる場合は、`ROLLER_ENCODER_DIRECTION`を`1`に戻すか、物理配線のA相とB相を入れ替えてください。両方を同時に行うと二重反転になります。指令方向と逆向きに300 RPM以上検出した場合は、急加速を防ぐため`ENCODER=wrong-direction`を出して停止します。目標回転数を700 RPM以上超えた場合は、`ROLLER=overspeed`を出して停止します。

## VS Code Teleplotモニター

USART2は、TIM8で取得したローラーRPM、共通電流指令、起動dutyをTeleplot形式で100 msごとに送信します。NUCLEO-F446REのST-LINK USB端子をPCへ接続し、VS CodeでTeleplotを開いてST-LINKのCOMポートを選択してください。

VESCの標準`STATUS 1`から、各VESCが内部電流センサで測定したモーター電流も取得します。これはバッテリー入力電流ではなく、VESCが制御に使うモーター電流です。シリアル送信は送信待ちバッファを使うため、Teleplot出力中も制御ループを文字送信完了待ちで停止させません。

| 項目 | 設定値 |
|---|---:|
| ボーレート | 115200 |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |
| フロー制御 | なし |

通常は次の系列が表示されます。

```text
>ROLLER_RPM:4490
>ROLLER_CURRENT_mA:3500
>ROLLER_PID_CURRENT_LIMIT_mA:16000
>ROLLER_FEEDFORWARD_CURRENT_mA:3500
>ROLLER_FEEDFORWARD_LIMIT_mA:3500
>ROLLER_PID_CORRECTION_CURRENT_mA:0
>ROLLER_DUTY_x10000:7189
>VESC_105_MOTOR_CURRENT_mA:1400
>VESC_112_MOTOR_CURRENT_mA:1200
>VESC_AVERAGE_MOTOR_CURRENT_mA:1300
>VESC_105_INPUT_VOLTAGE_mV:24650
>VESC_112_INPUT_VOLTAGE_mV:24650
>VESC_AVERAGE_INPUT_VOLTAGE_mV:24650
>VESC_CORRECTED_INPUT_VOLTAGE_mV:24990
>ROLLER_TARGET_DUTY_x10000:7189
>M2006_ID4_RPM:300
>M2006_ID5_RPM:-3750
>M2006_ID6_RPM:-10000
>M2006_ID4_CURRENT:420
>M2006_ID5_CURRENT:-800
>M2006_ID6_CURRENT:-1250
>M2006_LOAD_BUTTON:1
>CAN2_ID200_BUTTON:1
```

- `ROLLER_RPM`はAMT102-Vから計算したローラー回転数です。符号はA相/B相の配線順序と`ROLLER_ENCODER_DIRECTION`で決まります。
- `ROLLER_CURRENT_mA`は実測電流や2台の合計ではなく、VESC ID 105と112のそれぞれへ送る共通電流指令です。PID中は「フィードフォワード＋PID補正」です。
- `ROLLER_FEEDFORWARD_CURRENT_mA`は、PID切替時に足している固定フィードフォワード電流です。値は`Core/Src/main.c`の`ROLLER_FEEDFORWARD_CURRENT_A`で調整します。
- `ROLLER_FEEDFORWARD_LIMIT_mA`は、現在設定されている固定フィードフォワード電流の大きさです。
- `ROLLER_PID_CORRECTION_CURRENT_mA`は速度誤差に対する通常PIDの補正分です。オーバーシュートする場合は、まず`ROLLER_SPEED_KI`を下げて調整してください。
- `ROLLER_PID_CURRENT_LIMIT_mA`はPID補正分の上限です。各VESCへの共通指令が±30 A/台を超えないよう、フィードフォワード電流分だけ狭められます。
- 起動の第1段階では、`ROLLER_CURRENT_mA`に共通起動電流ランプが表示されます。例えば`2000`は、各VESCへ2.0 Aを指令していることを表します。
- `ROLLER_DUTY_x10000`は、起動の第2段階で実際に送信する共通duty指令です。PID制御へ切り替わると0に戻ります。
- `VESC_105_MOTOR_CURRENT_mA`と`VESC_112_MOTOR_CURRENT_mA`は、各VESCからCANで受信した実測モーター電流です。duty制御中も表示されます。
- `VESC_AVERAGE_MOTOR_CURRENT_mA`は、両方のSTATUS 1が250 ms以内に届いている場合の2台平均です。
- `VESC_AVERAGE_INPUT_VOLTAGE_mV`は2台の補正前平均入力電圧です。
- `VESC_CORRECTED_INPUT_VOLTAGE_mV`は一次関数で補正した、duty計算に実際に使う電圧です。
- `ROLLER_TARGET_DUTY_x10000`は補正電圧、目標RPM、負荷補正係数1.14から計算した到達先dutyです。例えば`7189`は約duty=0.719です。
- `M2006_ID4_RPM`、`M2006_ID5_RPM`、`M2006_ID6_RPM`はC610から受信した各M2006の実測回転数です。
- `M2006_ID4_CURRENT`、`M2006_ID5_CURRENT`、`M2006_ID6_CURRENT`は各C610へ送っている電流指令値です。実測電流やA単位ではありません。
- `M2006_LOAD_BUTTON`は、PC3またはCAN2共通入力の作動中にローラーが一度4000 RPM以上へ到達し、装填が許可された状態を表します。
- `CAN2_ID200_BUTTON`は、CAN2の標準ID 200で最後に受信した有効値が`data[0]=1`なら1、`data[0]=0`または100 ms以上未受信なら0です。

5000 ms以内に実測回転数が目標±300 RPM以内で安定しなかった場合、`STARTUP=timeout`と表示されます。このとき両VESCへ0 Aを送り、PC3が離され、CAN2共通入力も`data[0]=0`またはタイムアウトで解放状態になるまで再始動を禁止します。

CAN送信メールボックスが一時的に混雑した場合は2 ms以内で再試行し、それでも失敗した場合は次の10 ms制御周期で再送します。3制御周期連続で失敗した場合だけ安全停止し、次のような診断を出力します。

```text
CAN_TX_FAULT:id=112,reason=mailbox-timeout,hal=0x00000000,esr=0x00000000
```

`id`は最後に送信できなかったVESC IDです。`reason=mailbox-timeout`は送信メールボックスが空かなかった場合、`reason=HAL-error`はHALの送信登録が失敗した場合です。`hal`と`esr`はCAN配線、通信速度、ACK、Bus-Offなどを調べるためのエラー値です。異常停止後はPC3を離し、CAN2へ`data[0]=0`を送ってから再始動してください。

M2006側では、動作開始から500 ms経過後に必要な速度フィードバックが1000 ms届かないと`M2006=feedback-timeout`、標準IDの電流指令を100回連続で送信できないと`M2006=CAN-TX-fault`を出力します。初期設定では警告のみで、M2006への指令は継続します。強制停止させたい場合は、`Core/Src/main.c`の`M2006_FEEDBACK_FAULT_STOP_ENABLE`または`M2006_CAN_TX_FAULT_STOP_ENABLE`を1にしてください。

`ROLLER_ENCODER_PPR`は現在2048 PPRです。AMT102-VのDIPスイッチ設定に合わせて変更してください。TIM8はX4デコードなので、1回転あたりのカウント数は設定PPRの4倍です。

## C610/M2006の設定

- 3台のC610をそれぞれCAN ID 4、5、6に設定します。
- C610/M2006のCAN通信速度は1 Mbit/sです。VESC 2台とF446REも必ず1 Mbit/sにそろえてください。
- ID4/5/6のフィードバックは標準ID 0x204/0x205/0x206、電流指令は標準ID 0x200/0x1FFを使用します。
- ギヤやベルトを接続する前に、低い目標回転数で各モーターの回転方向とD7/D8の停止方向を確認してください。逆の場合は配線を無理に変更せず、`M2006_FEED_TARGET_SIGN`またはID4の方向判定を調整してください。

## 初回運転前に必要なVESC設定

このファームウェアの動作中にVESC Toolを接続する必要はありませんが、最初にモーターとバッテリーに合わせてVESCを設定してください。

- Motor Detectionを実行し、センサレスFOCを設定します。
- 1台目のVESC CAN IDを105、2台目を112に設定します。両方のCAN通信速度を1 Mbit/sにします。
- 同じ符号の指令で、2台が機構上必要な方向へ回転するように設定します。ギヤをかみ合わせる前に、必ず1台ずつ回転方向を確認してください。
- 両方のVESCでCANステータス定期送信を有効にします。VESC ToolのApp Settings内にあるCAN Status Message Modeで、少なくとも`STATUS 1`と`STATUS 5`を含むモードを選びます。CAN Status Rateは20～50 Hzを初期値にしてください。`STATUS 1`が実測モーター電流、`STATUS 5`が入力電圧に必要です。項目名はVESC Tool／ファームウェアの版によって多少異なります。
- モーター電流、バッテリー電流、ERPM、電圧、温度の安全上限を設定します。
- STM32やCANケーブルに異常が起きた場合に停止するよう、VESCのCAN/control timeoutを短く設定します。100 ms程度と安全なtimeout brake currentを初期値として実機で検証してください。

最初の試験はローラーを無負荷に近い状態にし、予期しない動きに備えて機構を固定して行ってください。ソフトウェア停止は非常停止の代わりにはなりません。モーター電源を物理的に遮断できる非常停止手段を用意してください。

また、直接接続したインクリメンタルエンコーダーは断線を自己検出できません。エンコーダー信号が失われるとRPMが0として観測され、制御器が大きな電流を指令する可能性があります。配線を確実に固定し、実機運用前に別途ハードウェア非常停止を確認してください。

## ビルド方法

CMakeの初回構成時に、omuraisu-libraryをGitHubから取得します。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

生成物は次の場所に出力されます。

- `build/Debug/vesc_current_control.elf`
- `build/Debug/vesc_current_control.hex`
- `build/Debug/vesc_current_control.bin`

## LED表示

| LED状態 | 意味 |
|---|---|
| 点灯 | 起動電流制御、起動duty制御、またはPID回転中 |
| 150 ms周期で点滅 | 制動中 |
| 消灯 | 停止中 |
| 100 ms周期で点滅 | CAN送信、M2006フィードバック、エンコーダー、または起動タイムアウト異常。再試行前に操作ボタンを離す |

VESC指令は必ずCAN拡張IDで送信する必要があります。おむらいすライブラリを使って指令のエンコード、上限制限、ステータス解析を行い、数値上のCAN IDが`0x800`未満の場合でも拡張フレームになるよう、STM32 HALで明示的に送信しています。
