# NUCLEO-F446RE用 VESCローラー回転数制御

このプロジェクトは、ギヤで機械的に同期した2台のセンサレスBLDCモーターで、1本のローラーを回転させます。AMT102-VのA相とB相をPC6/PC7からTIM8で直接読み取り、1つのPID制御器がCAN1を通じてVESC ID 105と112へ同符号の電流指令を送信します。

PC3に接続した外付けボタンを押している間、最初に共通電流をランプ状に増加させて2台を始動し、次に共通dutyをランプ状に増加させます。実測回転数が目標付近で安定するとPID回転数制御へ切り替わります。ボタンを離すと、0 RPMを目標とする制動PIDへ切り替わり、停止後は両モーターへ0 Aを送信します。

## 初期設定

| 設定項目 | 設定値 |
|---|---:|
| VESC CAN ID | モーター1: 105、モーター2: 112 |
| CAN1通信速度 | 500 kbit/s |
| ローラー操作ボタン | PC3、内部プルアップ、押下時LOW |
| エンコーダー入力 | PC6/TIM8_CH1=A相、PC7/TIM8_CH2=B相 |
| エンコーダー計数 | 2048 PPR、X4=8192カウント/回転 |
| ローラー目標回転数 | -4500 RPM |
| 起動電流 | 150 msで0から-2.0 A/台まで増加 |
| 電流制御からduty制御への切替条件 | 指令方向へ-300 RPM以上 |
| 起動duty | 2台の平均入力電圧と目標RPMから自動計算し、1000 msで増加 |
| duty制御からPIDへの切替条件 | 目標回転数±250 RPM以内を30 ms維持 |
| PID切替後の電流上限ランプ | 300 msで0 Aから±10 Aへ増加 |
| 起動タイムアウト | 起動開始から3000 ms |
| 回転用PID | Kp=0.015、Ki=0.00020、Kd=0.00002 |
| 回転用PID電流上限 | ±10 A/台 |
| 制動用PID | Kp=0.002、Ki=0、Kd=0 |
| 制動用PID電流上限 | ±5 A/台 |
| 制動タイムアウト | 1000 ms |
| RPM/PID更新周期 | 10 ms |
| VESC指令周期 | 10 ms |

制御値を調整する場合は、`Core/Src/main.c`の先頭付近にある`ROLLER_...`定数を変更してください。逆回転させる場合は`ROLLER_TARGET_RPM`を負の値にします。起動電流と起動dutyの符号は目標RPMから自動的に決まります。

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
| GND | CANトランシーバーおよびVESCのCAN GND |
| CANトランシーバーのCANH | VESCのCANH |
| CANトランシーバーのCANL | VESCのCANL |

CANバスの物理的な両端に120 Ωの終端抵抗を取り付けてください。

AMT102-Vは次のように直接接続します。

| AMT102-V | NUCLEO-F446RE |
|---|---|
| A相 | PC6 / TIM8_CH1 |
| B相 | PC7 / TIM8_CH2 |
| GND | GND |
| V+ | エンコーダーの仕様範囲内の電源 |

ローラー操作ボタンは、片方の端子をPC3、もう片方をGNDへ接続します。PC3は内部プルアップを有効にしているため外付けプルアップ抵抗は不要です。PC3へ5 Vを接続しないでください。青色のUSERボタン（PC13）はローラー操作には使用しません。

A相、B相、GNDの配線は短くし、VESCとモーターの相線から離してください。TIM8はハードウェアX4直交デコードを使用します。`g_roller_encoder.continuous_count`には16 bitタイマーの折り返しを補正した連続位置が入り、`g_roller_encoder.rpm`は10 msごとに更新されます。

TeleplotでRPMの符号が意図した方向と逆になる場合は、A相とB相を入れ替えてください。

## VS Code Teleplotモニター

USART2は、TIM8で取得したローラーRPM、共通電流指令、起動dutyをTeleplot形式で100 msごとに送信します。NUCLEO-F446REのST-LINK USB端子をPCへ接続し、VS CodeでTeleplotを開いてST-LINKのCOMポートを選択してください。

| 項目 | 設定値 |
|---|---:|
| ボーレート | 115200 |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |
| フロー制御 | なし |

通常は次の系列が表示されます。

```text
>ROLLER_RPM:-1987
>ROLLER_CURRENT_mA:-1250
>ROLLER_PID_CURRENT_LIMIT_mA:10000
>ROLLER_DUTY_x10000:-7049
>VESC_105_INPUT_VOLTAGE_mV:24650
>VESC_112_INPUT_VOLTAGE_mV:24650
>VESC_AVERAGE_INPUT_VOLTAGE_mV:24650
>VESC_CORRECTED_INPUT_VOLTAGE_mV:24990
>ROLLER_TARGET_DUTY_x10000:-7049
```

- `ROLLER_RPM`はAMT102-Vから計算したローラー回転数です。符号はA相とB相の配線順序で決まります。
- `ROLLER_CURRENT_mA`は実測電流ではなく、VESC ID 105と112へ共通で送る電流指令です。例えば`-1250`は、各VESCへ-1.25 Aを指令していることを表します。
- `ROLLER_PID_CURRENT_LIMIT_mA`はPID切替後の電流上限です。切替直後の0から300 msで10000まで増加します。
- 起動の第1段階では、`ROLLER_CURRENT_mA`に共通起動電流ランプが表示されます。例えば`-2000`は、各VESCへ-2.0 Aを指令していることを表します。
- `ROLLER_DUTY_x10000`は、起動の第2段階で実際に送信する共通duty指令です。PID制御へ切り替わると0に戻ります。
- `VESC_AVERAGE_INPUT_VOLTAGE_mV`は2台の補正前平均入力電圧です。
- `VESC_CORRECTED_INPUT_VOLTAGE_mV`は一次関数で補正した、duty計算に実際に使う電圧です。
- `ROLLER_TARGET_DUTY_x10000`は補正電圧、目標RPM、負荷補正係数1.14から計算した到達先dutyです。例えば`-7049`は約duty=-0.705です。

3000 ms以内に実測回転数が目標±250 RPM以内で安定しなかった場合、`STARTUP=timeout`と表示されます。このとき両VESCへ0 Aを送り、PC3ボタンを一度離すまで再始動を禁止します。

CAN送信メールボックスが一時的に混雑した場合は2 ms以内で再試行し、それでも失敗した場合は次の10 ms制御周期で再送します。3制御周期連続で失敗した場合だけ安全停止し、次のような診断を出力します。

```text
CAN_TX_FAULT:id=112,reason=mailbox-timeout,hal=0x00000000,esr=0x00000000
```

`id`は最後に送信できなかったVESC IDです。`reason=mailbox-timeout`は送信メールボックスが空かなかった場合、`reason=HAL-error`はHALの送信登録が失敗した場合です。`hal`と`esr`はCAN配線、通信速度、ACK、Bus-Offなどを調べるためのエラー値です。異常停止後はPC3ボタンを一度離してから再始動してください。

`ROLLER_ENCODER_PPR`は現在2048 PPRです。AMT102-VのDIPスイッチ設定に合わせて変更してください。TIM8はX4デコードなので、1回転あたりのカウント数は設定PPRの4倍です。

## 初回運転前に必要なVESC設定

このファームウェアの動作中にVESC Toolを接続する必要はありませんが、最初にモーターとバッテリーに合わせてVESCを設定してください。

- Motor Detectionを実行し、センサレスFOCを設定します。
- 1台目のVESC CAN IDを105、2台目を112に設定します。両方のCAN通信速度を500 kbit/sにします。
- 同じ符号の指令で、2台が機構上必要な方向へ回転するように設定します。ギヤをかみ合わせる前に、必ず1台ずつ回転方向を確認してください。
- 低回転時に制動を終了できるよう、VESCのCANステータス定期送信を有効にします。
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
| 100 ms周期で点滅 | CAN送信、エンコーダー、または起動タイムアウト異常。再試行前にボタンを離す |

VESC指令は必ずCAN拡張IDで送信する必要があります。おむらいすライブラリを使って指令のエンコード、上限制限、ステータス解析を行い、数値上のCAN IDが`0x800`未満の場合でも拡張フレームになるよう、STM32 HALで明示的に送信しています。
