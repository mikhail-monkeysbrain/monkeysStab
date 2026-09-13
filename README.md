# monkeysStab

Минимальный рабочий репозиторий JT-Zero Optical Flow для Raspberry Pi 5 + OV9281 + TF-Luna + ArduPilot.

## Что это

Текущий рабочий контур:

```text
OV9281 USB -> Optical Flow -> MAVLink OPTICAL_FLOW -> ArduPilot EKF3
TF-Luna    -> DISTANCE_SENSOR ---------------------> ArduPilot
```

Проект отделён от старого jtzero-kimera/Kimera-VIO контура. Старый `ground_motion_live_v2_v3_ab.cpp` больше не является исходной зависимостью: захват OV9281, TF-Luna и загрузка калибровки вынесены в `src/runtime.hpp`. Исторические Kimera/VIO/stereo датасеты в этот репозиторий не перенесены. В production source пока сохранены актуальные flight-диагностики и GUI, которые нужны для текущей проверки поведения системы.

## Текущее оборудование

- Raspberry Pi 5
- OV9281 USB, 640x480 MJPG
- TF-Luna: `/dev/ttyAMA2`
- MatekH743 / ArduCopter 4.7.1: `/dev/ttyAMA0`, 460800
- камера по стабильному by-id:
  `/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0`

## Текущая геометрия временного монтажа

В координатах ArduPilot FRD (вперёд / вправо / вниз), относительно FC IMU:

```text
OV9281:  X=+0.0625 m  Y=0  Z=+0.0500 m
TF-Luna: X=+0.0855 m  Y=0  Z=+0.0550 m
```

TF-Luna находится на 23 мм впереди OV9281 и примерно на 5 мм ниже её оптического центра.

## Проверенная конфигурация EKF3

```text
FLOW_TYPE=5
FLOW_OPTIONS=0
FLOW_ORIENT_YAW=0
FLOW_FXSCALER=0
FLOW_FYSCALER=0
EK3_FLOW_DELAY=0
EK3_SRC1_POSXY=0
EK3_SRC1_VELXY=5
EK3_SRC1_POSZ=2
EK3_SRC1_VELZ=0
EK3_SRC1_YAW=0
```

`EK3_SRC1_VELXY=5` означает OpticalFlow (горизонтальная скорость берётся из оптического потока).

`EK3_SRC1_POSZ=2` означает RangeFinder (вертикальная позиция в текущем стендовом профиле использует дальномер). На резких перепадах поверхности это значение нужно интерпретировать осторожно: TF-Luna измеряет расстояние до текущей поверхности, а не абсолютную высоту аппарата в помещении.

`EK3_SRC1_YAW=0` оставлять без ExternalNav yaw: Optical Flow не имеет независимого источника курса.

## Текущий профиль камеры

- `focal_scale = 0.931`
- feature ROI (рабочая область поиска точек) = `0.20 0.32 0.80 0.90`
- max features = `500`
- разрешение = `640x480`

Параметры камеры находятся в `config/ov9281_current_mount.yaml`.

## Зависимости

Нужны:

- Linux / Raspberry Pi OS
- `g++` с C++17
- OpenCV 4
- MAVLink C headers с dialect `ardupilotmega`
- Python 3 (только для preflight-проверок)

Пример пакетов:

```bash
sudo apt update
sudo apt install -y build-essential pkg-config libopencv-dev python3
```

MAVLink headers ищутся в `/usr/local/include/mavlink/v2.0` и `/usr/include/mavlink/v2.0`. Другой путь можно указать переменной `MAVLINK_ROOT`. `~/Kimera-VIO` больше не используется как неявная зависимость.

## Проверка сборки

После `git pull` сначала можно проверить только компиляцию, не открывая устройства и не отправляя MAVLink:

```bash
cd ~/monkeysStab
bash scripts/smoke_build.sh
```

Ожидаемый результат:

```text
BUILD PASS: /tmp/monkeysstab_optical_flow_buildcheck
```

## Запуск

```bash
cd ~/monkeysStab
bash scripts/run.sh
```

Launcher (скрипт запуска):

1. только читает геометрию и ключевые параметры FC;
2. останавливается при несовместимой конфигурации;
3. собирает рабочий C++ binary;
4. запускает GUI;
5. не ARM-ит FC и не переключает flight mode.

SPACE в GUI — принять текущую оценку FC за новую точку 0.

Q / ESC — завершить программу.

## Логи

Каждый запуск создаёт каталог:

```text
~/monkeysStab_runs/YYYYMMDD_HHMMSS_OPTICAL_FLOW/
```

с `optical_flow_mavlink.csv` и `build.log`.

## Важно

Этот репозиторий фиксирует рабочее состояние безподвесной системы на 13.09.2026. Будущий двухосевой гироподвес будет отдельным профилем/веткой, чтобы не ломать подтверждённую fixed-mount конфигурацию.
