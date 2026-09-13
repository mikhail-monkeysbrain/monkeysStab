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

## GUI настройки геометрии датчиков

Положение OV9281 и TF-Luna задаётся через отдельную **GUI-утилиту** (графическое окно):

```bash
cd ~/Desktop/monkeysStab
git pull --ff-only
bash scripts/geometry_gui.sh
```

Утилита сама подключается к FC (полётному контроллеру) по MAVLink. По умолчанию:

```text
FC:     /dev/ttyAMA0
baud:   460800
sysid:  1
compid: 1
```

В окне вводятся координаты **оптического центра** камеры и дальномера в миллиметрах относительно центра IMU полётного контроллера.

Используется система координат FRD:

```text
X +  вперёд
X -  назад
Y +  вправо
Y -  влево
Z +  вниз
Z -  вверх
```

Кнопка **«ПРОЧИТАТЬ ИЗ FC»** читает текущие:

```text
FLOW_POS_X
FLOW_POS_Y
FLOW_POS_Z
RNGFND1_POS_X
RNGFND1_POS_Y
RNGFND1_POS_Z
```

и показывает их в миллиметрах.

Кнопка **«ЗАПИСАТЬ В FC»**:

1. переводит введённые миллиметры в метры;
2. показывает подтверждение со всеми шестью координатами;
3. записывает `FLOW_POS_X/Y/Z` и `RNGFND1_POS_X/Y/Z` непосредственно в ArduPilot через MAVLink;
4. ждёт подтверждение `PARAM_VALUE` от FC для каждого параметра;
5. повторно читает все шесть параметров и проверяет, что FC действительно сохранил заданные значения;
6. после успешной проверки обновляет локальный `config/mount_geometry.json`;
7. обновляет положение камеры в `config/ov9281_current_mount.yaml`.

Таким образом FC и конфигурация `monkeysStab` остаются синхронизированы, и следующий `scripts/run.sh` проверяет именно те значения, которые были записаны через GUI.

**Запись геометрии выполнять при DISARMED (моторы выключены).** После изменения параметров перед следующим flight-тестом рекомендуется перезагрузить FC.

Утилита также показывает разнос TF-Luna относительно OV9281 по X/Y/Z.

## GUI настройки критических параметров FC

Критические параметры полётного контроллера настраиваются отдельной GUI-утилитой:

```bash
cd ~/Desktop/monkeysStab
git pull --ff-only
bash scripts/fc_setup_gui.sh
```

Утилита подключается к FC по MAVLink, читает текущие значения, показывает их и позволяет применить согласованный профиль monkeysStab.

Она управляет следующими группами параметров:

- **EKF3**: `AHRS_EKF_TYPE`, `EK3_ENABLE`, `EK3_SRC1_POSXY`, `EK3_SRC1_VELXY`, `EK3_SRC1_POSZ`, `EK3_SRC1_VELZ`, `EK3_SRC1_YAW`, `EK3_SRC_OPTIONS`;
- **Optical Flow**: `FLOW_TYPE`, `FLOW_OPTIONS`, `FLOW_ORIENT_YAW`, `FLOW_FXSCALER`, `FLOW_FYSCALER`, `EK3_FLOW_DELAY`, `EK3_FLOW_MAX`;
- **дальномер**: `RNGFND1_TYPE`, `RNGFND1_ORIENT`, `RNGFND1_MIN`, `RNGFND1_MAX`;
- **компас**: `COMPASS_ENABLE`, `COMPASS_USE`;
- геометрия `FLOW_POS_*` и `RNGFND1_POS_*` остаётся в отдельном Geometry GUI, который можно открыть кнопкой из этой утилиты.

Для текущей безподвесной системы профиль фиксирует:

```text
AHRS_EKF_TYPE = 3
EK3_ENABLE = 1

FLOW_TYPE = 5             # MAVLink
FLOW_OPTIONS = 0          # камера жёстко закреплена
FLOW_ORIENT_YAW = 0
FLOW_FXSCALER = 0
FLOW_FYSCALER = 0
EK3_FLOW_DELAY = 0

RNGFND1_TYPE = 10         # MAVLink DISTANCE_SENSOR
RNGFND1_ORIENT = 25       # вниз
RNGFND1_MIN = 0.10 m
RNGFND1_MAX = 8.0 m

EK3_SRC1_POSXY = 0        # None
EK3_SRC1_VELXY = 5        # OpticalFlow
EK3_SRC1_VELZ = 0         # None
EK3_SRC_OPTIONS = 0
```

В GUI отдельно выбираются:

- источник Z: **RangeFinder** или **Baro**;
- режим компаса/yaw: **компас используется для yaw**, **компас включён только для диагностики**, либо **компас полностью отключён**.

Если выбран компас как источник курса, утилита согласованно выставляет `COMPASS_ENABLE=1`, `COMPASS_USE=1`, `EK3_SRC1_YAW=1`. Если компас не используется для курса, `EK3_SRC1_YAW=0`.

**`EK3_SRC1_YAW=6` (ExternalNav yaw) в этой утилите намеренно недоступен.** Текущий Optical Flow не имеет независимого источника yaw (курса), поэтому возвращать yaw FC обратно как ExternalNav yaw нельзя.

### Компас: влияние и опциональность

Компас **не является обязательным датчиком для базовой работы текущего monkeysStab Optical Flow-контура**. OV9281 измеряет горизонтальное движение относительно поверхности, TF-Luna измеряет расстояние до поверхности, а IMU/гироскопы FC отслеживают быстрые изменения ориентации аппарата.

Поэтому для короткого относительного удержания точки возможна работа без магнитного компаса:

```text
COMPASS_ENABLE = 0
COMPASS_USE = 0
EK3_SRC1_YAW = 0
```

Что меняется без компаса: FC не получает независимую абсолютную привязку курса к магнитному северу. Гироскоп продолжает измерять поворот аппарата, но его небольшая ошибка может постепенно накапливаться. Поэтому оценка yaw (курса — направления, куда смотрит нос аппарата) со временем может дрейфовать.

Практическое следствие:

- для текущей отладки и коротких тестов удержания позиции компас **опционален**;
- для длительного полёта нужно отдельно проверить скорость дрейфа yaw без компаса;
- если дрейф окажется неприемлемым, компас можно включить как независимую абсолютную привязку курса;
- включение компаса не заменяет Optical Flow и не исправляет ошибки измерения X/Y камерой; оно в первую очередь влияет на оценку направления аппарата;
- `EK3_SRC1_YAW=6` (ExternalNav yaw — курс от внешней навигации) для текущего monkeysStab запрещён, потому что Optical Flow независимый yaw не измеряет.

GUI поэтому предоставляет три режима: компас используется для yaw; компас включён, но не участвует в yaw; компас полностью отключён. Выбор режима является частью конкретного FC-профиля и сохраняется в `config/fc_profile.json`.

После нажатия **«ПРИМЕНИТЬ ПРОФИЛЬ»** утилита:

1. показывает все ключевые изменения перед записью;
2. записывает параметры в FC через MAVLink;
3. ждёт подтверждение `PARAM_VALUE`;
4. повторно читает параметры и проверяет их;
5. сохраняет подтверждённый профиль в `config/fc_profile.json`.

`scripts/run.sh` перед каждым запуском сверяет FC с `config/fc_profile.json`. Если критический параметр отличается, запуск останавливается.

Запись выполнять при **DISARMED (моторы выключены)**. После изменения критических параметров FC нужно перезагрузить.

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

Ниже перечислены зависимости, необходимые для сборки и запуска текущего `monkeysStab`.

### Системные пакеты

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  g++ \
  pkg-config \
  libopencv-dev \
  python3 \
  python3-tk \
  git
```

Назначение:

- **build-essential** — базовый набор средств сборки C/C++, включая `make` и системные заголовки.
- **g++** — компилятор C++. Проект собирается в режиме C++17.
- **pkg-config** — сообщает скриптам сборки пути к заголовкам и библиотекам OpenCV.
- **libopencv-dev** — OpenCV 4. Используются Core, Image Processing, Video/Optical Flow, Calibration, Image Codecs и HighGUI.
- **python3** — используется read-only preflight-проверками и утилитой геометрии.
- **python3-tk** — Tkinter, графический интерфейс утилиты ввода положения камеры и дальномера.
- **git** — нужен для получения проекта и автоматической загрузки зафиксированной версии MAVLink headers.

### MAVLink C headers

Для обмена с ArduPilot нужны MAVLink C headers с dialect `ardupilotmega` (набором сообщений ArduPilot). Они не требуют старый Kimera-VIO.

Если подходящих системных headers нет, `scripts/smoke_build.sh` и `scripts/run.sh` автоматически вызывают `scripts/bootstrap_dependencies.sh`. Он загружает официальные MAVLink C headers в `third_party/mavlink/`.

Зафиксированная ревизия:

```text
mavlink/c_library_v2
commit 04fffaab116486ffdf7501c37a3f7393eb7beffc
```

Порядок поиска MAVLink:

```text
1. $MAVLINK_ROOT, если задан вручную
2. <репозиторий>/third_party/mavlink
3. /usr/local/include/mavlink/v2.0
4. /usr/include/mavlink/v2.0
5. автоматическая загрузка в third_party/mavlink
```

`/home/vio/Kimera-VIO/third_party/mavlink` больше не используется и не является зависимостью проекта.

### OpenCV

Текущий C++ код использует OpenCV 4:

```text
opencv_core
opencv_imgproc
opencv_video
opencv_calib3d
opencv_imgcodecs
opencv_highgui
```

Сборочные флаги и библиотеки берутся автоматически через `pkg-config --cflags opencv4` и `pkg-config --libs opencv4`. При наличии `libopencv-dev` отдельно устанавливать эти модули обычно не требуется.

### Linux API, входящие в систему

Дополнительные сторонние библиотеки для следующих функций не нужны: V4L2 (Video4Linux2 — стандартный Linux-интерфейс камеры) для OV9281; POSIX serial/termios (стандартный Linux-интерфейс последовательного порта) для TF-Luna и FC; `poll`, `mmap` и `pthread` для ожидания данных, отображения буферов камеры в память и фоновых потоков. Они предоставляются Linux/glibc и системными заголовками Raspberry Pi OS.

### Аппаратные интерфейсы, ожидаемые проектом

```text
OV9281 USB       -> V4L2, 640x480 MJPG
TF-Luna          -> /dev/ttyAMA2, 115200
MatekH743 / FC   -> /dev/ttyAMA0, 460800
```

По умолчанию OV9281 ищется по стабильному USB `by-id`, указанному выше. Пути можно переопределить переменными `MONKEYS_CAMERA`, `MONKEYS_LUNA` и `MONKEYS_FC`.

### Что НЕ является зависимостью

Для текущего `monkeysStab` не нужны Kimera-VIO, GTSAM, ROS/ROS 2, старый `jtzero-kimera`, stereo-код OV9281/OV5647, libcamera (production OV9281 работает через USB/V4L2), старые MOVE500/VIO диагностические программы и датасеты.

### Быстрая проверка окружения

```bash
cd ~/Desktop/monkeysStab
bash scripts/smoke_build.sh
```

Успешная проверка заканчивается строкой `BUILD PASS: /tmp/monkeysstab_optical_flow_buildcheck`. Она подтверждает наличие всех компиляционных зависимостей. Камера, TF-Luna и FC при `smoke_build` не открываются.

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

## Mission Planner через Wi-Fi

RPi и Windows PC могут работать в одной Wi-Fi/LAN сети без USB-соединения между PC и FC.

Архитектура:

```text
MatekH743
   ↕ /dev/ttyAMA0 @ 460800
Raspberry Pi 5 / MAVLink router
   ├── local TCP 127.0.0.1:5760  -> monkeysStab
   └── UDP 14550 over Wi-Fi      <-> Mission Planner
```

Критическое правило: физический `/dev/ttyAMA0` должен иметь **одного владельца**. Нельзя одновременно открывать UART из monkeysStab и отдельного telemetry-router: два процесса будут конкурировать за входящие MAVLink-байты.

Для первичной проверки Wi-Fi телеметрии, когда основной `scripts/run.sh` НЕ запущен:

```bash
cd ~/Desktop/monkeysStab
git pull --ff-only
bash scripts/run_mavlink_wifi.sh
```

Router открывает FC `/dev/ttyAMA0` на 460800, слушает UDP `14550` на всех сетевых интерфейсах и локальный TCP `127.0.0.1:5760`. Первый UDP-пакет от Mission Planner запоминает адрес PC; после этого обмен двусторонний.

Если IP Windows PC известен заранее, можно сразу указать его:

```bash
MONKEYS_GCS_IP=192.168.1.100 bash scripts/run_mavlink_wifi.sh
```

где `192.168.1.100` нужно заменить реальным IPv4-адресом PC в той же сети.

В Mission Planner для подключения используется UDP/UDPCl на порту `14550` в зависимости от выбранного способа соединения. Windows Firewall должен разрешать Mission Planner принимать UDP 14550.

Текущая утилита предназначена сначала для **изолированной проверки канала FC ↔ RPi ↔ Wi-Fi ↔ Mission Planner**. До перевода production `scripts/run.sh` на локальный TCP endpoint не запускайте router и основной monkeysStab одновременно.

После подтверждения телеметрии следующий архитектурный шаг — переключить monkeysStab с прямого `/dev/ttyAMA0` на локальный TCP `127.0.0.1:5760`. Тогда router станет единственным владельцем UART, а Mission Planner и monkeysStab смогут работать одновременно.

## Логи

Каждый запуск создаёт каталог:

```text
~/monkeysStab_runs/YYYYMMDD_HHMMSS_OPTICAL_FLOW/
```

с `optical_flow_mavlink.csv` и `build.log`.

## Важно

Этот репозиторий фиксирует рабочее состояние безподвесной системы на 13.09.2026. Будущий двухосевой гироподвес будет отдельным профилем/веткой, чтобы не ломать подтверждённую fixed-mount конфигурацию.
