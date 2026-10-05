# Автозапуск JT-Zero и статус VO на OSD

## Что делает изменение

1. При загрузке Raspberry Pi сервис `jtzero-flight.service` запускает штатный
   `startFlightRuntime.sh`. Он, в свою очередь, запускает Web UI, MAVLink router
   и flight runtime с `MONKEYS_RAW_UNIFIED_PUBLISH=1`.
2. Flight runtime отправляет MAVLink `STATUSTEXT`:
   - `JT VO READY` — после 1 секунды непрерывной успешной production-передачи
     optical flow в FC;
   - `JT VO LOST` — если поток успешных OF-пакетов пропал.
3. `JT VO READY` повторяется раз в 2 секунды, чтобы надпись оставалась видимой
   в OSD-панели MESSAGE.

Изменение не меняет WORKED5, focal scale, расчёт optical flow, temporal aggregation,
параметры EKF или управление полётом.

## Установка автозапуска

Из корня репозитория:

```bash
bash scripts/install_boot_autostart.sh
```

Скрипт только устанавливает и включает systemd unit. Уже работающий runtime он
не перезапускает. Сервис стартует автоматически после следующей перезагрузки.

Проверка:

```bash
systemctl is-enabled jtzero-flight.service
systemctl status jtzero-flight.service --no-pager
```

После перезагрузки:

```bash
systemctl status jtzero-flight.service --no-pager
journalctl -u jtzero-flight.service -b --no-pager -n 100
```

## Настройка OSD

На используемом экране ArduPilot должна быть включена панель MESSAGE.
Для первого экрана нужен параметр:

```text
OSD1_MESSAGE_EN = 1
```

Положение задают `OSD1_MESSAGE_X` и `OSD1_MESSAGE_Y`.

Сначала проверь отображение `JT VO READY` на земле при работающей камере,
TF-Luna и runtime. Только после этого используй индикацию как операторский
признак перед взлётом.

## Смысл READY

`READY` здесь означает не просто «камера открылась». Runtime должен успешно
передавать production optical-flow пакеты в FC; поток должен оставаться свежим
не менее одной секунды. Это операторская индикация и не является дополнительным
flight gate: она ничего не ARM-ит и не блокирует.
