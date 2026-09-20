# JT-Zero / monkeysStab — состояние проекта

> Единый источник текущего технического контекста. Обновлять после эксперимента, который меняет выводы.
> Хронологический журнал не заменяет этот файл: здесь хранится именно текущее состояние знаний.

Обновлено: 2026-09-20  
Рабочая ветка исследований: `test/worked5-deltar-rotation-shadow`

## 1. Текущая цель

Сохранить доказанную работоспособность WORKED5 и устранить ложное XY при вращении аппарата, прежде всего при yaw на месте. Текущий исследовательский путь — ΔR/HIGHRES shadow. Production/FROZEN-контур не менять без отдельного независимого основания.

## 2. Жёсткая граница: frozen control

WORKED5 — контрольная рабочая система, а не объект текущей подгонки.

Зафиксированные результаты blind/контрольных метрических тестов:
- blind серия: ошибки примерно -0.859%, +3.897%, -1.410%; max |error| ≈ 3.897%;
- FAST blind N=5 для realtime FUSED-V2: -4.82%, -1.34%, -4.95%, -3.15%, -2.23%; mean ≈ -3.30%, MAE ≈ 13.84 мм, RMSE ≈ 15.09 мм;
- контрольный прогон 20260918_173832_OPTICAL_FLOW: физически ≈541 мм, WORKED5 ≈543.15 мм, ошибка ≈+0.415%;
- известны и плохие прогоны (например BAD478 ≈ -10.4%): WORKED5 не объявляется идеальным на любом произвольном прогоне.

Исторические контрольные refs:
- `worked/5pct-accuracy`, commit `94942be...`;
- frozen-control: `test/worked5-frozen-control`, commit `ed43ecce...`.

Правило: если новый shadow не совпадает с WORKED5, сначала исследовать shadow/условия эксперимента. Не менять focal, geometry, scale, thresholds WORKED5 ради улучшения одного shadow-прогона.

## 3. Аппаратная конфигурация

- Raspberry Pi 5.
- FC: MatekH743, ArduCopter 4.7.0.
- Камера: Arducam OV9281 USB, обычно 640x480 MJPG, около 93–101 fps.
- TF-Luna: напрямую к RPi через `/dev/ttyAMA2`.
- Компас используется.
- Основной контур: OV9281 → OPTICAL_FLOW; TF-Luna → масштаб OF + RangeFinder; compass → yaw; ArduPilot EKF3 → итоговый XY/Z.

## 4. Физическая геометрия

Источник: `config/mount_geometry.json`. Reference = FC_IMU, frame = FRD.

- OV9281: X=+0.0625 м, Y=0, Z=+0.050 м.
- TF-Luna: X=+0.0855 м, Y=0, Z=+0.055 м.

Геометрия была физически измерена рулеткой и после этого не менялась. Боковое смещение камеры визуально отсутствует. Эти значения не являются свободными fitting parameters.

Mount камеры из `config/ov9281_current_mount.yaml`:
- image top → vehicle forward;
- image right → vehicle right;
- optical axis → down;
- OpenCV Cx → +Y_FRD, Cy → -X_FRD, Cz → +Z_FRD.

Кодовая цепочка YAML → B_R_C → FLU→FRD внутренне согласована. Это НЕ доказывает отсутствие малого физического angular mounting error.

## 5. Что доказано про рабочий OF/EKF контур

Optical flow и RangeFinder реально fusion'ятся EKF3. После исправления covariance были наблюдаемы близкие measured/EKF позиции (около 0.136/0.135 м в соответствующем тесте).

Bench closure ранее:
- pitch ~27°: EKF closure ≈34 мм;
- roll ~26°: ≈13 мм;
- Z-only ~430 мм: XY closure ≈23 мм;
- combined motion: ≈47.6 мм.

Текущая задача — не создание optical flow с нуля, а улучшение поведения при вращении без разрушения рабочего baseline.

## 6. Текущая проблема yaw

При yaw на месте камера наблюдает значительное apparent translation. После rigid-body lever compensation остаётся ложный XY центра IMU.

Последний разобранный HIGHRES-valid yaw, 286 одинаковых интервалов:
- camera: N=-57.08 мм, E=+63.53 мм, |XY|=85.41 мм;
- lever: N=-52.05 мм, E=+39.77 мм, |XY|=65.51 мм;
- IMU = camera - lever: N=-5.03 мм, E=+23.76 мм, |XY|=24.29 мм.

Тождество camera - lever = IMU выполняется с численной точностью. Значит арифметика lever subtraction работает; остаётся несогласованность наблюдаемого camera motion и предсказанного rigid-body camera motion.

На тех же интервалах ранее получалось примерно:
- ATTITUDE residual XY ≈13.46 мм;
- ordinary gyro ΔR ≈22.24 мм;
- HIGHRES ΔR ≈24.29 мм.

Нельзя из одного этого делать вывод, что ATTITUDE физически точнее HIGHRES: возможна компенсация задержек/фильтрации другими систематическими эффектами.

## 7. ΔR / HIGHRES: доказанные результаты

Все ATTITUDE / ordinary gyro ΔR / HIGHRES ΔR варианты используют одинаковые:
- feature correspondences;
- K/D;
- range/height;
- extrinsics;
- camera/range positions;
- ground-plane metric model.

Различается источник межкадрового вращения.

`integrateBodyRates()`:
- интерполирует rate на t0/t1;
- включает внутренние samples;
- trapezoidal integration;
- `dR = dR * expSO3(w*dt)`.
Явной ошибки границ интервала или порядка умножения при аудите не найдено.

## 8. HIGHRES clock mapping — доказанная исправленная причина

FC HIGHRES clock и RPi camera CLOCK_MONOTONIC имеют воспроизводимое расхождение скорости около +2050 ppm (~0.205%, ~2.05 мс/с).

Старый длинный прогон:
- affine slope ≈1.0020527;
- накопленный drift ≈236 мс;
- HIGHRES bracket initially работал лишь в начале.

Независимый новый прогон подтвердил около +2056 ppm.

Введён affine FC-time → RPi CLOCK_MONOTONIC mapper по lower-envelope receive offsets. После этого прежний отказ «HIGHRES valid только первые секунды» устранён.

Не возвращаться к receive timestamp как основному времени измерения: MAVLink transport jitter делает его хуже FC measurement timestamp + affine mapping.

## 9. Gyro scale — гипотеза отвергнута

Старое предположение `gyro × 1.037` было результатом сравнения разных наборов кадров.

На одинаковых valid intervals:
- ordinary gyro ΔR ≈72.184°;
- HIGHRES ΔR ≈72.206°;
- разница ≈0.022° (~0.03%).

Не вводить gyro scale correction без нового независимого доказательства.

## 10. Camera↔gyro phase — не основная причина

Phase sweep V1 был НЕВАЛИДЕН: положительные offsets требовали HIGHRES samples из будущего и теряли coverage.

Исправление: commit `aad92ab`, delayed phase sweep V2; одинаковая доступность всех вариантов.

V2, основной yaw, common-valid:
- -5 ms: false XY ≈32.99 мм;
- 0 ms: ≈32.88 мм;
- +3 ms: ≈32.82 мм;
- +5 ms: ≈32.78 мм;
- +7 ms: ≈32.74 мм;
- +10 ms: ≈32.70 мм.

0→+10 ms улучшает лишь ~0.18 мм (~0.5%). Малый phase effect возможен, но он не объясняет основной yaw residual. Не расширять sweep без нового основания и не hardcode'ить +6/+7 ms.

## 11. Focal / intrinsics — статус

Текущий YAML содержит fx≈568.53, fy≈569.68. Исторически:
- ChArUco physical-height оценка давала k≈1.0925 → fx≈621.1, fy≈622.4;
- MOVE500 давал k≈1.1056 → fx≈628.6, fy≈629.8;
- оценки отличались примерно на 1.2%.

Это важный исторический результат, но НЕ доказательство текущего дефекта WORKED5. Позднее WORKED5 с текущей замороженной конфигурацией прошёл независимые метрические проверки.

Статус: не менять K/focal_scale ради yaw-shadow без эксперимента, который отделяет ошибку intrinsics от уже работающего production масштаба.

## 12. Открытые гипотезы

Пока НЕ доказано, что оставшийся yaw residual вызван:
- малым angular mounting error камеры;
- distortion/intrinsics именно в rotation model;
- LK/feature distribution;
- ground-plane reconstruction;
- остаточной timing/latency ошибкой вне проверенного ±10 ms phase effect;
- ATTITUDE filtering/latency;
- неидеальным физическим yaw (roll/pitch/translation);
- иной систематикой metric shadow.

Следующий эксперимент должен изолировать один из этих факторов, не меняя frozen control.

## 13. Отвергнутые / superseded гипотезы

- REJECTED: `gyro ×1.037` как исправление. Причина: same-frame ordinary/HIGHRES angle отличается ~0.03%.
- REJECTED AS PRIMARY: camera↔gyro phase в диапазоне -5…+10 ms. Причина: V2 меняет false XY всего ~0.5%.
- INVALID EXPERIMENT: phase sweep V1. Причина: causal evaluation положительных offsets требовала будущие gyro samples.
- DO NOT INFER: старые focal calibration результаты автоматически означают, что focal WORKED5 неверен. Позднейшие blind tests имеют более высокий приоритет для production baseline.
- NOT SUPPORTED: простая ошибка арифметики lever subtraction. Проверено camera - lever = IMU.

## 14. Web UI — сохранять

Уже реализованные/важные функции:
- запуск/остановка записи прогона;
- после остановки modal имени прогона;
- вкладка «журнал» и browser логов;
- скачивание одного/нескольких логов;
- Flight/dashboard, telemetry, FC ARM/DISARM/modes;
- realtime диагностические отображения.

`scripts/run_web.sh` вручную поднимает web service на 0.0.0.0:8080. Persistent/autostart вебморды пока отдельная незакрытая задача. Не ломать web UI при ΔR исследованиях.

## 15. Экспериментальная дисциплина

Перед новым экспериментом фиксировать:

1. HYPOTHESIS — ровно одну проверяемую причину.
2. PREDICTION — наблюдаемый результат, если причина верна.
3. CONTROL — что запрещено менять.
4. VERDICT RULE — заранее заданный критерий подтверждения/опровержения.
5. RUN / COMMIT — точные артефакты.
6. RESULT — числа.
7. VERDICT — proven / rejected / inconclusive / invalid.

Нельзя менять production/frozen параметр только потому, что он улучшает один shadow-run.

## 16. Текущий следующий шаг

Не менять WORKED5, focal, geometry, gyro scale или clock mapper.

Следующая диагностическая цель: локализовать yaw residual до/после метрической реконструкции. Предпочтительный тест — pixel-domain rotation prediction на тех же LK correspondences:
- HIGHRES ΔR + K/D + B_R_C предсказывает px1 из px0 для чистого вращения;
- сравнить с реально измеренным LK px1;
- без TF-Luna, lever arm, EKF и N/E metric conversion.

Если pixel-domain rotation residual мал и несистематичен, искать проблему после image rotation model (ground-plane/lever/metric conversion). Если residual систематичен по направлению/положению кадра — искать camera model/extrinsics/distortion/timing/feature behavior.

Сначала проверить, сохраняются ли необходимые per-feature correspondences в существующих логах; если нет — добавить только shadow logger.
