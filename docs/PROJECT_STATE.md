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

## 6.1. Pixel-domain yaw localization — 2026-09-20

Прогон `20260920_124420_OPTICAL_FLOW` добавил прямую проверку наблюдаемого LK flow против вращения, предсказанного HIGHRES ΔR, до range/ground-plane/lever/EKF.

Доказано на активном yaw:
- в покое median pixel residual ≈ 0.107 px;
- при |dyaw| > 0.1° median ≈ 1.718 px;
- при |dyaw| > 0.3° median ≈ 2.329 px;
- при |dyaw| > 0.5° median ≈ 2.886 px;
- residual преимущественно по image-u: при |dyaw| > 0.3° median du ≈ -2.263 px, dv ≈ +0.246 px;
- корреляция HIGHRES integrated angle с pixel residual ≈ 0.962.

На основном common-valid yaw metric HIGHRES residual IMU был порядка 36 мм (N≈-18 мм, E≈+31 мм).

Вывод: систематическое расхождение уже присутствует в pixel-domain. TF-Luna, ground-plane reconstruction, lever subtraction, N/E conversion и EKF не являются первым местом возникновения этой ошибки. Это не доказывает конкретную первопричину: остаются rotation convention/order, camera↔body angular extrinsic, camera model/distortion и иные причины до metric reconstruction.

Следующий эксперимент: на одних и тех же LK correspondences параллельно сравнить текущий HIGHRES `ΔR^T`, альтернативный `ΔR` и endpoint rotation из ATTITUDE. Production/WORKED5 не менять.

## 6.2. Rotation convention control — 2026-09-20

Прогон `20260920_124928_OPTICAL_FLOW` сравнил на одних LK correspondences текущий HIGHRES `ΔR^T`, альтернативный `ΔR` и endpoint rotation ATTITUDE.

На активном yaw (|dyaw| > 0.1°, 229 кадров):
- HIGHRES `ΔR^T`: median residual ≈1.442 px, du≈-1.319 px, dv≈+0.045 px;
- HIGHRES direct `ΔR`: ≈1.571 px, du≈-0.417 px, dv≈-0.293 px;
- ATTITUDE endpoints: ≈1.328 px, du≈-1.229 px, dv≈-0.062 px.

При |dyaw| > 0.3°: HIGHRES `ΔR^T` ≈1.856 px, direct `ΔR` ≈2.045 px, ATTITUDE ≈1.721 px.

Выводы:
- гипотеза простой ошибки transpose/convention не подтверждена; direct `ΔR` хуже и production convention менять нельзя;
- ATTITUDE немного лучше HIGHRES уже в pixel-domain, что согласуется с прежним metric-shadow сравнением;
- ни один вариант не устраняет систематический yaw residual, доминирующий по image-u;
- следующий тест — shadow-only sweep малого angular camera extrinsic по трём осям, без изменения frozen geometry/WORKED5.

## 6.3. Ограничение ручных yaw-тестов и extrinsic sweep — 2026-09-20

Условия yaw-тестов: аппарат вращается рукой. Точный итоговый угол (включая 90°) не является ground truth; неизбежны небольшие поступательные смещения, сопутствующие roll/pitch и дрожание. Анализ должен опираться на фактический покадровый ΔR/ATTITUDE, а не на номинальные 90°.

Первый angular-extrinsic sweep показал чувствительность pixel residual преимущественно к camera-roll: в активном yaw baseline около 1.406 px, а +5° около 1.126 px; pitch/yaw sweep почти плоский. Это доказывает чувствительность residual к этой степени свободы, но НЕ доказывает физическую ошибку установки камеры на +5°: ручная трансляция/наклон могут частично компенсироваться неправильным angular extrinsic.

Поэтому расширять roll sweep до больших углов и тем более менять production `B_R_C` сейчас нельзя. Следующий диагностический шаг должен использовать пространственную структуру residual по отдельным LK features, чтобы отделить rotation-like mismatch от translation-like optical-flow component.

## 6.4. Правило интерпретации ручных стендовых прогонов — 2026-09-20

Все текущие стендовые движения выполняются рукой. Это постоянное условие эксперимента, а не исключение. Любой nominal yaw/pitch/roll может одновременно содержать реальную XYZ-трансляцию, сопутствующие вращения по другим осям и дрожание. Номинальный конечный угол (например 90°) не является ground truth.

Следствия:
- нельзя считать весь LK residual относительно pure-rotation prediction ошибкой ΔR/extrinsic;
- нельзя оценивать angular extrinsic по одному ручному yaw и переносить найденный минимум в production;
- pixel residual, растущий с угловым шагом, является наблюдением, но сам по себе не отделяет rotation-model error от реальной camera translation;
- новые физические прогоны не запрашивать, пока существующие логи способны проверить вопрос; новый прогон допустим только для заранее сформулированной гипотезы с критерием результата, которую принципиально нельзя проверить имеющимися данными.

Переоценка накопленных данных с этим ограничением:
- `124420`, `124928` и extrinsic-run воспроизводят рост pixel residual на активном yaw, но это НЕ доказательство ошибки HIGHRES ΔR;
- direct `ΔR` хуже `ΔR^T` остаётся полезным convention-control;
- преимущество ATTITUDE над HIGHRES в metric/pixel residual также не превращает ATTITUDE в ground truth;
- roll +5° sweep считается только sensitivity result, не calibration result;
- clock-rate mismatch и исправление FC→RPi affine mapping остаются независимо доказанными.

## 6.5. HIGHRES vs ATTITUDE gyro source — 2026-09-20

Код проекта подтверждает: ordinary gyro shadow берёт `rollspeed/pitchspeed/yawspeed` из MAVLink ATTITUDE, а HIGHRES shadow берёт `xgyro/ygyro/zgyro` непосредственно из HIGHRES_IMU.

Проверка ArduPilot source показывает принципиальное различие источников:
- HIGHRES_IMU формируется из `AP::ins().get_gyro()`;
- ATTITUDE rates формируются из `AP::ahrs().get_gyro()`;
- `AP_AHRS::get_gyro()` документирован как smoothed gyro corrected for drift;
- AHRS MAVLink message передаёт `omegaI = ahrs.get_gyro_drift()`;
- ArduPilot `get_gyro_latest()` явно вычисляет `AP::ins().get_gyro(primary) + get_gyro_drift()`.

Следовательно, обнаруженный static HIGHRES X offset нельзя трактовать как ошибку clock/extrinsic: наиболее прямой кандидат — отсутствие AHRS/EKF drift correction в HIGHRES_IMU относительно ATTITUDE rates. Знак correction должен проверяться данными, не предполагаться.

В shadow-код добавлен захват MAVLink AHRS omegaIx/Iy/Iz и логирование рядом с raw HIGHRES, включая диагностическое `raw + omegaI`. Production/WORKED5 не меняются. Специальный физический прогон ради этого не требуется; данные будут собраны при следующем естественном запуске системы.

## 6.6. AHRS omegaI подтверждён данными и добавлен corrected HIGHRES shadow — 2026-09-20

Статический capture после commit 6445573: 1870 HIGHRES samples, AHRS omegaI доступен для 1865 (99.73%).

Средние значения:
- raw HIGHRES: gx=+0.00636251, gy=-0.00052951, gz=-0.00079928 rad/s;
- AHRS omegaI: x=-0.00624566, y=+0.00055629, z=+0.00080563 rad/s;
- raw+omegaI: gx=+0.00011685, gy=+0.00002678, gz=+0.00000635 rad/s.

Таким образом, ранее обнаруженный статический HIGHRES offset почти полностью объясняется отсутствием AHRS drift correction в raw HIGHRES_IMU. Это существенно сильнее гипотез clock/extrinsic для поведения около нулевого вращения.

Commit 3c73b70 добавляет HIGHRES_CORRECTED_SHADOW_V1:
- тот же HIGHRES time_usec;
- тот же affine FC->RPi clock map;
- на каждом HIGHRES sample применяется свежий AHRS omegaI;
- corrected поток интегрируется тем же integrateBodyRates();
- corrected camera/lever/IMU delta и residual пишутся в deltar_rotation_shadow.csv;
- WORKED5, production OPTICAL_FLOW и FC output не меняются.

Критерий: corrected HIGHRES должен уменьшить static/low-rate накопление относительно raw HIGHRES, не ухудшая интервалы заметного вращения. Ручное движение не считается pure rotation; абсолютный XY residual не трактуется как ground-truth ошибка.

## 6.7. HIGHRES_CORRECTED_SHADOW подтверждён на paired runtime — 2026-09-20

Run: 20260920_135641_OPTICAL_FLOW. Сравнение выполнено только на кадрах, где одновременно валидны ATTITUDE, ordinary gyro, raw HIGHRES и corrected HIGHRES.

Ключевой low-rate результат при фактическом Δθ < 0.01° (1567 кадров):
- ATTITUDE endpoints: cumulative XY 1.73 mm;
- ordinary gyro: 1.37 mm;
- raw HIGHRES: 22.81 mm;
- corrected HIGHRES: 1.30 mm.

Таким образом, AHRS omegaI уменьшил характерное low-rate накопление raw HIGHRES примерно в 17.6 раза и привёл его практически к ordinary gyro.

При существенном вращении correction не разрушает масштаб:
- Δθ > 0.3°: ordinary gyro Σangle 217.463°, raw HIGHRES 217.705°, corrected HIGHRES 217.765°;
- corrected против ordinary отличается примерно на 0.14% по суммарному углу этой выборки.
Visual residual также не ухудшился: при Δθ > 0.3° raw HIGHRES и corrected HIGHRES дают около 0.199 mm median residual.

Интерпретация:
- low-rate дефект raw HIGHRES локализован как отсутствие AHRS gyro drift correction;
- clock affine mapping сохраняется;
- ручной bias, gyro scale, focal и extrinsic для исправления этого дефекта не нужны;
- ручные движения стенда не считаются pure rotation, поэтому абсолютный XY residual не является ground truth.

Архитектурный guardrail перед production:
ArduPilot OPTICAL_FLOW получает raw angular image flow и сам выполняет компенсацию body rate. Поэтому нельзя напрямую заменить production flow на translation-only metric delta из corrected ΔR: это привело бы к двойной компенсации вращения. Следующий production-кандидат должен либо сохранить AP-compatible raw-flow semantics, либо явно изменить весь интерфейс компенсации. До такого A/B WORKED5 и отправляемый OPTICAL_FLOW остаются без изменений.

## 6.8. Production interface audit: corrected ΔR нельзя просто подставить в OPTICAL_FLOW — 2026-09-20

Проверен текущий publisher и актуальный ArduPilot MAV optical-flow backend.

Текущий monkeysStab отправляет MAVLink OPTICAL_FLOW через float `flow_rate_x/y`. Это raw angular image-flow semantics. В ArduPilot MAV backend при обычном режиме `state.bodyRate` берётся из `AP::ahrs().get_gyro()`; далее EKF формирует motion-compensated flow как `-rawFlowRates + rawGyroRates`. При `FLOW_OPTIONS` с опцией Stabilised backend вместо этого принудительно ставит bodyRate=0.

Следствия:
- production WORKED5/current publisher уже рассчитан на то, что roll/pitch body-rate compensation выполняет ArduPilot;
- HIGHRES_CORRECTED metric shadow уже использует camera-aligned ΔR для удаления вращения и выдаёт translation-like результат;
- прямая отправка этого результата в существующем нестабилизированном режиме дала бы повторную компенсацию body rate;
- corrected HIGHRES поэтому не должен заменять `flow_send_x/y` без изменения контракта с FC.

Два архитектурно корректных варианта:
A) сохранить текущий AP-compatible raw-flow контракт; corrected HIGHRES использовать только для диагностики/улучшения оценки, но в отправляемом flow должна оставаться соответствующая вращательная составляющая;
B) полностью компенсировать вращение на RPi с camera-aligned corrected ΔR и отправлять уже stabilised flow, одновременно переводя MAV backend в `FLOW_OPTIONS=1` (Stabilised), чтобы ArduPilot не вычитал body rate второй раз.

Вариант B потенциально лучше использует найденное преимущество corrected HIGHRES: точный FC measurement timestamp -> affine map в camera CLOCK_MONOTONIC -> интеграл строго на интервале кадров. Но это отдельный A/B-кандидат и требует проверки знаков, единиц, lever-arm semantics и fallback до изменения flight production.

Дополнительный timing guardrail: MAV backend сейчас ставит время кадра по моменту получения сообщения (`AP_HAL::micros64()`), а не по `OPTICAL_FLOW.time_usec`; поэтому существующее ограничение pipeline latency остаётся необходимым.

До завершения этого A/B:
- WORKED5 frozen;
- production `flow_send_x/y` не менять;
- `FLOW_OPTIONS` автоматически не менять;
- focal/extrinsic/gyro scale не трогать.

## 6.9. Stabilised corrected-ΔR flow shadow — 2026-09-20

Commit `2143d59` добавляет только diagnostic shadow для будущего AP `FLOW_OPTIONS=Stabilised`; FC publisher не изменён.

Из уже рассчитанного `HIGHRES_CORRECTED` metric step:
- берётся FC/IMU displacement после full ΔR и lever-arm correction;
- local velocity переводится через `R0^T` в body FRD;
- используется та же геометрия camera height, что в metric estimator;
- формируется stabilised angular flow для downward camera:
  `flow_x=-v_body_y/h`, `flow_y=+v_body_x/h`.

В `deltar_rotation_shadow.csv` добавлены:
`stabilised_shadow_valid,stabilised_shadow_flow_x,stabilised_shadow_flow_y`.

Этот контур не вызывает `sendOpticalFlow()`, не меняет `flow_send_x/y`, WORKED5, EKF или FC parameters. Его задача — проверить единицы/знаки/поведение будущего stabilised interface до любого flight A/B.

## 6.10. Stabilised flow: внутренний аудит знаков и единиц — 2026-09-20

Commit `b398df3` расширяет только diagnostic shadow. Production publisher и FC parameters не меняются.

Для каждого valid stabilised interval теперь логируются:
- точная camera height `h0`;
- `v_local[N,E,D]` до преобразования системы координат;
- `v_body[x,y,z]` после `R0^T`;
- stabilised `flow_x/flow_y`;
- обратное преобразование `flow -> body vx/vy`;
- `stabilised_shadow_roundtrip_err`.

Контракт:
`flow_x=-v_body_y/h`, `flow_y=+v_body_x/h`.
Обратная проверка:
`vx=flow_y*h`, `vy=-flow_x*h`.

Критерий: round-trip error должен быть только численной погрешностью. Это проверяет внутренние знаки и единицы интерфейса без предположения о чистом ручном yaw и без нового специального физического теста.

Также исправлена единица в 6.7: `highres_corr_residual_median_m` ранее был ошибочно подписан как px; корректная единица после ×1000 — mm.

## 6.11. Stabilised flow round-trip — PASS — 2026-09-20

Run: `20260920_144422_OPTICAL_FLOW`.

- rows: 6238;
- `stabilised_shadow_valid`: 5047;
- `stabilised_shadow_roundtrip_err` median: 0;
- max: `2.86098e-17`.

Примеры последних valid кадров при `h≈0.195–0.205 m` восстанавливают `v_body[x,y]` из stabilised `flow_x/y` до машинной точности.

Вердикт: внутренняя цепочка единиц и знаков shadow-контракта
`flow_x=-v_body_y/h`, `flow_y=+v_body_x/h`
алгебраически согласована. Это закрывает только внутренний round-trip; соответствие внешней знаковой конвенции ArduPilot остаётся отдельной проверкой. Production publisher и `FLOW_OPTIONS` не изменены.

## 6.12. ArduPilot Stabilised / FLOW_POS contract — 2026-09-20

Аудит актуального ArduPilot master уточнил внешний контракт.

1. `AP_OpticalFlow_MAV` при `Option::Stabilised` зануляет только передаваемые в EKF X/Y body rates; packet `flow_rate_x/y` остаётся angular optical flow.
2. EKF `writeOptFlowMeas()` преобразует `flowRadXY=-rawFlowRates`, а Z body rate всегда восстанавливает из nav IMU, потому что optical-flow interface передаёт только X/Y gyro.
3. `FuseOptFlow()` моделирует скорость focal point как
   `relVelSensor = body_velocity + bodyRadXYZ × posOffsetBody`.
   Следовательно при Stabilised сохраняется yaw lever-arm term через `bodyRadXYZ.z × FLOW_POS`.
4. Текущий RPi stabilised shadow уже выдаёт FC/IMU-centric translation: полный camera lever arm вычтен до формирования flow. Если одновременно оставить ненулевой camera `FLOW_POS`, EKF повторно добавит как минимум yaw lever-arm term. Контракты несовместимы.
5. Для полного RPi full-rotation + full-lever compensation чистый EKF контракт требует effective optical-flow position offset = 0. Альтернативно RPi должен вернуть sensor/focal-point semantics вместо IMU-centric semantics.

Решение для дальнейшего A/B: не менять production сейчас. Сначала добавить shadow-кандидат sensor-centric Stabilised flow, сохраняющий camera focal-point displacement после rotation compensation, но ДО вычитания lever arm. Он должен быть совместим с реальным camera FLOW_POS в EKF. Сравнить его с IMU-centric вариантом офлайн/в shadow. Никаких специальных ручных прогонов для этого не требуется.

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
