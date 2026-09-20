# JT-Zero / monkeysStab — Worklog

## 2026-09-20 — конец сессии

### Наблюдение: stop/start runtime при непрерывном ARM

Серия run1/run2/run3 выполнялась без DISARM между прогонами: пользователь останавливал и снова запускал систему через веб-интерфейс, а FC/EKF3 оставался в одной непрерывной ARM-сессии.

Следствие: эти три файла нельзя трактовать как три независимых EKF-эксперимента. При остановке runtime исчезал выбранный горизонтальный источник `EK3_SRC1_VELXY=5 (OpticalFlow)`, затем появлялся снова, тогда как EKF продолжал жить.

Текущая гипотеза: сочетание длительного ARM + исчезновения/возврата OPTICAL_FLOW может быть связано с деградацией состояния EKF между run1/run2/run3. Это пока не доказанная причинность: RPi CSV не содержит промежутки, когда runtime был остановлен.

Отдельное наблюдение пользователя: после этой серии выполнена проверка без ARM, ориентируясь по веб-интерфейсу. Видимых проблем не обнаружено («вроде бы всё работает»). Это наблюдение по UI, не строгий логовый эксперимент. Оно усиливает необходимость отделять обычный stop/start diagnostic runtime от поведения FC/EKF в непрерывной ARM-сессии.

До дополнительного forensic не использовать run1/run2/run3 как три независимых оценки точности EKF. Run3 особенно не использовать как метрический accuracy-test: деградация EKF наблюдалась уже до физического движения.

### ViSP как независимый диагностический контур

Рассмотрена ViSP (Visual Servoing Platform, Inria). Решение на текущем этапе: НЕ включать ViSP в production Variant A/B и НЕ делать её зависимостью flight runtime. Использовать как независимый диагностический/reference-инструмент на PC/Linux-песочнице.

Предлагаемая архитектура:

```text
RPi5 / аппарат
  production: OV9281 -> monkeysStab -> Variant B -> MAVLink -> FC
  diagnostic tap:
      original camera frames + original timestamps
      TF-Luna
      ATTITUDE
      corrected HIGHRES / необходимые gyro данные
      EKF state
      WORKED5 / Variant-B result as reference
              |
           Ethernet
              |
              v
PC diagnostic station
  recorder -> reproducible dataset
  live/replay -> ViSP KLT / homography / geometry
  comparison -> WORKED5 / Variant B / physical ground truth
```

Ключевые требования:

1. PC/ViSP не должен возвращать данные в production-контур. Отказ сети, PC или ViSP не должен влиять на отправку OPTICAL_FLOW в FC.
2. Для независимой диагностики передавать исходные кадры, а не изображение вебморды и не только уже рассчитанный flow.
3. Сохранять исходный camera monotonic timestamp и frame_id. Не создавать новую проблему синхронизации отдельными несвязанными video/telemetry clocks.
4. Предпочтительно передавать исходные MJPEG bytes OV9281 без decode/re-encode на RPi. Production продолжает декодировать кадр локально; diagnostic tap отправляет тот же compressed frame.
5. Первая версия может передавать каждый второй кадр (~50 fps) при сохранении оригинальных timestamps; production остаётся на своей частоте.
6. Калибровка K/D, mount geometry и configuration hash передаются как session metadata, а не дублируются на каждом кадре.
7. На PC обязательны два режима: LIVE и RECORD/REPLAY. Главная ценность — воспроизводимый dataset, который можно многократно прогонять разными алгоритмами без нового ручного физического движения.
8. ViSP использовать сначала как независимый shadow/reference estimator: KLT, planar homography, camera/pose geometry. Не ожидать автоматического улучшения только от замены OpenCV LK на обёртку ViSP.
9. Сравнивать на одном dataset текущий estimator, ViSP KLT/homography и физический ground truth. Это должно помочь отделить frontend/геометрию от общей ошибки масштаба.
10. Diagnostic stream проектировать не специфичным для ViSP: будущими потребителями могут быть OpenCV, собственные estimators, VIO и другие offline-инструменты.

### Важное изменение экспериментального lifecycle

Запись/разделение прогонов не должна требовать остановки production runtime.

Целевая схема:

```text
production runtime:  ------------------------------------> непрерывно

RUN1              RUN2              RUN3
record start/stop record start/stop record start/stop
LOG ONLY          LOG ONLY          LOG ONLY
```

Кнопка записи/остановки диагностического прогона должна управлять только границами dataset/log, а не жизненным циклом источника OPTICAL_FLOW. Это особенно важно при ARM, чтобы между экспериментами не отбирать у EKF3 выбранный VELXY source.

### Следующий шаг

Новых физических прогонов сейчас не требуется.

При продолжении:
- сохранить текущий Variant B/causal35 без изменений;
- закончить forensic серии run1/run2/run3 с учётом единой ARM/EKF-сессии;
- отдельно проверить поведение EKF при OF loss/recovery, не смешивая это с метрической точностью;
- проверить, какие исходные camera artifacts уже сохраняет monkeysStab;
- затем спроектировать JT-Zero Diagnostic Stream: RPi publisher -> Ethernet -> PC recorder/replay -> ViSP shadow;
- до этого не менять WORKED5, focal, geometry, causal35 или FC parameters на основании текущей серии.
