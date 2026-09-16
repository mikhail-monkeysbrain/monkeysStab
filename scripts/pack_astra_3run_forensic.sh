#!/usr/bin/env bash
set -euo pipefail

OUT="/home/vio/ASTRA_3RUN_FORENSIC_20260916"
ARCHIVE="${OUT}.tar.gz"
rm -rf "$OUT" "$ARCHIVE"
mkdir -p "$OUT"

RAW1="/home/vio/monkeysStab_datasets/20260916_192841_ASTRA_AB_RAW"
BLIND4="/home/vio/monkeysStab_datasets/20260916_202937_ASTRA_BLIND4_RAW"
CANON="/home/vio/monkeysStab_datasets/20260916_123940_CANONICAL"

copy_dataset() {
  local src="$1" dst="$2"
  [[ -d "$src" ]] || { echo "MISSING DATASET: $src" >&2; exit 2; }
  mkdir -p "$OUT/$dst"
  cp -a "$src"/. "$OUT/$dst"/
}

copy_dataset "$RAW1" "01_ASTRA_AB_RAW"
copy_dataset "$BLIND4" "02_ASTRA_BLIND4_RAW"
copy_dataset "$CANON" "03_CANONICAL"

cat > "$OUT/README_FOR_ASTRA_RU.txt" <<'EOF'
ASTRA — THREE EXISTING RUNS FORENSIC PACKAGE
============================================

ЦЕЛЬ
Разработать/проверить алгоритм оценки физического горизонтального перемещения
по уже записанным данным OV9281 + TF-Luna + FC. Новый эксперимент специально
не проводился. Не подгонять алгоритм под известный ответ.

АППАРАТУРА / ГЕОМЕТРИЯ
Raspberry Pi 5.
OV9281 USB, 640x480 MJPG.
FC MatekH743, ArduCopter 4.7.0.
TF-Luna подключён к RPi.
INS_POS1 = (0, 0, 0) m.
OV9281/FLOW position relative FC = (+0.0625, 0, +0.0500) m.
TF-Luna position relative FC = (+0.0855, 0, +0.0550) m.
Камера смотрит вниз в рабочей конфигурации.
Аппарат перемещался руками по поверхности стола, не отрывая от неё.
Небольшие roll/pitch/yaw и боковой уход допустимы.
Стол НЕ выровнен по горизонту и НЕ является идеально плоским.
Нельзя считать постоянную высоту камеры над идеальной горизонтальной плоскостью
истинной без проверки по данным TF-Luna/attitude.

DATASETS
01_ASTRA_AB_RAW
  Полный ранее записанный RAW A/B/A dataset: frames.mjpgbin, frames.csv,
  optical_flow_mavlink.csv, FC DataFlash и manifest.
  Этот набор уже использовался в предыдущем forensic-анализе и является
  development/reference, а не blind validation.

02_ASTRA_BLIND4_RAW
  Непрерывная запись четырёх физических A->B проходов.
  return_event:
    11=A1, 12=B1
    13=A2, 14=B2
    15=A3, 16=B3
    17=A4
  Метка 18=B4 отсутствует ИЗ-ЗА подтверждённой ошибки capture-программы:
  после SPACE на B4 процесс завершался до записи pending event=18 в следующую
  CSV-строку. Оператор фактически нажал B4; RAW запись и DataFlash завершились.
  Первые три legs имеют обе сохранённые границы. Четвёртый leg не использовать
  как строгую количественную validation-секцию без явного метода восстановления
  границы и отдельной маркировки результата как recovered/inferred.

03_CANONICAL
  Ранее записанный canonical dataset. Использовать как независимый источник
  покадровых изображений/таймингов/production flow для проверки причин
  нерецепрокности и устойчивости estimator.

BLIND DISCIPLINE
Не пытаться извлекать/угадывать скрытые физические GT из имён, предыдущих
обсуждений или косвенных подсказок. Сначала зафиксировать estimator и выдать
оценки расстояния для пригодных legs вместе с uncertainty/diagnostics.
Только после этого сравнивать с отдельно раскрываемым ground truth.

ТРЕБОВАНИЯ К АНАЛИЗУ
1. Сначала audit целостности, timestamps, dropped/gap intervals и event bounds.
2. Работать с исходными JPEG/кадрами, а не только с production optical flow.
3. Использовать RAW FC/IMU/attitude/range там, где они реально улучшают модель.
4. Учитывать наклон/неровность стола и изменение roll/pitch/yaw.
5. Отдельно диагностировать camera gaps, большие межкадровые displacement,
   деградацию correspondences/inliers и потерю движения.
6. Не считать EKF ground truth.
7. Не калибровать масштаб на hidden validation GT.
8. Выдать воспроизводимый код estimator + команды replay.
9. Для каждого leg: estimated displacement vector, magnitude, confidence/
   uncertainty, coverage, rejected intervals и причины.
10. Явно разделить measured, inferred и assumed quantities.

ВАЖНО
focal_scale production на этих тестах был 0.931. Не считать это метрическим
ground truth; при необходимости проверить модель камеры независимо по RAW.
EOF

{
  echo "package_created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "repo_head=$(cd "$HOME/Desktop/monkeysStab" && git rev-parse HEAD)"
  echo "repo_branch=$(cd "$HOME/Desktop/monkeysStab" && git branch --show-current)"
  echo "source_01=$RAW1"
  echo "source_02=$BLIND4"
  echo "source_03=$CANON"
  echo "blind4_expected_events=11,12,13,14,15,16,17"
  echo "blind4_missing_event=18"
  echo "blind4_missing_event_reason=capture_shutdown_before_pending_event_csv_persist"
} > "$OUT/PACKAGE_MANIFEST.txt"

# Include exact source/launchers used to interpret logs, without modifying datasets.
mkdir -p "$OUT/code_snapshot"
cp -a "$HOME/Desktop/monkeysStab/src/optical_flow_mavlink.cpp" "$OUT/code_snapshot/"
cp -a "$HOME/Desktop/monkeysStab/scripts/run.sh" "$OUT/code_snapshot/"
for f in capture_astra_ab_dataset.sh capture_astra_blind4_dataset.sh; do
  [[ -f "$HOME/Desktop/monkeysStab/scripts/$f" ]] && cp -a "$HOME/Desktop/monkeysStab/scripts/$f" "$OUT/code_snapshot/"
done

echo "===== PACKAGE INVENTORY ====="
du -sh "$OUT"/01_ASTRA_AB_RAW "$OUT"/02_ASTRA_BLIND4_RAW "$OUT"/03_CANONICAL
find "$OUT" -maxdepth 2 -type f -printf '%P %s bytes\n' | sort > "$OUT/FILE_INVENTORY.txt"

tar -C "$(dirname "$OUT")" -czf "$ARCHIVE" "$(basename "$OUT")"

echo
echo "===== ASTRA 3-RUN ARCHIVE READY ====="
ls -lh "$ARCHIVE"
sha256sum "$ARCHIVE"
echo "Archive: $ARCHIVE"
