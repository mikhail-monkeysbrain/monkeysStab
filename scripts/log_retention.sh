#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT="${MONKEYS_RUN_ROOT:-$HOME/monkeysStab_runs}"
BUDGET_MB="${MONKEYS_RUNTIME_LOG_BUDGET_MB:-5120}"
MIN_FREE_MB="${MONKEYS_RUNTIME_MIN_FREE_MB:-3072}"

mkdir -p "$RUN_ROOT"

# Только автоматически создаваемые production-каталоги. Именованные записи
# RUN_ROOT/recordings, DataFlash capture и прочие forensic-файлы не трогаются.
mapfile -t RUN_DIRS < <(
  find "$RUN_ROOT" -mindepth 1 -maxdepth 1 -type d     -name '????????_??????_OPTICAL_FLOW' -printf '%T@ %p\n' 2>/dev/null   | sort -n
)

dir_kb() {
  du -sk -- "$1" 2>/dev/null | awk '{print $1+0}'
}

total_kb=0
for entry in "${RUN_DIRS[@]}"; do
  p="${entry#* }"
  (( total_kb += $(dir_kb "$p") ))
done

budget_kb=$((BUDGET_MB * 1024))
min_free_kb=$((MIN_FREE_MB * 1024))
free_kb=$(df -Pk "$RUN_ROOT" | awk 'NR==2 {print $4+0}')

echo "LOG RETENTION: runtime=$((total_kb/1024)) MB budget=${BUDGET_MB} MB free=$((free_kb/1024)) MB min_free=${MIN_FREE_MB} MB"

deleted=0
for entry in "${RUN_DIRS[@]}"; do
  if (( total_kb <= budget_kb && free_kb >= min_free_kb )); then
    break
  fi
  p="${entry#* }"
  kb=$(dir_kb "$p")
  echo "LOG RETENTION: удаляю старый runtime: $p ($((kb/1024)) MB)"
  rm -rf --one-file-system -- "$p"
  (( total_kb -= kb ))
  (( deleted += 1 ))
  free_kb=$(df -Pk "$RUN_ROOT" | awk 'NR==2 {print $4+0}')
done

free_kb=$(df -Pk "$RUN_ROOT" | awk 'NR==2 {print $4+0}')
if (( free_kb < min_free_kb )); then
  echo "ОШИБКА: после очистки свободно только $((free_kb/1024)) MB; требуется ${MIN_FREE_MB} MB." >&2
  echo "Именованные recordings/forensic данные автоматически не удаляются." >&2
  exit 3
fi

echo "LOG RETENTION: OK deleted=${deleted} runtime=$((total_kb/1024)) MB free=$((free_kb/1024)) MB"
