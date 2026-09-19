#!/usr/bin/env python3
from pathlib import Path

p=Path("tools/run_imu_zupt_guided_test.py")
s=p.read_text(encoding="utf-8")

old='''    print()
    if not ready:
        print("ОШИБКА: система не готова за 90 секунд. Лог:",LOG);raise SystemExit(4)
'''

new='''    print()
    if not ready:
        print("ОШИБКА: система не готова за 90 секунд.")
        print("Причина: router/Web/runtime были запущены, но guided-test не получил готовую camera/IMU telemetry")
        print("        (ожидались imu_cam_fresh=true и imu_cam_speed != null).")
        print("Лог:", LOG)
        print("\\n===== ПОСЛЕДНИЕ СТРОКИ SERVICE LOG =====")
        try:
            with open(LOG, "r", encoding="utf-8", errors="replace") as lf:
                tail = lf.readlines()[-80:]
            if tail:
                print("".join(tail).rstrip())
            else:
                print("(лог пуст)")
        except Exception as e:
            print("Не удалось прочитать лог:", e)
        try:
            d=get()
            print("\\n===== ПОСЛЕДНЯЯ TELEMETRY =====")
            print("imu_cam_fresh =", d.get("imu_cam_fresh"))
            print("imu_cam_speed =", d.get("imu_cam_speed"))
            print("imu_cam_seq =", d.get("imu_cam_seq"))
        except Exception as e:
            print("\\nПоследнюю telemetry получить не удалось:", repr(e))
        raise SystemExit(4)
'''

if old not in s:
    raise SystemExit("readiness timeout anchor not found; source not modified")

s=s.replace(old,new,1)

old2='''        if proc.poll() is not None:
            print("\\nОШИБКА запуска. Лог:",LOG);raise SystemExit(3)
'''
new2='''        if proc.poll() is not None:
            print("\\nОШИБКА запуска: сервисный процесс завершился, code =", proc.returncode)
            print("Лог:", LOG)
            print("\\n===== ПОСЛЕДНИЕ СТРОКИ SERVICE LOG =====")
            try:
                with open(LOG, "r", encoding="utf-8", errors="replace") as lf:
                    print("".join(lf.readlines()[-80:]).rstrip())
            except Exception as e:
                print("Не удалось прочитать лог:", e)
            raise SystemExit(3)
'''
if old2 not in s:
    raise SystemExit("early-exit anchor not found; source not modified")
s=s.replace(old2,new2,1)

p.write_text(s,encoding="utf-8")
print("patched",p)
print("guided startup failures now print reason, log tail and telemetry; test UI otherwise unchanged")
