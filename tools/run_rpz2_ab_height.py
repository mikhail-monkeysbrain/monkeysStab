#!/usr/bin/env python3
import csv
import pathlib
import time
import os

EVENTS=[]

def mark(event, step, title, value=""):
    EVENTS.append((time.monotonic_ns(), event, step, title, value))

def save_timeline(distance_mm=""):
    p=pathlib.Path.home()/"rpz2_ab_height_latest.csv"
    with p.open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["mono_ns","event","step","title","value"])
        w.writerows(EVENTS)
        if distance_mm!="":
            w.writerow([time.monotonic_ns(),"GROUND_TRUTH_AB_MM",0,"AB_LENGTH",distance_mm])
    return p

def wait(msg, step, title):
    mark("WAIT_BEGIN",step,title)
    input("\n"+msg+" ")
    mark("WAIT_END",step,title)

def timed(msg, sec, step, title):
    print(f"\nЭТАП {step}: {msg}")
    mark("STEP_BEGIN",step,title)
    t0=time.monotonic()
    width=36
    while True:
        e=time.monotonic()-t0
        if e>=sec: break
        n=min(width,int(width*e/sec))
        print(f"\r[{'█'*n}{'░'*(width-n)}] {e:4.1f}/{sec:.0f} c",end="",flush=True)
        time.sleep(0.05)
    print(f"\r[{'█'*width}] {sec:4.1f}/{sec:.0f} c   ГОТОВО")
    mark("STEP_END",step,title)

def main():
    print("="*72)
    print("monkeysStab — RPZ2: A→B→A НА ВЫСОТЕ")
    print("="*72)
    print("ВИНТЫ СНЯТЬ. Аппарат должен оставаться ARM весь прогон.")
    print("A→B измеряется по поверхности стола после посадки в B.")
    print("Финальная точная доводка в A выполняется только на земле по меткам.")

    for p in ("/tmp/monkeys_rpz2_capture_first", "/tmp/monkeys_rpz2_capture_last"):
        try:
            os.unlink(p)
        except FileNotFoundError:
            pass
    pathlib.Path("/tmp/monkeys_rpz2_capture_first").touch()
    mark("SNAPSHOT_REQUEST", "1", "FIRST")
    timed("ПОКОЙ. Не двигать аппарат.", 10, 1, "REST_A")

    wait('"приготовиться, нажать enter"',2,"READY_AB")

    print("\nЭТАП 3–5: подъем → проход в B по воздуху → посадка в B.")
    mark("AB_BEGIN",3,"LIFT_AB_LAND_B")
    wait('"нажмите enter после посадки в Б"',5,"LANDED_B")
    mark("AB_END",5,"LIFT_AB_LAND_B")

    print("\nЭТАП 6: измерьте длину AB по поверхности стола.")
    while True:
        raw=input('\n"введите длину прохода в мм" ').strip().replace(",",".")
        try:
            distance=float(raw)
            if distance>0: break
        except ValueError:
            pass
        print("Нужно ввести положительное число в миллиметрах.")
    mark("GROUND_TRUTH_AB_MM",7,"AB_LENGTH",f"{distance:.3f}")

    wait('"приготовиться к возврату в A, нажать enter"',9,"READY_BA")

    print("\nЭТАП 10–11: подъем → перенос в A.")
    mark("BA_BEGIN",10,"LIFT_BA")
    wait('"нажмите enter"',12,"AIR_RETURN_COMPLETE")
    mark("BA_END",12,"LIFT_BA")
    print("Установите аппарат на земле примерно в точке A.")

    print("\nЭТАП 13: уже на земле сдвиньте аппарат ровно в точку старта по меткам.")
    mark("GROUND_FINE_BEGIN",13,"GROUND_FINE_TO_A")
    wait('"нажмите enter по завершении"',14,"FINISH")
    mark("GROUND_FINE_END",14,"GROUND_FINE_TO_A")

    p=save_timeline()
    print("\n"+"="*72)
    print("RPZ2 ЗАВЕРШЁН")
    print(f"AB = {distance:.1f} мм")
    print(f"Timeline: {p}")
    print("Остановите запись прогона в веб-интерфейсе.")
    print("="*72)

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        p=save_timeline()
        print(f"\nRPZ2 ПРЕРВАН. Timeline сохранен: {p}")
        raise SystemExit(130)
