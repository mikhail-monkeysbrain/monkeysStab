#!/usr/bin/env python3
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ROOT_PATH = Path(ROOT)
GEOMETRY_JSON = ROOT_PATH / "config" / "mount_geometry.json"
CAMERA_YAML = ROOT_PATH / "config" / "ov9281_current_mount.yaml"
PARAMS = {
    "cam_x": "FLOW_POS_X",
    "cam_y": "FLOW_POS_Y",
    "cam_z": "FLOW_POS_Z",
    "rng_x": "RNGFND1_POS_X",
    "rng_y": "RNGFND1_POS_Y",
    "rng_z": "RNGFND1_POS_Z",
}

class GeometryGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("monkeysStab — геометрия камеры и дальномера")
        self.geometry("850x620")
        self.minsize(820, 590)

        self.device = tk.StringVar(value=os.environ.get("MONKEYS_FC", "tcp://127.0.0.1:5760"))
        self.baud = tk.StringVar(value=os.environ.get("MONKEYS_FC_BAUD", "460800"))
        self.sysid = tk.StringVar(value=os.environ.get("MONKEYS_FC_SYSID", "1"))
        self.compid = tk.StringVar(value=os.environ.get("MONKEYS_FC_COMPID", "1"))
        self.status = tk.StringVar(value="Готово. Сначала нажмите «ПРОЧИТАТЬ ИЗ FC».")

        self.vars = {k: tk.StringVar(value="") for k in PARAMS}
        self._build()
        self.after(250, self.read_fc)

    def _build(self):
        pad = {"padx": 10, "pady": 6}
        root = ttk.Frame(self, padding=16)
        root.pack(fill="both", expand=True)

        ttk.Label(root, text="Положение датчиков относительно центра FC",
                  font=("DejaVu Sans", 16, "bold")).pack(anchor="w")
        ttk.Label(root, text="Введите координаты в миллиметрах. Система координат ArduPilot FRD: "
                             "X — вперёд, Y — вправо, Z — вниз.",
                  wraplength=790).pack(anchor="w", pady=(4, 12))

        conn = ttk.LabelFrame(root, text="Подключение к полётному контроллеру", padding=10)
        conn.pack(fill="x", pady=(0, 12))
        for col in range(8):
            conn.columnconfigure(col, weight=1)
        ttk.Label(conn, text="Порт").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.device, width=18).grid(row=0, column=1, sticky="ew", **pad)
        ttk.Label(conn, text="Baud").grid(row=0, column=2, sticky="w")
        ttk.Entry(conn, textvariable=self.baud, width=10).grid(row=0, column=3, sticky="ew", **pad)
        ttk.Label(conn, text="SysID").grid(row=0, column=4, sticky="w")
        ttk.Entry(conn, textvariable=self.sysid, width=6).grid(row=0, column=5, sticky="ew", **pad)
        ttk.Label(conn, text="CompID").grid(row=0, column=6, sticky="w")
        ttk.Entry(conn, textvariable=self.compid, width=6).grid(row=0, column=7, sticky="ew", **pad)

        table = ttk.Frame(root)
        table.pack(fill="x", pady=(0, 12))
        table.columnconfigure(0, weight=2)
        for c in (1,2,3):
            table.columnconfigure(c, weight=1)

        ttk.Label(table, text="Датчик", font=("DejaVu Sans", 11, "bold")).grid(row=0,column=0,sticky="w",**pad)
        ttk.Label(table, text="X, мм\\n(+ вперёд)", justify="center").grid(row=0,column=1,**pad)
        ttk.Label(table, text="Y, мм\\n(+ вправо)", justify="center").grid(row=0,column=2,**pad)
        ttk.Label(table, text="Z, мм\\n(+ вниз)", justify="center").grid(row=0,column=3,**pad)

        self._sensor_row(table, 1, "OV9281 / камера", "cam")
        self._sensor_row(table, 2, "TF-Luna / дальномер", "rng")

        note = ttk.LabelFrame(root, text="Что будет записано", padding=10)
        note.pack(fill="x", pady=(0, 12))
        ttk.Label(note, text=(
            "Камера → FLOW_POS_X / FLOW_POS_Y / FLOW_POS_Z\n"
            "TF-Luna → RNGFND1_POS_X / RNGFND1_POS_Y / RNGFND1_POS_Z\n\n"
            "Утилита переводит миллиметры в метры, записывает параметры через MAVLink, "
            "ждёт подтверждение PARAM_VALUE и затем повторно читает все 6 значений."
        ), justify="left", wraplength=780).pack(anchor="w")

        buttons = ttk.Frame(root)
        buttons.pack(fill="x", pady=(2, 12))
        ttk.Button(buttons, text="ПРОЧИТАТЬ ИЗ FC", command=self.read_fc).pack(side="left", padx=(0,10))
        ttk.Button(buttons, text="ЗАПИСАТЬ В FC", command=self.write_fc).pack(side="left", padx=(0,10))
        ttk.Button(buttons, text="ЗАКРЫТЬ", command=self.destroy).pack(side="right")

        status_box = ttk.LabelFrame(root, text="Состояние", padding=10)
        status_box.pack(fill="both", expand=True)
        ttk.Label(status_box, textvariable=self.status, wraplength=780, justify="left").pack(anchor="w")

    def _sensor_row(self, parent, row, label, prefix):
        ttk.Label(parent, text=label).grid(row=row,column=0,sticky="w",padx=10,pady=8)
        for idx, axis in enumerate(("x","y","z"), start=1):
            e=ttk.Entry(parent, textvariable=self.vars[f"{prefix}_{axis}"], width=14, justify="center")
            e.grid(row=row,column=idx,sticky="ew",padx=10,pady=8)

    def _cmd_base(self):
        return [
            os.path.join(ROOT, "build", "fc_param_tool"),
            self.device.get().strip(),
            self.baud.get().strip(),
            self.sysid.get().strip(),
            self.compid.get().strip(),
        ]

    def _run(self, args):
        try:
            cp = subprocess.run(args, text=True, capture_output=True, timeout=25)
        except Exception as e:
            raise RuntimeError(str(e))
        if cp.returncode != 0:
            msg=(cp.stderr or cp.stdout or f"код возврата {cp.returncode}").strip()
            raise RuntimeError(msg)
        return cp.stdout

    @staticmethod
    def _parse_values(text):
        out={}
        for line in text.splitlines():
            if "=" not in line or line.startswith("TARGET "):
                continue
            left,right=line.split("=",1)
            token=right.strip().split()[0]
            try:
                out[left.strip()]=float(token)
            except ValueError:
                pass
        return out

    def read_fc(self):
        try:
            names=list(PARAMS.values())
            out=self._run(self._cmd_base()+["read"]+names)
            vals=self._parse_values(out)
            missing=[p for p in names if p not in vals]
            if missing:
                raise RuntimeError("Не прочитаны параметры: "+", ".join(missing))
            for key,param in PARAMS.items():
                self.vars[key].set(f"{vals[param]*1000.0:.1f}")
            dx=(vals["RNGFND1_POS_X"]-vals["FLOW_POS_X"])*1000
            dy=(vals["RNGFND1_POS_Y"]-vals["FLOW_POS_Y"])*1000
            dz=(vals["RNGFND1_POS_Z"]-vals["FLOW_POS_Z"])*1000
            self.status.set(
                "Параметры прочитаны из FC.\n"
                f"Разнос TF-Luna относительно камеры: ΔX={dx:+.1f} мм, "
                f"ΔY={dy:+.1f} мм, ΔZ={dz:+.1f} мм."
            )
        except Exception as e:
            messagebox.showerror("Ошибка чтения", str(e))
            self.status.set("Ошибка чтения из FC.")

    def _entered_meters(self):
        vals={}
        for key,param in PARAMS.items():
            raw=self.vars[key].get().strip().replace(",", ".")
            if raw=="":
                raise ValueError(f"Не заполнено поле {param}")
            mm=float(raw)
            if not math.isfinite(mm):
                raise ValueError(f"Некорректное значение {param}")
            if abs(mm)>2000:
                raise ValueError(f"{param}: {mm:g} мм выглядит ошибочно (допуск утилиты ±2000 мм)")
            vals[param]=mm/1000.0
        return vals

    def _save_local_geometry(self, vals):
        if GEOMETRY_JSON.exists():
            with GEOMETRY_JSON.open("r", encoding="utf-8") as f:
                cfg=json.load(f)
        else:
            cfg={"frame":"FRD","units":"m","reference":"FC_IMU","camera":{"name":"OV9281"},
                 "rangefinder":{"name":"TF-Luna"}}
        cfg.setdefault("camera", {})["name"]="OV9281"
        cfg.setdefault("rangefinder", {})["name"]="TF-Luna"
        cfg["camera"].update({"x":vals["FLOW_POS_X"],"y":vals["FLOW_POS_Y"],"z":vals["FLOW_POS_Z"]})
        cfg["rangefinder"].update({"x":vals["RNGFND1_POS_X"],"y":vals["RNGFND1_POS_Y"],"z":vals["RNGFND1_POS_Z"]})
        GEOMETRY_JSON.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")

    def _update_camera_yaml(self, vals):
        text=CAMERA_YAML.read_text(encoding="utf-8")
        x=vals["FLOW_POS_X"]; y=vals["FLOW_POS_Y"]; z=vals["FLOW_POS_Z"]
        # YAML T_BS uses body FLU translation: X forward, Y left, Z up.
        block=(
            "data: [ 0.000000000, -1.000000000,  0.000000000,  {x:.6f},\n"
            "       -1.000000000,  0.000000000,  0.000000000,  {yflu:.6f},\n"
            "        0.000000000,  0.000000000, -1.000000000,  {zflu:.6f},\n"
            "        0.000000000,  0.000000000,  0.000000000,  1.0000 ]"
        ).format(x=x,yflu=-y,zflu=-z)
        text2,n=re.subn(r"data:\s*\[.*?1\.0000\s*\]",block,text,count=1,flags=re.S)
        if n!=1:
            raise RuntimeError("Не удалось обновить T_BS.data в camera YAML")
        CAMERA_YAML.write_text(text2,encoding="utf-8")

    def write_fc(self):
        try:
            vals=self._entered_meters()
        except Exception as e:
            messagebox.showerror("Ошибка ввода", str(e))
            return

        summary=(
            "Будут записаны параметры FC:\n\n"
            f"Камера: X={vals['FLOW_POS_X']*1000:+.1f} мм, "
            f"Y={vals['FLOW_POS_Y']*1000:+.1f} мм, Z={vals['FLOW_POS_Z']*1000:+.1f} мм\n"
            f"TF-Luna: X={vals['RNGFND1_POS_X']*1000:+.1f} мм, "
            f"Y={vals['RNGFND1_POS_Y']*1000:+.1f} мм, Z={vals['RNGFND1_POS_Z']*1000:+.1f} мм\n\n"
            "Аппарат должен быть DISARMED (моторы выключены). Продолжить?"
        )
        if not messagebox.askyesno("Подтверждение записи", summary):
            return

        args=self._cmd_base()+["set"]
        for p in PARAMS.values():
            args += [p, f"{vals[p]:.6f}"]

        try:
            self.status.set("Записываю параметры в FC и проверяю подтверждение...")
            self.update_idletasks()
            self._run(args)

            verify=self._run(self._cmd_base()+["read"]+list(PARAMS.values()))
            got=self._parse_values(verify)
            bad=[]
            for p,v in vals.items():
                if p not in got or not math.isclose(got[p],v,rel_tol=0,abs_tol=max(1e-6,abs(v)*1e-5)):
                    bad.append(p)
            if bad:
                raise RuntimeError("Проверка после записи не прошла: "+", ".join(bad))

            # Только после подтверждения FC обновляем локальную геометрию проекта,
            # чтобы следующий preflight сравнивал FC с теми же значениями.
            self._save_local_geometry(vals)
            self._update_camera_yaml(vals)

            dx=(got["RNGFND1_POS_X"]-got["FLOW_POS_X"])*1000
            dy=(got["RNGFND1_POS_Y"]-got["FLOW_POS_Y"])*1000
            dz=(got["RNGFND1_POS_Z"]-got["FLOW_POS_Z"])*1000
            self.status.set(
                "ГОТОВО: все 6 параметров записаны и повторно прочитаны из FC.\n"
                f"Разнос TF-Luna относительно камеры: ΔX={dx:+.1f} мм, "
                f"ΔY={dy:+.1f} мм, ΔZ={dz:+.1f} мм.\n"
                "Локальный mount_geometry.json и camera YAML тоже обновлены.\n"
                "Перезагрузите FC перед следующим flight-тестом."
            )
            messagebox.showinfo("Запись завершена",
                                "Все 6 параметров записаны и подтверждены FC.\n"
                                "Конфигурация monkeysStab синхронизирована с FC.")
        except Exception as e:
            messagebox.showerror("Ошибка записи", str(e))
            self.status.set("ОШИБКА: параметры не были полностью подтверждены FC.")

if __name__ == "__main__":
    try:
        app=GeometryGui()
        app.mainloop()
    except tk.TclError as e:
        print("ОШИБКА GUI:", e, file=sys.stderr)
        sys.exit(2)
