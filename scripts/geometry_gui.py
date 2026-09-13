#!/usr/bin/env python3
import json
import math
import re
import tkinter as tk
from pathlib import Path
from tkinter import messagebox

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "mount_geometry.json"
CAMERA_YAML = ROOT / "config" / "ov9281_current_mount.yaml"

FONT = ("DejaVu Sans", 11)
FONT_BOLD = ("DejaVu Sans", 11, "bold")
TITLE_FONT = ("DejaVu Sans", 15, "bold")

def load_cfg():
    with CFG.open("r", encoding="utf-8") as f:
        return json.load(f)

def fmt_mm(v_m):
    return f"{v_m * 1000.0:.1f}"

class GeometryApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("monkeysStab — геометрия датчиков")
        self.geometry("820x620")
        self.minsize(760, 560)
        self.configure(padx=18, pady=16)

        cfg = load_cfg()

        tk.Label(self, text="Положение камеры и дальномера", font=TITLE_FONT).pack(anchor="w")
        tk.Label(
            self,
            text="Все размеры вводятся в миллиметрах относительно центра IMU полётного контроллера.",
            font=FONT,
        ).pack(anchor="w", pady=(4, 2))
        tk.Label(
            self,
            text="Оси FRD: X + вперёд, X − назад; Y + вправо, Y − влево; Z + вниз, Z − вверх.",
            font=FONT,
        ).pack(anchor="w", pady=(0, 14))

        body = tk.Frame(self)
        body.pack(fill="x")

        self.vars = {}
        self._sensor_box(body, 0, "camera", "OV9281 — оптический центр камеры", cfg["camera"])
        self._sensor_box(body, 1, "rangefinder", "TF-Luna — оптический центр дальномера", cfg["rangefinder"])

        sep = tk.LabelFrame(self, text="Расстояние между датчиками", font=FONT_BOLD, padx=12, pady=10)
        sep.pack(fill="x", pady=(16, 10))
        self.sep_label = tk.Label(sep, font=FONT, justify="left", anchor="w")
        self.sep_label.pack(fill="x")

        params = tk.LabelFrame(self, text="Какие параметры должны быть в ArduPilot", font=FONT_BOLD, padx=12, pady=10)
        params.pack(fill="x", pady=(0, 10))
        self.params_text = tk.Text(params, height=8, font=("DejaVu Sans Mono", 10), wrap="none")
        self.params_text.pack(fill="x")
        self.params_text.configure(state="disabled")

        note = tk.Label(
            self,
            text=(
                "Сохранение меняет config/mount_geometry.json и положение камеры в "
                "config/ov9281_current_mount.yaml. Параметры FC автоматически НЕ записываются."
            ),
            font=FONT,
            justify="left",
            anchor="w",
            fg="#444444",
        )
        note.pack(fill="x", pady=(2, 12))

        buttons = tk.Frame(self)
        buttons.pack(fill="x")
        tk.Button(buttons, text="СОХРАНИТЬ КОНФИГУРАЦИЮ", font=FONT_BOLD, command=self.save, padx=18, pady=8).pack(side="left")
        tk.Button(buttons, text="ПЕРЕЗАГРУЗИТЬ ЗНАЧЕНИЯ", font=FONT, command=self.reload, padx=12, pady=8).pack(side="left", padx=10)
        tk.Button(buttons, text="ЗАКРЫТЬ", font=FONT, command=self.destroy, padx=12, pady=8).pack(side="right")

        for v in self.vars.values():
            v.trace_add("write", lambda *_: self.refresh_preview())
        self.refresh_preview()

    def _sensor_box(self, parent, col, key, title, values):
        box = tk.LabelFrame(parent, text=title, font=FONT_BOLD, padx=14, pady=12)
        box.grid(row=0, column=col, padx=(0, 10) if col == 0 else (10, 0), sticky="nsew")
        parent.grid_columnconfigure(col, weight=1)
        for r, (axis, hint) in enumerate([
            ("x", "X, мм  (+ вперёд / − назад)"),
            ("y", "Y, мм  (+ вправо / − влево)"),
            ("z", "Z, мм  (+ вниз / − вверх)"),
        ]):
            tk.Label(box, text=hint, font=FONT).grid(row=r, column=0, sticky="w", pady=5)
            var = tk.StringVar(value=fmt_mm(float(values[axis])))
            ent = tk.Entry(box, textvariable=var, font=("DejaVu Sans Mono", 11), width=12, justify="right")
            ent.grid(row=r, column=1, sticky="e", padx=(14, 0), pady=5)
            self.vars[f"{key}_{axis}"] = var

    def values_m(self):
        vals = {}
        for sensor in ("camera", "rangefinder"):
            vals[sensor] = {}
            for axis in ("x", "y", "z"):
                raw = self.vars[f"{sensor}_{axis}"].get().strip().replace(",", ".")
                vals[sensor][axis] = float(raw) / 1000.0
        return vals

    def refresh_preview(self):
        try:
            v = self.values_m()
        except ValueError:
            self.sep_label.config(text="Введите числовые значения во все поля.")
            return

        dx = (v["rangefinder"]["x"] - v["camera"]["x"]) * 1000.0
        dy = (v["rangefinder"]["y"] - v["camera"]["y"]) * 1000.0
        dz = (v["rangefinder"]["z"] - v["camera"]["z"]) * 1000.0
        dxy = math.hypot(dx, dy)
        d3 = math.sqrt(dx*dx + dy*dy + dz*dz)

        self.sep_label.config(text=(
            f"TF-Luna относительно OV9281:  ΔX={dx:+.1f} мм   ΔY={dy:+.1f} мм   ΔZ={dz:+.1f} мм\n"
            f"Разнос по поверхности XY: {dxy:.1f} мм     Полное расстояние: {d3:.1f} мм"
        ))

        c = v["camera"]
        r = v["rangefinder"]
        text = (
            f"FLOW_POS_X   = {c['x']:.4f}\n"
            f"FLOW_POS_Y   = {c['y']:.4f}\n"
            f"FLOW_POS_Z   = {c['z']:.4f}\n\n"
            f"RNGFND1_POS_X = {r['x']:.4f}\n"
            f"RNGFND1_POS_Y = {r['y']:.4f}\n"
            f"RNGFND1_POS_Z = {r['z']:.4f}\n"
        )
        self.params_text.configure(state="normal")
        self.params_text.delete("1.0", "end")
        self.params_text.insert("1.0", text)
        self.params_text.configure(state="disabled")

    def update_camera_yaml(self, camera):
        text = CAMERA_YAML.read_text(encoding="utf-8")
        x, y, z = camera["x"], camera["y"], camera["z"]

        # T_BS uses body FLU translation: [X forward, Y left, Z up].
        new_block = (
            "data: [ 0.000000000, -1.000000000,  0.000000000,  {x:.6f},\n"
            "       -1.000000000,  0.000000000,  0.000000000,  {yflu:.6f},\n"
            "        0.000000000,  0.000000000, -1.000000000,  {zflu:.6f},\n"
            "        0.000000000,  0.000000000,  0.000000000,  1.0000 ]"
        ).format(x=x, yflu=-y, zflu=-z)

        pattern = r"data:\s*\[.*?1\.0000\s*\]"
        text2, n = re.subn(pattern, new_block, text, count=1, flags=re.S)
        if n != 1:
            raise RuntimeError("Не удалось найти T_BS.data в camera YAML")

        text2 = re.sub(
            r"camera focal point FRD \[.*?\] m from FC IMU",
            f"camera focal point FRD [ {x:+.4f}, {y:+.4f}, {z:+.4f} ] m from FC IMU",
            text2,
            count=1,
        )
        CAMERA_YAML.write_text(text2, encoding="utf-8")

    def save(self):
        try:
            v = self.values_m()
        except ValueError:
            messagebox.showerror("Ошибка", "Все координаты должны быть числами.")
            return

        for sensor in ("camera", "rangefinder"):
            for axis in ("x", "y", "z"):
                if abs(v[sensor][axis]) > 1.0:
                    messagebox.showerror("Ошибка", "Координата больше 1000 мм. Проверьте единицы измерения.")
                    return

        old = load_cfg()
        old["camera"].update(v["camera"])
        old["rangefinder"].update(v["rangefinder"])

        CFG.write_text(json.dumps(old, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        try:
            self.update_camera_yaml(v["camera"])
        except Exception as e:
            messagebox.showerror("Ошибка", f"JSON сохранён, но camera YAML обновить не удалось:\n{e}")
            return

        messagebox.showinfo(
            "Сохранено",
            "Геометрия сохранена.\n\n"
            "Теперь внесите показанные FLOW_POS_* и RNGFND1_POS_* в Mission Planner, "
            "нажмите Write Params и перезагрузите FC."
        )

    def reload(self):
        try:
            cfg = load_cfg()
            for sensor in ("camera", "rangefinder"):
                for axis in ("x", "y", "z"):
                    self.vars[f"{sensor}_{axis}"].set(fmt_mm(float(cfg[sensor][axis])))
        except Exception as e:
            messagebox.showerror("Ошибка", str(e))

if __name__ == "__main__":
    GeometryApp().mainloop()
