#!/usr/bin/env python3
import json
import math
import os
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox, ttk

ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),".."))
FC_TOOL=os.path.join(ROOT,"build","fc_param_tool")
PROFILE_JSON=os.path.join(ROOT,"config","fc_profile.json")

# Параметры, которые являются частью текущего production-контура monkeysStab.
PARAMS=[
    "AHRS_EKF_TYPE","EK3_ENABLE",
    "FLOW_TYPE","FLOW_OPTIONS","FLOW_ORIENT_YAW","FLOW_FXSCALER","FLOW_FYSCALER",
    "EK3_FLOW_DELAY","EK3_FLOW_MAX",
    "RNGFND1_TYPE","RNGFND1_ORIENT","RNGFND1_MIN","RNGFND1_MAX",
    "EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_POSZ","EK3_SRC1_VELZ",
    "EK3_SRC1_YAW","EK3_SRC_OPTIONS",
    "COMPASS_ENABLE","COMPASS_USE",
]

# Проверенный fixed-mount профиль monkeysStab.
FIXED={
    "AHRS_EKF_TYPE":3,
    "EK3_ENABLE":1,
    "FLOW_TYPE":5,             # MAVLink
    "FLOW_OPTIONS":0,          # камера НЕ на стабилизированном подвесе
    "FLOW_ORIENT_YAW":0,
    "FLOW_FXSCALER":0,
    "FLOW_FYSCALER":0,
    "EK3_FLOW_DELAY":0,
    "RNGFND1_TYPE":10,         # MAVLink DISTANCE_SENSOR
    "RNGFND1_ORIENT":25,       # вниз
    "RNGFND1_MIN":0.10,
    "RNGFND1_MAX":7.0,
    "EK3_SRC1_POSXY":0,        # None
    "EK3_SRC1_VELXY":5,        # OpticalFlow
    "EK3_SRC1_VELZ":0,         # None
    "EK3_SRC_OPTIONS":0,
}

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("monkeysStab — критические параметры FC")
        # Окно должно помещаться и на 720p/VNC-экране. Раньше фиксированные
        # 790 px по высоте выталкивали нижнюю панель кнопок за границу экрана.
        super().geometry("1040x700")
        super().minsize(900,620)
        self.device=tk.StringVar(value=os.environ.get("MONKEYS_FC","tcp://127.0.0.1:5760"))
        self.baud=tk.StringVar(value=os.environ.get("MONKEYS_FC_BAUD","460800"))
        self.sysid=tk.StringVar(value=os.environ.get("MONKEYS_FC_SYSID","1"))
        self.compid=tk.StringVar(value=os.environ.get("MONKEYS_FC_COMPID","1"))
        self.posz=tk.StringVar(value="RangeFinder")
        self.compass_mode=tk.StringVar(value="Не использовать для yaw")
        self.flow_max=tk.StringVar(value="2.5")
        self.current={}
        self.status=tk.StringVar(value="Чтение параметров FC...")
        self._build()
        self.after(250,self.read_fc)

    def _build(self):
        root=ttk.Frame(self,padding=12); root.pack(fill="both",expand=True)
        ttk.Label(root,text="Критические параметры monkeysStab",
                  font=("DejaVu Sans",16,"bold")).pack(anchor="w")
        ttk.Label(root,text=(
            "GUI настраивает только параметры, необходимые текущему Optical Flow-контуру. "
            "ExternalNav yaw (EK3_SRC1_YAW=6) здесь намеренно недоступен."
        ),wraplength=990).pack(anchor="w",pady=(2,7))

        conn=ttk.LabelFrame(root,text="Подключение к FC",padding=8); conn.pack(fill="x")
        for i in range(8): conn.columnconfigure(i,weight=1)
        vals=[("Порт",self.device),("Baud",self.baud),("SysID",self.sysid),("CompID",self.compid)]
        col=0
        for label,var in vals:
            ttk.Label(conn,text=label).grid(row=0,column=col,sticky="w",padx=5)
            ttk.Entry(conn,textvariable=var,width=14).grid(row=0,column=col+1,sticky="ew",padx=5)
            col+=2

        body=ttk.Frame(root); body.pack(fill="x",pady=7)
        body.columnconfigure(0,weight=1); body.columnconfigure(1,weight=1)

        left=ttk.Frame(body); left.grid(row=0,column=0,sticky="nsew",padx=(0,6))
        right=ttk.Frame(body); right.grid(row=0,column=1,sticky="nsew",padx=(6,0))

        ekf=ttk.LabelFrame(left,text="EKF3 и источники положения",padding=10); ekf.pack(fill="x",pady=(0,10))
        ttk.Label(ekf,text="Горизонтальная позиция: None (0)").grid(row=0,column=0,sticky="w",pady=3)
        ttk.Label(ekf,text="Горизонтальная скорость: OpticalFlow (5)").grid(row=1,column=0,sticky="w",pady=3)
        ttk.Label(ekf,text="Вертикальная скорость: None (0)").grid(row=2,column=0,sticky="w",pady=3)
        ttk.Label(ekf,text="Источник Z:").grid(row=3,column=0,sticky="w",pady=5)
        ttk.Combobox(ekf,textvariable=self.posz,state="readonly",
                     values=["RangeFinder","Baro"],width=20).grid(row=3,column=1,sticky="e",padx=8)

        compass=ttk.LabelFrame(left,text="Компас и yaw (курс)",padding=10); compass.pack(fill="x",pady=(0,10))
        ttk.Label(compass,text="Режим:").grid(row=0,column=0,sticky="w",pady=5)
        ttk.Combobox(compass,textvariable=self.compass_mode,state="readonly",width=31,
                     values=[
                         "Компас используется для yaw",
                         "Компас включён, но не используется для yaw",
                         "Компас полностью отключён",
                     ]).grid(row=0,column=1,sticky="e",padx=8)
        ttk.Label(compass,text=(
            "Для monkeysStab yaw от ExternalNav не используется. "
            "Если выбран компас, одновременно выставляются COMPASS_ENABLE=1, "
            "COMPASS_USE=1 и EK3_SRC1_YAW=1."
        ),wraplength=450,justify="left").grid(row=1,column=0,columnspan=2,sticky="w",pady=(6,0))

        flow=ttk.LabelFrame(right,text="Optical Flow",padding=10); flow.pack(fill="x",pady=(0,10))
        lines=[
            "FLOW_TYPE = 5  (MAVLink)",
            "FLOW_OPTIONS = 0  (жёстко закреплённая камера)",
            "FLOW_ORIENT_YAW = 0",
            "FLOW_FXSCALER = 0",
            "FLOW_FYSCALER = 0",
            "EK3_FLOW_DELAY = 0 мс",
        ]
        for i,t in enumerate(lines): ttk.Label(flow,text=t).grid(row=i,column=0,sticky="w",pady=2)
        ttk.Label(flow,text="EK3_FLOW_MAX, rad/s:").grid(row=len(lines),column=0,sticky="w",pady=5)
        ttk.Entry(flow,textvariable=self.flow_max,width=10).grid(row=len(lines),column=1,sticky="e")

        rng=ttk.LabelFrame(right,text="TF-Luna → MAVLink RangeFinder",padding=10); rng.pack(fill="x",pady=(0,10))
        for i,t in enumerate([
            "RNGFND1_TYPE = 10  (MAVLink)",
            "RNGFND1_ORIENT = 25  (вниз)",
            "RNGFND1_MIN = 0.10 м",
            "RNGFND1_MAX = 7.0 м",
            "Положение RNGFND1_POS_* задаётся отдельным Geometry GUI.",
        ]):
            ttk.Label(rng,text=t).grid(row=i,column=0,sticky="w",pady=2)

        values=ttk.LabelFrame(root,text="Текущие значения FC",padding=8); values.pack(fill="both",expand=True)
        self.text=tk.Text(values,height=8,font=("DejaVu Sans Mono",9),wrap="none")
        self.text.pack(fill="both",expand=True)
        self.text.configure(state="disabled")

        # Кнопки размещаются до расширяемого списка значений, чтобы они всегда
        # оставались видимыми даже при небольшой высоте рабочего стола.
        buttons=ttk.Frame(root)
        buttons.pack(fill="x",pady=(8,0),before=values)
        ttk.Button(buttons,text="ПРОЧИТАТЬ ИЗ FC",command=self.read_fc).pack(side="left",padx=(0,8))
        ttk.Button(buttons,text="ПРИМЕНИТЬ ПРОФИЛЬ",command=self.apply).pack(side="left",padx=(0,8))
        ttk.Button(buttons,text="ГЕОМЕТРИЯ ДАТЧИКОВ",command=self.open_geometry_gui).pack(side="left")
        ttk.Button(buttons,text="ЗАКРЫТЬ",command=self.destroy).pack(side="right")
        ttk.Label(root,textvariable=self.status,wraplength=990).pack(anchor="w",pady=(8,0))

    def base(self):
        return [FC_TOOL,self.device.get().strip(),self.baud.get().strip(),
                self.sysid.get().strip(),self.compid.get().strip()]

    def run(self,args,timeout=30):
        cp=subprocess.run(args,text=True,capture_output=True,timeout=timeout)
        if cp.returncode!=0:
            raise RuntimeError((cp.stderr or cp.stdout or "неизвестная ошибка").strip())
        return cp.stdout

    def parse(self,text):
        out={}
        for line in text.splitlines():
            if "=" not in line or line.startswith("TARGET "): continue
            k,v=line.split("=",1)
            try: out[k.strip()]=float(v.strip().split()[0])
            except ValueError: pass
        return out

    def show(self,vals):
        rows=[]
        for p in PARAMS:
            rows.append(f"{p:<20} = {vals.get(p,'НЕ ПРОЧИТАН')}")
        self.text.configure(state="normal")
        self.text.delete("1.0","end"); self.text.insert("1.0","\n".join(rows))
        self.text.configure(state="disabled")

    def read_fc(self):
        try:
            out=self.run(self.base()+["read"]+PARAMS)
            vals=self.parse(out); self.current=vals; self.show(vals)
            if "EK3_SRC1_POSZ" in vals:
                self.posz.set("RangeFinder" if int(round(vals["EK3_SRC1_POSZ"]))==2 else "Baro")
            ce=int(round(vals.get("COMPASS_ENABLE",0)))
            cu=int(round(vals.get("COMPASS_USE",0)))
            yaw=int(round(vals.get("EK3_SRC1_YAW",0)))
            if ce==1 and cu==1 and yaw==1:
                self.compass_mode.set("Компас используется для yaw")
            elif ce==0:
                self.compass_mode.set("Компас полностью отключён")
            else:
                self.compass_mode.set("Компас включён, но не используется для yaw")
            if "EK3_FLOW_MAX" in vals: self.flow_max.set(f"{vals['EK3_FLOW_MAX']:g}")
            self.status.set("Параметры прочитаны из FC.")
        except Exception as e:
            self.status.set("Ошибка чтения.")
            messagebox.showerror("Ошибка чтения",str(e))

    def desired(self):
        d=dict(FIXED)
        d["EK3_SRC1_POSZ"]=2 if self.posz.get()=="RangeFinder" else 1

        mode=self.compass_mode.get()
        if mode=="Компас используется для yaw":
            d.update({"COMPASS_ENABLE":1,"COMPASS_USE":1,"EK3_SRC1_YAW":1})
        elif mode=="Компас полностью отключён":
            d.update({"COMPASS_ENABLE":0,"COMPASS_USE":0,"EK3_SRC1_YAW":0})
        else:
            d.update({"COMPASS_ENABLE":1,"COMPASS_USE":0,"EK3_SRC1_YAW":0})

        raw=self.flow_max.get().strip().replace(",",".")
        v=float(raw)
        if not math.isfinite(v) or v<=0 or v>10:
            raise ValueError("EK3_FLOW_MAX должен быть в разумном диапазоне 0..10 rad/s")
        d["EK3_FLOW_MAX"]=v
        return d

    def apply(self):
        try:
            d=self.desired()
        except Exception as e:
            messagebox.showerror("Ошибка ввода",str(e)); return

        lines=[
            "Будут записаны критические параметры monkeysStab:",
            "",
            f"Z: {'RangeFinder' if d['EK3_SRC1_POSZ']==2 else 'Baro'}",
            f"Yaw: {'Compass' if d['EK3_SRC1_YAW']==1 else 'None'}",
            f"Compass enable/use: {d['COMPASS_ENABLE']}/{d['COMPASS_USE']}",
            "VELXY: OpticalFlow (5)",
            "POSXY: None (0)",
            f"RangeFinder: MAVLink, Down, {d['RNGFND1_MIN']:.2f}..{d['RNGFND1_MAX']:.1f} м",
            f"EK3_FLOW_MAX: {d['EK3_FLOW_MAX']:g} rad/s",
            "",
            "ExternalNav yaw НЕ будет включён.",
            "Аппарат должен быть DISARMED. Продолжить?",
        ]
        if not messagebox.askyesno("Подтверждение","\n".join(lines)): return

        args=self.base()+["set"]
        for p,v in d.items(): args += [p,f"{v:.6f}"]
        try:
            self.status.set("Записываю параметры и проверяю подтверждение FC...")
            self.update_idletasks()
            self.run(args,45)
            out=self.run(self.base()+["read"]+list(d.keys()),30)
            got=self.parse(out)
            bad=[]
            for p,v in d.items():
                if p not in got or not math.isclose(got[p],v,rel_tol=0,abs_tol=max(1e-6,abs(v)*1e-5)):
                    bad.append(f"{p}: got={got.get(p)} expected={v}")
            if bad: raise RuntimeError("Не подтверждены:\n"+"\n".join(bad))
            self.current.update(got); self.show(self.current)
            with open(PROFILE_JSON,"w",encoding="utf-8") as fh:
                json.dump({
                    "profile":"fixed_mount_optical_flow",
                    "description":"Текущий проверенный профиль monkeysStab без гироподвеса",
                    "params":d
                },fh,ensure_ascii=False,indent=2)
                fh.write("\n")
            self.status.set("ГОТОВО: параметры записаны, подтверждены FC и сохранены в config/fc_profile.json. Перезагрузите FC.")
            messagebox.showinfo("Готово","Параметры записаны и подтверждены.\nПрофиль monkeysStab синхронизирован с FC.\nПеред тестом перезагрузите FC.")
        except Exception as e:
            self.status.set("Ошибка записи.")
            messagebox.showerror("Ошибка записи",str(e))

    def open_geometry_gui(self):
        subprocess.Popen(["bash",os.path.join(ROOT,"scripts","geometry_gui.sh")])

if __name__=="__main__":
    try:
        App().mainloop()
    except tk.TclError as e:
        print("ОШИБКА GUI:",e,file=sys.stderr); sys.exit(2)
