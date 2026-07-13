import tkinter as tk
from tkinter import ttk, messagebox
import threading, math, csv, os
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")

try:
    import serial, serial.tools.list_ports
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False

from robot3dWidget import *
from config import *


# ─── Main App ────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arduino GPS + IMU Tracker")
        self.configure(bg=BG); self.geometry("1560x860"); self.minsize(1200,700)
        self.serial_port=None; self.running=False; self.connected=False
        self.lat=tk.DoubleVar(value=0.0); self.lng=tk.DoubleVar(value=0.0)
        self.roll=tk.DoubleVar(value=0.0); self.pitch=tk.DoubleVar(value=0.0)
        self.yaw=tk.DoubleVar(value=0.0)
        self.status_txt=tk.StringVar(value="Disconnected")

        # ── Recording state ──────────────────
        self._recording = False
        self._csv_file = None
        self._csv_writer = None
        self._rec_path = ""
        self._rec_rows = 0
        self._rec_start = None
        self._blink_state = False

        self._build(); self._refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ── Layout ──────────────────────────────
    def _build(self):
        self._topbar()
        main=tk.Frame(self,bg=BG); main.pack(fill="both",expand=True,padx=8,pady=(0,8))
        main.columnconfigure(0,weight=3)
        main.columnconfigure(1,weight=2,minsize=300)
        main.columnconfigure(2,weight=1,minsize=310)
        main.rowconfigure(0,weight=1)
        self._map_panel(main)
        self._robot3d_panel(main)
        self._sidebar(main)

    def _topbar(self):
        bar=tk.Frame(self,bg=PANEL,height=52); bar.pack(fill="x",padx=8,pady=8); bar.pack_propagate(False)
        tk.Label(bar,text="⬡ GPS·IMU TRACKER",bg=PANEL,fg=ACCENT,font=("Consolas",15,"bold")).pack(side="left",padx=16)
        tk.Label(bar,text="│",bg=PANEL,fg=BORDER,font=("Consolas",20)).pack(side="left")
        tk.Label(bar,text="PORT",bg=PANEL,fg=MUTED,font=FSM).pack(side="left",padx=(12,4))
        self.port_var=tk.StringVar()
        self.port_cb=ttk.Combobox(bar,textvariable=self.port_var,width=14,state="readonly",font=FSM)
        self.port_cb.pack(side="left",padx=(0,8))
        tk.Label(bar,text="BAUD",bg=PANEL,fg=MUTED,font=FSM).pack(side="left",padx=(0,4))
        self.baud_var=tk.StringVar(value="115200")
        ttk.Combobox(bar,textvariable=self.baud_var,values=["9600","19200","38400","57600","115200"],
                     width=8,state="readonly",font=FSM).pack(side="left",padx=(0,12))
        self._btn(bar,"↺",self._refresh_ports,fg=MUTED).pack(side="left",padx=(0,8))
        self.cbtn=self._btn(bar,"CONNECT",self._toggle_connect,fg=GREEN,font=("Consolas",10,"bold"))
        self.cbtn.pack(side="left",padx=(0,16))
        tk.Label(bar,text="│",bg=PANEL,fg=BORDER,font=("Consolas",20)).pack(side="left")
        self.dot=tk.Label(bar,text="●",bg=PANEL,fg=RED,font=("Consolas",14)); self.dot.pack(side="left",padx=(12,4))
        tk.Label(bar,textvariable=self.status_txt,bg=PANEL,fg=TEXT,font=FM).pack(side="left")

        # ── Right side: Demo + Record controls ──
        self._btn(bar,"▶ DEMO",self._demo_start,fg=YELLOW).pack(side="right",padx=(8,16))

        # Recording row (right-anchored, built right-to-left)
        tk.Label(bar,text="│",bg=PANEL,fg=BORDER,font=("Consolas",20)).pack(side="right",padx=4)

        # Rec info label (rows + elapsed)
        self.rec_info_lbl = tk.Label(bar, text="", bg=PANEL, fg=MUTED, font=FSM)
        self.rec_info_lbl.pack(side="right", padx=(0,8))

        # Blinking dot
        self.rec_dot = tk.Label(bar, text="●", bg=PANEL, fg=PANEL, font=("Consolas",14))
        self.rec_dot.pack(side="right", padx=(0,2))

        # Record / Stop button
        self.rec_btn = self._btn(bar, "⏺ RECORD", self._toggle_record, fg=RED, font=("Consolas",10,"bold"))
        self.rec_btn.pack(side="right", padx=(0,4))

    def _map_panel(self, parent):
        frame=tk.Frame(parent,bg=BORDER); frame.grid(row=0,column=0,sticky="nsew",padx=(0,6))
        hdr=tk.Frame(frame,bg=PANEL); hdr.pack(fill="x")
        tk.Label(hdr,text="MAP  (drag=pan · scroll=zoom)",bg=PANEL,fg=MUTED,font=FSM,padx=10,pady=6).pack(side="left")
        self.gps_lbl=tk.Label(hdr,text="GPS: NO FIX",bg=PANEL,fg=RED,font=FSM,padx=10); self.gps_lbl.pack(side="right")
        if PIL_OK:
            self.map=MapWidget(frame); self.map.pack(fill="both",expand=True)
        else:
            tk.Label(frame,text="Install Pillow:\npip install Pillow requests",
                     bg="#0a1628",fg=MUTED,font=("Consolas",13)).pack(expand=True)
            self.map=None
        ctrl=tk.Frame(frame,bg=PANEL); ctrl.pack(fill="x")
        self._btn(ctrl,"＋",lambda:self.map and self.map.zoom_in(),padx=10,pady=4).pack(side="left",padx=(8,2),pady=3)
        self._btn(ctrl,"－",lambda:self.map and self.map.zoom_out(),padx=10,pady=4).pack(side="left",padx=2,pady=3)
        self._btn(ctrl,"⊙ CENTER",lambda:self.map and self.map.recenter(),fg=MUTED,padx=8,pady=4).pack(side="left",padx=2,pady=3)
        self._btn(ctrl,"✕ TRAIL",self._clear_trail,fg=MUTED,padx=8,pady=4).pack(side="left",padx=2,pady=3)
        self.trail_lbl=tk.Label(ctrl,text="TRAIL: 0 pts",bg=PANEL,fg=MUTED,font=FSM); self.trail_lbl.pack(side="right",padx=12)

    def _robot3d_panel(self, parent):
        frame = tk.Frame(parent, bg=BORDER)
        frame.grid(row=0, column=1, sticky="nsew", padx=(0,6))
        hdr = tk.Frame(frame, bg=PANEL); hdr.pack(fill="x")
        tk.Label(hdr, text="3D ROBOT  (IMU ATTITUDE)", bg=PANEL, fg=MUTED, font=FSM, padx=10, pady=6).pack(side="left")
        self.imu_status_lbl = tk.Label(hdr, text="IMU: --", bg=PANEL, fg=MUTED, font=FSM, padx=10)
        self.imu_status_lbl.pack(side="right")
        self.robot3d = Robot3DWidget(frame)
        self.robot3d.pack(fill="both", expand=True)
        bottom = tk.Frame(frame, bg=PANEL); bottom.pack(fill="x")
        bottom.columnconfigure(0,weight=1); bottom.columnconfigure(1,weight=1); bottom.columnconfigure(2,weight=1)
        for col, (label, var, color) in enumerate([
            ("ROLL",  self.roll,  ACCENT),
            ("PITCH", self.pitch, GREEN),
            ("YAW",   self.yaw,   YELLOW),
        ]):
            f = tk.Frame(bottom, bg=PANEL); f.grid(row=0, column=col, padx=8, pady=6, sticky="ew")
            tk.Label(f, text=label, bg=PANEL, fg=MUTED, font=FSM).pack()
            lbl = tk.Label(f, text="+0.0°", bg=PANEL, fg=color, font=("Consolas", 13, "bold"))
            lbl.pack()
            def make_fmt(v=var, l=lbl):
                def fmt(*_):
                    try: l.config(text=f"{v.get():+.1f}°")
                    except: pass
                return fmt
            var.trace_add("write", make_fmt())

    def _sidebar(self, parent):
        side=tk.Frame(parent,bg=BG); side.grid(row=0,column=2,sticky="nsew")
        gcard=self._card(side,"GPS COORDINATES"); gcard.pack(fill="x",pady=(0,6))
        gf=tk.Frame(gcard,bg=CARD); gf.pack(fill="x",padx=8,pady=8)
        gf.columnconfigure(0,weight=1); gf.columnconfigure(1,weight=1)
        self._val_lbl(gf,"LATITUDE", self.lat, 6,"°",0,0)
        self._val_lbl(gf,"LONGITUDE",self.lng, 6,"°",0,1)
        acard=self._card(side,"ATTITUDE INDICATOR"); acard.pack(fill="x",pady=(0,6))
        ai=tk.Frame(acard,bg=CARD); ai.pack(fill="x",padx=8,pady=6)
        self.ai=AI(ai,size=180); self.ai.pack()
        rp=tk.Frame(ai,bg=CARD); rp.pack(fill="x",pady=(4,0))
        rp.columnconfigure(0,weight=1); rp.columnconfigure(1,weight=1)
        self._mini(rp,"ROLL", self.roll, 0,0); self._mini(rp,"PITCH",self.pitch,0,1)
        ccard=self._card(side,"HEADING / YAW"); ccard.pack(fill="x",pady=(0,6))
        ci=tk.Frame(ccard,bg=CARD); ci.pack(fill="x",padx=8,pady=6)
        row=tk.Frame(ci,bg=CARD); row.pack()
        self.compass=Compass(row,size=130); self.compass.pack(side="left",padx=(0,12))
        yf=tk.Frame(row,bg=CARD); yf.pack(side="left",anchor="center")
        tk.Label(yf,text="YAW",bg=CARD,fg=MUTED,font=FSM).pack()
        ylbl=tk.Label(yf,text="0.0",bg=CARD,fg=ACCENT,font=("Consolas",22,"bold")); ylbl.pack()
        def fy(*_):
            try: ylbl.config(text=f"{self.yaw.get():.1f}")
            except: pass
        self.yaw.trace_add("write",fy)
        tk.Label(yf,text="degrees",bg=CARD,fg=MUTED,font=FSM).pack()
        lcard=self._card(side,"DATA LOG"); lcard.pack(fill="both",expand=True)
        self.log=tk.Text(lcard,bg=CARD,fg=GREEN,font=("Consolas",8),state="disabled",relief="flat",highlightthickness=0,height=7)
        self.log.pack(fill="both",expand=True,padx=6,pady=6)

    # ── Widget Helpers ───────────────────────
    def _card(self, parent, title):
        outer=tk.Frame(parent,bg=BORDER); inner=tk.Frame(outer,bg=CARD)
        inner.pack(fill="both",expand=True,padx=1,pady=1)
        tk.Label(inner,text=title,bg=CARD,fg=MUTED,font=FSM,padx=10,pady=6).pack(anchor="w")
        tk.Frame(inner,bg=BORDER,height=1).pack(fill="x")
        return inner

    def _btn(self, parent, text, cmd, fg=TEXT, font=FSM, padx=8, pady=4):
        b=tk.Label(parent,text=text,bg=PANEL,fg=fg,font=font,padx=padx,pady=pady,cursor="hand2")
        b.bind("<Button-1>",lambda e:cmd())
        b.bind("<Enter>",lambda e:b.config(bg=BORDER))
        b.bind("<Leave>",lambda e:b.config(bg=PANEL))
        return b

    def _val_lbl(self, parent, title, var, dec, unit, row, col):
        f=tk.Frame(parent,bg=CARD); f.grid(row=row,column=col,sticky="nsew",padx=6,pady=4)
        tk.Label(f,text=title,bg=CARD,fg=MUTED,font=FSM).pack(anchor="w")
        lbl=tk.Label(f,text=f"0.{'0'*dec}{unit}",bg=CARD,fg=TEXT,font=("Consolas",11,"bold")); lbl.pack(anchor="w")
        def fmt(*_):
            try: lbl.config(text=f"{var.get():.{dec}f}{unit}")
            except: pass
        var.trace_add("write",fmt)

    def _mini(self, parent, title, var, row, col):
        f=tk.Frame(parent,bg=CARD); f.grid(row=row,column=col,sticky="nsew",padx=6,pady=4)
        tk.Label(f,text=title,bg=CARD,fg=MUTED,font=FSM).pack()
        lbl=tk.Label(f,text="+0.0°",bg=CARD,fg=ACCENT,font=("Consolas",13,"bold")); lbl.pack()
        def fmt(*_):
            try: lbl.config(text=f"{var.get():+.1f}°")
            except: pass
        var.trace_add("write",fmt)

    # ── CSV Recording ────────────────────────
    def _toggle_record(self):
        if self._recording:
            self._stop_record()
        else:
            self._start_record()

    def _start_record(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"gps_imu_{ts}.csv"
        # Save next to the script by default
        self._rec_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), fname)
        try:
            self._csv_file = open(self._rec_path, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            # Header row
            self._csv_writer.writerow([
                "timestamp", "elapsed_s",
                "latitude", "longitude",
                "roll_deg", "pitch_deg", "yaw_deg"
            ])
            self._csv_file.flush()
        except Exception as e:
            messagebox.showerror("Record Error", str(e))
            return

        self._recording = True
        self._rec_rows = 0
        self._rec_start = datetime.now()
        self.rec_btn.config(text="⏹ STOP", fg=YELLOW)
        self._log(f"[{self._ts()}] ● Recording → {fname}")
        self._blink()            # start blink loop
        self._update_rec_info()  # start info refresh loop

    def _stop_record(self):
        self._recording = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self.rec_btn.config(text="⏺ RECORD", fg=RED)
        self.rec_dot.config(fg=PANEL)   # hide dot
        elapsed = (datetime.now() - self._rec_start).total_seconds() if self._rec_start else 0
        self.rec_info_lbl.config(
            text=f"Saved {self._rec_rows} rows  ({elapsed:.0f}s)",
            fg=GREEN
        )
        self._log(f"[{self._ts()}] ■ Stopped — {self._rec_rows} rows saved → {os.path.basename(self._rec_path)}")
        # Clear info after 6 s
        self.after(6000, lambda: self.rec_info_lbl.config(text="", fg=MUTED))

    def _write_csv_row(self, lat, lng, roll, pitch, yaw):
        """Called from _update() while recording is active."""
        if not self._recording or self._csv_writer is None or self._csv_file is None or self._rec_start is None:
            return
        now = datetime.now()
        elapsed = (now - self._rec_start).total_seconds()
        try:
            self._csv_writer.writerow([
                now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                f"{elapsed:.3f}",
                f"{lat:.7f}", f"{lng:.7f}",
                f"{roll:.3f}", f"{pitch:.3f}", f"{yaw:.3f}",
            ])
            self._csv_file.flush()   # ensure data is written immediately
            self._rec_rows += 1
        except Exception as e:
            self._log(f"[REC ERR] {e}")

    def _blink(self):
        """Toggle the red recording dot every 600 ms while recording."""
        if not self._recording:
            return
        if not self.winfo_exists():
            return
        self._blink_state = not self._blink_state
        self.rec_dot.config(fg=RED if self._blink_state else PANEL)
        self.after(600, self._blink)

    def _update_rec_info(self):
        """Refresh elapsed time + row count label every second while recording."""
        if not self._recording:
            return
        if not self.winfo_exists():
            return
        if self._rec_start is None:
            return
        elapsed = (datetime.now() - self._rec_start).total_seconds()
        m = int(elapsed) // 60; s = int(elapsed) % 60
        self.rec_info_lbl.config(
            text=f"{m:02d}:{s:02d}  {self._rec_rows} rows",
            fg=RED
        )
        self.after(1000, self._update_rec_info)

    # ── Serial ───────────────────────────────
    def _refresh_ports(self):
        if not SERIAL_OK: self.port_cb["values"]=["pip install pyserial"]; return
        ports=[p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"]=ports or ["No ports found"]
        if ports: self.port_var.set(ports[0])

    def _toggle_connect(self):
        self._disconnect() if self.connected else self._connect()

    def _connect(self):
        if not SERIAL_OK: messagebox.showerror("Error","pip install pyserial"); return
        port=self.port_var.get(); baud=int(self.baud_var.get())
        try:
            self.serial_port=serial.Serial(port,baud,timeout=1)
            self.connected=True; self.running=True
            threading.Thread(target=self._read_loop,daemon=True).start()
            self._setstatus(f"Connected — {port}",GREEN)
            self.cbtn.config(text="DISCONNECT",fg=RED)
            self._log(f"[{self._ts()}] Connected {port} @ {baud}")
        except Exception as e: messagebox.showerror("Error",str(e))

    def _disconnect(self):
        self.running=False
        if self.serial_port and self.serial_port.is_open: self.serial_port.close()
        self.connected=False; self._setstatus("Disconnected",RED)
        self.cbtn.config(text="CONNECT",fg=GREEN)
        self._log(f"[{self._ts()}] Disconnected")

    def _read_loop(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                line=self.serial_port.readline().decode("utf-8",errors="ignore").strip()
                if line: self.after(0,self._parse,line)
            except Exception as e: self.after(0,self._log,f"[ERR] {e}"); break

    def _parse(self, line):
        try:
            parts=[p.strip() for p in line.split(",")]
            if len(parts)!=5: return
            lat,lng,roll,pitch,yaw=map(float,parts)
            self._update(lat,lng,roll,pitch,yaw)
            self._log(f"[{self._ts()}] {line}")
        except ValueError: self._log(f"[{self._ts()}] BAD: {line}")

    # ── Data Update ──────────────────────────
    def _update(self, lat, lng, roll, pitch, yaw):
        self.roll.set(round(roll,2)); self.pitch.set(round(pitch,2)); self.yaw.set(round(yaw%360,2))
        self.ai.update(roll,pitch); self.compass.update(yaw)
        self.robot3d.update(roll, pitch, yaw)
        self.imu_status_lbl.config(
            text=f"R:{roll:+.1f}° P:{pitch:+.1f}° Y:{yaw:.1f}°", fg=GREEN)
        valid=not(lat==0.0 and lng==0.0)
        if valid:
            self.lat.set(round(lat,6)); self.lng.set(round(lng,6))
            self.gps_lbl.config(text=f"GPS: FIX  {lat:.5f}, {lng:.5f}",fg=GREEN)
            if self.map:
                self.map.add_trail(lat,lng)
                self.map.set_position(lat,lng)
                self.trail_lbl.config(text=f"TRAIL: {len(self.map._trail)} pts")
        else:
            self.gps_lbl.config(text="GPS: NO FIX",fg=RED)

        # ── Write to CSV if recording ─────────
        self._write_csv_row(lat, lng, roll, pitch, yaw)

    def _clear_trail(self):
        if self.map: self.map.clear_trail()
        self.trail_lbl.config(text="TRAIL: 0 pts")

    # ── Demo ─────────────────────────────────
    def _demo_start(self):
        self._setstatus("Demo Mode",YELLOW); self._log(f"[{self._ts()}] Demo started"); self._demo(0)

    def _demo(self, i):
        if not self.winfo_exists(): return
        clat,clng=1.1307,104.0531; rad=0.0008; a=i*2
        lat=clat+rad*math.sin(math.radians(a)); lng=clng+rad*math.cos(math.radians(a))
        roll=20*math.sin(math.radians(a*2)); pitch=12*math.cos(math.radians(a*1.5)); yaw=a%360
        self._update(lat,lng,roll,pitch,yaw)
        self.after(120,self._demo,i+1)

    # ── Utils ────────────────────────────────
    def _setstatus(self, t, c): self.status_txt.set(t); self.dot.config(fg=c)

    def _log(self, msg):
        self.log.config(state="normal"); self.log.insert("end",msg+"\n"); self.log.see("end")
        lines=int(self.log.index("end-1c").split(".")[0])
        if lines>500: self.log.delete("1.0",f"{lines-500}.0")
        self.log.config(state="disabled")

    def _ts(self): return datetime.now().strftime("%H:%M:%S")

    def _close(self):
        if self._recording:
            self._stop_record()
        self.running=False
        if self.serial_port and self.serial_port.is_open: self.serial_port.close()
        self.destroy()


if __name__=="__main__":
    missing=[]
    if not PIL_OK: missing.append("Pillow")
    if not SERIAL_OK: missing.append("pyserial")
    if missing: print(f"Run: pip install {' '.join(missing)} requests")
    try:
        App().mainloop()
    except KeyboardInterrupt:
        pass