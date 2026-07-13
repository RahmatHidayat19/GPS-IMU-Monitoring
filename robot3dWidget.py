import math
import tkinter as tk
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from config import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageTk, ImageDraw
from additional_func import ll_to_tile, lng_to_px, lat_to_py, fetch_tile
from collections import deque

try:
    from PIL import Image, ImageTk, ImageDraw
    PIL_OK = True
except ImportError:
    PIL_OK = False

# ─── 3D Robot Visualizer ─────────────────────
class Robot3DWidget(tk.Frame):
    """Renders a 3D box-body robot that responds to roll/pitch/yaw."""

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=CARD, **kw)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._pending = False

        self.fig = Figure(figsize=(3.2, 3.2), dpi=90, facecolor="#1c2128")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._style_axes()

        self.canvas_widget = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget.get_tk_widget().pack(fill="both", expand=True)

        self._draw()

    def _style_axes(self):
        ax = self.ax
        ax.set_facecolor("#0d1117")
        ax.set_xlim(-2, 2); ax.set_ylim(-2, 2); ax.set_zlim(-2, 2)
        ax.set_box_aspect([1,1,1])
        ax.set_axis_off()
        for axis_name in ('xaxis', 'yaxis', 'zaxis'):
            pane = getattr(getattr(ax, axis_name), 'pane', None)
            if pane is not None:
                pane.fill = False
                pane.set_edgecolor('none')
        ax.grid(False)
        self.fig.tight_layout(pad=0)

    def update(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        if not self._pending:
            self._pending = True
            self.after(50, self._deferred_draw)

    def _deferred_draw(self):
        self._pending = False
        self._draw()

    def _rotation_matrix(self, roll_deg, pitch_deg, yaw_deg):
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
        Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
        Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
        return Rz @ Ry @ Rx

    def _make_box(self, cx, cy, cz, lx, ly, lz, R):
        dx, dy, dz = lx/2, ly/2, lz/2
        corners = np.array([
            [-dx,-dy,-dz],[dx,-dy,-dz],[dx,dy,-dz],[-dx,dy,-dz],
            [-dx,-dy, dz],[dx,-dy, dz],[dx,dy, dz],[-dx,dy, dz],
        ])
        rotated = (R @ corners.T).T + np.array([cx,cy,cz])
        idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[1,2,6,5],[0,3,7,4]]
        return [[rotated[i] for i in face] for face in idx]

    def _draw(self):
        ax = self.ax
        ax.cla()
        self._style_axes()
        R = self._rotation_matrix(self.roll, self.pitch, self.yaw)
        body_faces = self._make_box(0, 0, 0, 1.2, 0.7, 0.4, R)
        body_col = Poly3DCollection(body_faces, alpha=0.85, facecolor="#1a3a5c", edgecolor="#58a6ff", linewidth=0.8)
        ax.add_collection3d(body_col)
        ant_base = np.array([0.0, 0.0, 0.52]); ant_tip = np.array([0.0, 0.0, 1.1])
        ab_r = R @ ant_base; at_r = R @ ant_tip
        ax.plot([ab_r[0],at_r[0]],[ab_r[1],at_r[1]],[ab_r[2],at_r[2]],color="#d29922",linewidth=1.5,zorder=10)
        ax.scatter(*at_r, color="#d29922", s=18, zorder=11)
        fwd = R @ np.array([1.5, 0, 0])
        ax.quiver(0,0,0,fwd[0],fwd[1],fwd[2],length=0.7,color="#f85149",linewidth=1.5,arrow_length_ratio=0.35,normalize=False)
        theta = np.linspace(0, 2*math.pi, 40)
        ax.plot(0.9*np.cos(theta),0.9*np.sin(theta),np.full(40,-1.5),color="#21262d",linewidth=1.5,alpha=0.5)
        for v in np.linspace(-1.8, 1.8, 7):
            ax.plot([-1.8,1.8],[v,v],[-1.5,-1.5],color="#21262d",linewidth=0.5,alpha=0.4)
            ax.plot([v,v],[-1.8,1.8],[-1.5,-1.5],color="#21262d",linewidth=0.5,alpha=0.4)
        ax.text2D(0.5,0.97,f"ROLL {self.roll:+.1f}°  PITCH {self.pitch:+.1f}°  YAW {self.yaw:.1f}°",
                  transform=ax.transAxes,ha="center",va="top",color="#8b949e",fontsize=7,fontfamily="monospace")
        ax.view_init(elev=22, azim=45)
        self.canvas_widget.draw()


# ─── Attitude Indicator ───────────────────────
class AI(tk.Canvas):
    def __init__(self, parent, size=180, **kw):
        super().__init__(parent,width=size,height=size,bg=BG,highlightthickness=0,**kw)
        self.cx=size//2; self.cy=size//2; self.r=size//2-6
        self.roll=self.pitch=0.0; self._draw()

    def update(self, roll, pitch): self.roll=roll; self.pitch=pitch; self._draw()

    def _draw(self):
        self.delete("all")
        cx,cy,r=self.cx,self.cy,self.r
        pp=r/45; ppx=max(-r,min(r,self.pitch*pp))
        rr=math.radians(self.roll); cos_r,sin_r=math.cos(rr),math.sin(rr)
        def rot(dx,dy): return (cx+dx*cos_r-dy*sin_r, cy+dx*sin_r+dy*cos_r)
        self.create_oval(cx-r,cy-r,cx+r,cy+r,fill="#0a1628",outline=BORDER,width=2)
        W=r*2.5
        for half,color in [("sky","#1a3a5c"),("earth","#5c3d1a")]:
            pts=[]; hx1,hy1=rot(-W,ppx); hx2,hy2=rot(W,ppx)
            pts.append((hx1,hy1)); steps=36
            sign=-1 if half=="sky" else 1
            for i in range(steps+1):
                a=math.pi*i/steps
                pts.append((cx+r*math.cos(math.pi+a*sign),cy+r*math.sin(math.pi+a*sign)))
            pts.append((hx2,hy2)); flat=[v for p in pts for v in p]
            if len(flat)>=6: self.create_polygon(flat,fill=color,outline="",smooth=False)
        x1,y1=rot(-r*1.5,ppx); x2,y2=rot(r*1.5,ppx)
        self.create_line(x1,y1,x2,y2,fill="white",width=2)
        for deg in [-20,-10,10,20]:
            py=ppx-deg*pp; w2=r*0.3 if abs(deg)==10 else r*0.5
            lx1,ly1=rot(-w2,py); lx2,ly2=rot(w2,py)
            mx=(lx1+lx2)/2; my=(ly1+ly2)/2
            if (mx-cx)**2+(my-cy)**2<r**2:
                self.create_line(lx1,ly1,lx2,ly2,fill="white",width=1)
        for ang in [-60,-45,-30,-20,-10,0,10,20,30,45,60]:
            a=math.radians(ang-90)+rr; ln=10 if ang%30==0 else 5
            self.create_line(cx+(r-3)*math.cos(a),cy+(r-3)*math.sin(a),
                             cx+(r-3-ln)*math.cos(a),cy+(r-3-ln)*math.sin(a),fill="white",width=1)
        self.create_line(cx-30,cy,cx-10,cy,fill=YELLOW,width=3,capstyle="round")
        self.create_line(cx+10,cy,cx+30,cy,fill=YELLOW,width=3,capstyle="round")
        self.create_oval(cx-4,cy-4,cx+4,cy+4,fill=YELLOW,outline="")
        self.create_oval(cx-r,cy-r,cx+r,cy+r,outline=ACCENT,width=2,fill="")
        ta=math.radians(-90)+rr; tr=r+3
        tx=cx+tr*math.cos(ta); ty=cy+tr*math.sin(ta)
        a1=ta+math.pi*5/6; a2=ta-math.pi*5/6
        self.create_polygon(tx,ty,tx+6*math.cos(a1),ty+6*math.sin(a1),
                            tx+6*math.cos(a2),ty+6*math.sin(a2),fill=ACCENT,outline="")


# ─── Compass ─────────────────────────────────
class Compass(tk.Canvas):
    def __init__(self, parent, size=130, **kw):
        super().__init__(parent,width=size,height=size,bg=BG,highlightthickness=0,**kw)
        self.cx=size//2; self.cy=size//2; self.r=size//2-6; self.yaw=0.0; self._draw()

    def update(self, yaw): self.yaw=yaw%360; self._draw()

    def _draw(self):
        self.delete("all")
        cx,cy,r=self.cx,self.cy,self.r
        self.create_oval(cx-r,cy-r,cx+r,cy+r,fill=CARD,outline=ACCENT,width=2)
        for lbl,deg in [("N",0),("E",90),("S",180),("W",270)]:
            a=math.radians(deg-self.yaw-90)
            lx=cx+(r-14)*math.cos(a); ly=cy+(r-14)*math.sin(a)
            self.create_text(lx,ly,text=lbl,fill=RED if lbl=="N" else TEXT,font=("Consolas",9,"bold"))
        for deg in range(0,360,10):
            a=math.radians(deg-self.yaw-90); ln=10 if deg%30==0 else 5
            self.create_line(cx+(r-3)*math.cos(a),cy+(r-3)*math.sin(a),
                             cx+(r-3-ln)*math.cos(a),cy+(r-3-ln)*math.sin(a),fill=MUTED,width=1)
        nr=r-20
        self.create_line(cx,cy,cx+nr*math.cos(math.radians(-90)),cy+nr*math.sin(math.radians(-90)),fill=RED,width=3,capstyle="round")
        self.create_line(cx,cy,cx+nr*math.cos(math.radians(90)),cy+nr*math.sin(math.radians(90)),fill=TEXT,width=2,capstyle="round")
        self.create_oval(cx-5,cy-5,cx+5,cy+5,fill=ACCENT,outline="")
        self.create_text(cx,cy+r-8,text=f"{self.yaw:05.1f}°",fill=ACCENT,font=FSM)



class MapWidget(tk.Frame):
    ZMIN, ZMAX = 2, 19

    def __init__(self, parent, **kw):
        super().__init__(parent, bg=BG, **kw)
        self._lat = 0.0; self._lng = 0.0; self._zoom = 15
        self._trail = deque(maxlen=400)
        self._offset = [0,0]
        self._drag_xy = (0,0)
        self._tkimgs = []

        self.canvas = tk.Canvas(self, bg="#0a1628", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>",       lambda e: self._render())
        self.canvas.bind("<ButtonPress-1>",   self._ds)
        self.canvas.bind("<B1-Motion>",       self._dm)
        self.canvas.bind("<MouseWheel>",      self._scroll)
        self.canvas.bind("<Button-4>",        self._scroll)
        self.canvas.bind("<Button-5>",        self._scroll)

    def set_position(self, lat, lng, recenter=False):
        self._lat=lat; self._lng=lng
        if recenter: self._offset=[0,0]
        self.after(0, self._render)

    def add_trail(self, lat, lng): self._trail.append((lat,lng))
    def clear_trail(self): self._trail.clear(); self.after(0,self._render)
    def zoom_in(self):  self._zoom=min(self.ZMAX,self._zoom+1); self.after(0,self._render)
    def zoom_out(self): self._zoom=max(self.ZMIN,self._zoom-1); self.after(0,self._render)
    def recenter(self): self._offset=[0,0]; self.after(0,self._render)

    def _ds(self, e): self._drag_xy=(e.x,e.y)
    def _dm(self, e):
        self._offset[0]+=e.x-self._drag_xy[0]
        self._offset[1]+=e.y-self._drag_xy[1]
        self._drag_xy=(e.x,e.y)
        self.after(0,self._render)
    def _scroll(self, e):
        d = getattr(e,"delta",0)
        if d>0 or e.num==4: self.zoom_in()
        else: self.zoom_out()

    def _render(self):
        if not PIL_OK: return
        w=self.canvas.winfo_width(); h=self.canvas.winfo_height()
        if w<=1 or h<=1: return
        z=self._zoom; lat=self._lat; lng=self._lng
        ctx,cty = ll_to_tile(lat,lng,z)
        cx_px = lng_to_px(lng, ctx*TILE_SIZE, z)
        cy_py = lat_to_py(lat, cty*TILE_SIZE, z)
        gcx = ctx*TILE_SIZE+cx_px+self._offset[0]
        gcy = cty*TILE_SIZE+cy_py+self._offset[1]
        tl_px = gcx-w//2; tl_py = gcy-h//2
        txs=int(tl_px//TILE_SIZE); tys=int(tl_py//TILE_SIZE)
        txe=int((tl_px+w)//TILE_SIZE); tye=int((tl_py+h)//TILE_SIZE)

        comp = Image.new("RGBA",(w,h),(15,17,23,255))
        for ty in range(tys,tye+1):
            for tx in range(txs,txe+1):
                tile = fetch_tile(z,tx,ty)
                comp.paste(tile,(int(tx*TILE_SIZE-tl_px),int(ty*TILE_SIZE-tl_py)))

        draw = ImageDraw.Draw(comp)

        if len(self._trail)>=2:
            pts=[(int(lng_to_px(p[1],tl_px,z)),int(lat_to_py(p[0],tl_py,z)))
                 for p in self._trail]
            total=len(pts)
            for i in range(total-1):
                a=int(80+175*i/max(total-1,1))
                draw.line([pts[i],pts[i+1]],fill=(88,166,255,a),width=3)

        if lat!=0 or lng!=0:
            mx=int(lng_to_px(lng,tl_px,z)); my=int(lat_to_py(lat,tl_py,z))
            draw.ellipse([mx-14,my-14,mx+14,my+14],fill=(88,166,255,55),outline=(88,166,255,200),width=2)
            draw.ellipse([mx-6,my-6,mx+6,my+6],fill=(88,166,255,255))
            draw.ellipse([mx-3,my-3,mx+3,my+3],fill=(255,255,255,255))

        tk_img=ImageTk.PhotoImage(comp)
        self._tkimgs=[tk_img]
        self.canvas.delete("all")
        self.canvas.create_image(0,0,anchor="nw",image=tk_img)
        self.canvas.create_text(8,8,anchor="nw",
            text=f"Zoom {z}  |  © OpenStreetMap contributors",fill=MUTED,font=FSM)

