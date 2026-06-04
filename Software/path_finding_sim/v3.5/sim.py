#!/usr/bin/env python3
"""
sim.py -- RSX CanSat 2026 descent-guidance interactive simulator (v3)

Real-time front-end on the real C++ v3 guidance (solve-from-current-state):
  * Play / Pause / Reset      -- fly forward in time, watch the glider.
  * sliders                   -- change every boundary condition live (re-solves):
                                 deploy heading, landing axis, glide ratio,
                                 min-turn R, max R, approach len, box size, etc.
  * Replan now / auto-replan  -- re-solve from the current state (measured GR).
  * DRAG the glider dot        -- horizontal gust/push, then it re-solves.
  * z-shift slider + Apply     -- inject a SUDDEN altitude change (thermal /
                                 downdraft / baro glitch) and watch the planner
                                 re-solve loops/radius for the new altitude.

The planner picks landing direction + spiral side + radius + loops itself; the
only "direction" input is the landing AXIS (drawn as the dotted red line).
Run:  python3 sim.py     (needs an interactive backend; see README if no window)
"""
import math, numpy as np
import matplotlib, matplotlib.pyplot as plt
INTERACTIVE = False
for _bk in ("TkAgg", "QtAgg", "Qt5Agg", "MacOSX", "GTK3Agg"):
    try:
        plt.switch_backend(_bk); _f = plt.figure(); plt.close(_f); INTERACTIVE = True; break
    except Exception:
        continue
if not INTERACTIVE:
    plt.switch_backend("Agg")
from matplotlib.widgets import Slider, Button       # noqa: E402
import pathguidance as pg                            # noqa: E402

G = 9.81
PHASE = {0:"Init",1:"Align",2:"Homing",3:"Loiter",4:"Approach",5:"Landed"}
SEGCOL = {0:"#eef3ff",1:"#eafbef"}
LAND = (0.0, 0.0)

def default_vals():
    return dict(home=45.0, axis=30.0, gr=3.0, minR=25.0, maxR=70.0, appr=40.0,
                box=220.0, look=8.0, alt=200.0, sink=1.0)

def build_params(v):
    p = pg.GuidanceParams()
    p.start_n, p.start_e, p.start_d = -150.0, -120.0, -v["alt"]
    p.start_heading = math.radians(v["home"])
    p.land_n, p.land_e, p.land_d = LAND[0], LAND[1], 0.0
    p.landing_axis = math.radians(v["axis"])
    p.glide_ratio = v["gr"]; p.approach_len = v["appr"]
    p.min_turn_radius = v["minR"]; p.max_radius = v["maxR"]
    h = v["box"]
    p.box_n_min, p.box_n_max, p.box_e_min, p.box_e_max = -h, h, -h, h
    p.lookahead_drop = v["look"]
    return p

def wrap_pi(a): return (a+math.pi)%(2*math.pi)-math.pi

def reference_xy(g, npts=600):
    L = g.total_length()
    if L <= 0: return np.array([]), np.array([])
    ss = np.linspace(0, L, npts); q = [g.eval_s(float(s)) for s in ss]
    return np.array([w[0] for w in q]), np.array([w[1] for w in q])

def curvature_xy(g, npts=800):
    L = g.total_length()
    if L <= 0: return np.array([]), np.array([])
    ss = np.linspace(0, L, npts)
    return ss, np.array([g.curvature_at(float(s)) for s in ss])

class Sim:
    def __init__(self, p, descent_rate=5.0, dt=0.05, kp_bank=2.2):
        self.dt, self.kp_bank, self.descent_rate = dt, kp_bank, descent_rate
        self.sink_bias = 1.0
        self.set_params(p)
    def set_params(self, p):
        self.p = p; self.g = pg.PathGuidance(p); self.status = self.g.plan(); self.reset_state()
    def feasible(self):
        return str(self.status).split('.')[-1] in ("Ok","Degraded")
    def reset_state(self):
        self.n, self.e, self.d = self.p.start_n, self.p.start_e, self.p.start_d
        self.yaw = self.p.start_heading
        self.Vh = self.p.glide_ratio*self.descent_rate
        self.phi_max = math.atan(self.Vh*self.Vh/(G*self.p.min_turn_radius))
        self.trail_n, self.trail_e = [self.n], [self.e]
        self.alt_log, self.xte_log, self.replan_marks = [], [], []
        self.landed, self.last_replan_d, self.cmd = False, self.d, None
    def _state(self):
        st = pg.State(); st.n,st.e,st.d = self.n,self.e,self.d
        st.vn=self.Vh*math.cos(self.yaw); st.ve=self.Vh*math.sin(self.yaw)
        st.vd=self.descent_rate*self.sink_bias; st.yaw=self.yaw
        return st
    def replan_now(self):
        if self.landed or not self.feasible(): return None
        r = self.g.replan(self._state())
        if str(r).split('.')[-1] in ("Ok","Degraded"): self.replan_marks.append((self.e,self.n))
        return r
    def zshift(self, dz):
        # dz>0 = sudden altitude GAIN (d more negative). clamp above ground/below deploy.
        self.d = max(self.p.start_d, min(self.p.land_d-1.0, self.d - dz))
        self.replan_now()
    def step(self, auto_replan_m=0.0):
        if self.landed or not self.feasible(): return
        self.cmd = self.g.get_heading(self._state())
        ph = int(self.cmd.phase)
        if auto_replan_m>0.0 and ph in (1,3) and (self.d-self.last_replan_d)>=auto_replan_m:
            self.replan_now(); self.last_replan_d=self.d; self.cmd=self.g.get_heading(self._state())
        err = wrap_pi(self.cmd.heading-self.yaw)
        bank = max(-self.phi_max, min(self.phi_max, self.kp_bank*err))
        self.yaw = wrap_pi(self.yaw + (G*math.tan(bank)/self.Vh)*self.dt)
        self.n += self.Vh*math.cos(self.yaw)*self.dt
        self.e += self.Vh*math.sin(self.yaw)*self.dt
        self.d += self.descent_rate*self.sink_bias*self.dt
        rn,re_ = reference_xy(self.g,220)
        xte = float(np.min(np.hypot(rn-self.n,re_-self.e))) if rn.size else 0.0
        self.trail_n.append(self.n); self.trail_e.append(self.e)
        self.alt_log.append(-self.d); self.xte_log.append(xte)
        if self.d >= self.p.land_d-1e-3: self.landed=True
    def miss(self):
        return math.hypot(self.n-self.p.land_n, self.e-self.p.land_e)

def run_gui():
    vals = default_vals(); sim = Sim(build_params(vals))
    fig = plt.figure(figsize=(14.5,9))
    fig.canvas.manager.set_window_title("RSX CanSat 2026 -- v3 descent guidance")
    gs = fig.add_gridspec(2,2,width_ratios=[1.55,1.0],left=0.055,right=0.985,top=0.95,bottom=0.40,hspace=0.34,wspace=0.20)
    axP=fig.add_subplot(gs[:,0]); axK=fig.add_subplot(gs[0,1]); axX=fig.add_subplot(gs[1,1])

    box_rect = plt.Rectangle((-vals["box"],-vals["box"]),2*vals["box"],2*vals["box"],fill=False,ec="0.7",ls="--")
    axP.add_patch(box_rect)
    (ref_line,)=axP.plot([],[],"--",color="0.6",lw=1.5,label="plan")
    (trail_line,)=axP.plot([],[],"-",color="#1f6feb",lw=2.0,label="flown")
    (carrot_line,)=axP.plot([],[],"-",color="#fb8500",lw=1.0,alpha=0.85)
    (circle_line,)=axP.plot([],[],":",color="0.7",lw=1.0)
    (glider_pt,)=axP.plot([],[],"o",color="#1f6feb",ms=10,mec="white",mew=1.3,zorder=9)
    (replan_pts,)=axP.plot([],[],"o",color="#2da44e",ms=6,ls="none",zorder=7)
    axP.scatter([LAND[1]],[LAND[0]],c="red",s=95,marker="*",zorder=6)
    axis_line=axP.plot([],[],":",color="red",lw=1.2)[0]
    (deploy_arrow,)=axP.plot([],[],"-",color="green",lw=1.5)
    axP.set_xlabel("East (m)"); axP.set_ylabel("North (m)")
    axP.grid(alpha=0.3); axP.legend(loc="upper left",fontsize=8); axP.set_aspect("equal",adjustable="datalim")

    (kappa_line,)=axK.plot([],[],"-",color="#8250df",lw=1.7)
    axK.set_xlabel("arc length s (m)"); axK.set_ylabel("kappa (1/m)")
    axK.set_title("Curvature (align & spiral steps, G1)",fontsize=9); axK.grid(alpha=0.3)
    seg_spans=[]
    (xte_line,)=axX.plot([],[],"-",color="#d1242f",lw=1.4)
    axX.set_xlabel("altitude AGL (m)"); axX.set_ylabel("cross-track (m)")
    axX.set_title("Tracking error",fontsize=9); axX.grid(alpha=0.3); axX.invert_xaxis()

    def mks(x,y,w,lbl,lo,hi,val,fmt="%.0f"):
        return Slider(fig.add_axes([x,y,w,0.02]),lbl,lo,hi,valinit=val,valfmt=fmt)
    cL,cM,cR=0.06,0.40,0.74; w=0.22
    s={}
    s["home"]=mks(cL,0.31,w,"deploy hdg (deg)",0,359,vals["home"])
    s["axis"]=mks(cL,0.275,w,"landing axis (deg)",0,179,vals["axis"])
    s["gr"]=mks(cL,0.24,w,"glide ratio",2.0,6.0,vals["gr"],"%.2f")
    s["alt"]=mks(cL,0.205,w,"deploy alt (m)",80,400,vals["alt"])
    s["minR"]=mks(cM,0.31,w,"min turn R (m)",15,50,vals["minR"])
    s["maxR"]=mks(cM,0.275,w,"max spiral R (m)",40,120,vals["maxR"])
    s["appr"]=mks(cM,0.24,w,"approach len (m)",20,90,vals["appr"])
    s["box"]=mks(cM,0.205,w,"box half-size (m)",90,260,vals["box"])
    s["look"]=mks(cR,0.31,w,"lookahead drop (m)",2,25,vals["look"])
    s["sink"]=mks(cR,0.275,w,"sink bias x",0.8,1.4,vals["sink"],"%.2f")
    s_rep=mks(cR,0.24,w,"auto-replan every (m)",0,60,0.0)
    s_spd=mks(cR,0.205,w,"sim speed (x)",0.5,6.0,2.0,"%.1f")
    s_z=mks(cR,0.17,w,"z-shift (m, +=gain)",-100,100,0.0)

    b_play=Button(fig.add_axes([cL,0.085,0.10,0.05]),"Play")
    b_reset=Button(fig.add_axes([cL+0.12,0.085,0.10,0.05]),"Reset")
    b_rep=Button(fig.add_axes([cL+0.24,0.085,0.11,0.05]),"Replan now")
    b_z=Button(fig.add_axes([cL+0.37,0.085,0.13,0.05]),"Apply z-shift")
    fig.text(cR,0.10,"Drag the glider = gust.\nz-shift = sudden altitude jump,\nthen it re-solves.",fontsize=8.5,color="0.3")

    stt={"run":False,"drag":False}

    def redraw_plan():
        for sp in seg_spans: sp.remove()
        seg_spans.clear()
        rn,re_=reference_xy(sim.g); ref_line.set_data(re_,rn)
        ss,ks=curvature_xy(sim.g); kappa_line.set_data(ss,ks)
        if ss.size:
            axK.set_xlim(0,ss[-1]); axK.set_ylim(min(-0.001,ks.min()*1.3),max(0.001,ks.max()*1.4))
            for i in range(sim.g.seg_count()):
                s0=sim.g.seg_s0(i); ln=sim.g.seg_len(i)
                seg_spans.append(axK.axvspan(s0,s0+ln,color=SEGCOL.get(int(sim.g.seg_type(i)),"white"),alpha=0.5))
        if sim.g.spiral_radius()>0:
            c=sim.g.center(); th=np.linspace(0,2*math.pi,90)
            circle_line.set_data(c.e+sim.g.spiral_radius()*np.sin(th),c.n+sim.g.spiral_radius()*np.cos(th))
        else: circle_line.set_data([],[])
        h=sim.p.box_n_max
        box_rect.set_bounds(-h,-h,2*h,2*h)
        ax=sim.p.landing_axis
        axis_line.set_data([LAND[1]-60*math.sin(ax),LAND[1]+60*math.sin(ax)],
                           [LAND[0]-60*math.cos(ax),LAND[0]+60*math.cos(ax)])
        deploy_arrow.set_data([sim.p.start_e,sim.p.start_e+25*math.sin(sim.p.start_heading)],
                              [sim.p.start_n,sim.p.start_n+25*math.cos(sim.p.start_heading)])
        axP.relim(); axP.autoscale_view()

    def redraw_state():
        trail_line.set_data(sim.trail_e,sim.trail_n)
        glider_pt.set_data([sim.e],[sim.n])
        if sim.cmd is not None: carrot_line.set_data([sim.e,sim.cmd.carrot.e],[sim.n,sim.cmd.carrot.n])
        if sim.replan_marks:
            re_,rn=zip(*sim.replan_marks); replan_pts.set_data(re_,rn)
        else: replan_pts.set_data([],[])
        if sim.alt_log:
            xte_line.set_data(sim.alt_log,sim.xte_log)
            axX.set_xlim(max(sim.alt_log),0); axX.set_ylim(0,max(2.0,max(sim.xte_log)*1.15))
        ph=PHASE.get(int(sim.cmd.phase),"-") if sim.cmd else "-"
        sname=str(sim.status).split('.')[-1]
        t=f"v3: {sname}"
        if sim.feasible():
            t+=(f" | {ph} | alt {-sim.d:.0f} m | land_dir {math.degrees(sim.g.landing_direction())%360:.0f}"
                f" side{sim.g.spiral_side():+d} R{sim.g.spiral_radius():.0f} N{sim.g.loops()}"
                f" | resid {sim.g.budget_residual():+.0f} m")
            if sim.landed: t+=f" | MISS {sim.miss():.1f} m"
        axP.set_title(t,fontsize=8.5)
        fig.canvas.draw_idle()

    timer=fig.canvas.new_timer(interval=33)
    def on_tick():
        if not stt["run"] or stt["drag"]: return
        for _ in range(max(1,int(round(s_spd.val*2)))):
            sim.step(s_rep.val)
            if sim.landed: break
        redraw_state()
        if sim.landed: stt["run"]=False; b_play.label.set_text("Play")
    timer.add_callback(on_tick); timer.start()

    def apply(_=None):
        v={k:s[k].val for k in s}; sim.sink_bias=v["sink"]
        sim.set_params(build_params(v))
        stt["run"]=False; b_play.label.set_text("Play"); redraw_plan(); redraw_state()
    for k in s: s[k].on_changed(apply)
    def on_play(_): stt["run"]=not stt["run"]; b_play.label.set_text("Pause" if stt["run"] else "Play")
    b_play.on_clicked(on_play)
    def on_reset(_): sim.reset_state(); stt["run"]=False; b_play.label.set_text("Play"); redraw_state()
    b_reset.on_clicked(on_reset)
    def on_replan(_): sim.replan_now(); redraw_plan(); redraw_state()
    b_rep.on_clicked(on_replan)
    def on_z(_): sim.zshift(s_z.val); redraw_plan(); redraw_state()
    b_z.on_clicked(on_z)

    def hit(ev): return (ev.inaxes is axP and ev.xdata is not None and
                         math.hypot(ev.xdata-sim.e,ev.ydata-sim.n)<max(8.0,sim.p.min_turn_radius*0.3))
    def on_press(ev):
        if hit(ev): stt["drag"]=True
    def on_motion(ev):
        if stt["drag"] and ev.inaxes is axP and ev.xdata is not None:
            sim.e,sim.n=ev.xdata,ev.ydata; sim.trail_e.append(sim.e); sim.trail_n.append(sim.n); redraw_state()
    def on_release(ev): stt["drag"]=False
    fig.canvas.mpl_connect("button_press_event",on_press)
    fig.canvas.mpl_connect("motion_notify_event",on_motion)
    fig.canvas.mpl_connect("button_release_event",on_release)
    redraw_plan(); redraw_state(); plt.show()

def save_static():
    vals=default_vals(); sim=Sim(build_params(vals))
    while not sim.landed and sim.feasible(): sim.step()
    fig=plt.figure(figsize=(14,8))
    gs=fig.add_gridspec(2,2,width_ratios=[1.55,1.0],left=0.06,right=0.98,top=0.92,bottom=0.08,hspace=0.3,wspace=0.2)
    axP=fig.add_subplot(gs[:,0]); axK=fig.add_subplot(gs[0,1]); axX=fig.add_subplot(gs[1,1])
    h=sim.p.box_n_max; axP.add_patch(plt.Rectangle((-h,-h),2*h,2*h,fill=False,ec="0.7",ls="--"))
    rn,re_=reference_xy(sim.g); axP.plot(re_,rn,"--",color="0.6",lw=1.5,label="plan")
    axP.plot(sim.trail_e,sim.trail_n,"-",color="#1f6feb",lw=2,label="flown")
    axP.scatter([sim.p.start_e],[sim.p.start_n],c="green",s=55); axP.scatter([LAND[1]],[LAND[0]],c="red",s=95,marker="*")
    ax=sim.p.landing_axis; axP.plot([LAND[1]-60*math.sin(ax),LAND[1]+60*math.sin(ax)],[LAND[0]-60*math.cos(ax),LAND[0]+60*math.cos(ax)],":",color="red")
    axP.set_aspect("equal"); axP.grid(alpha=0.3); axP.legend(fontsize=8)
    axP.set_title("v3  land_dir %.0f side%+d R%.0f N%d  miss %.1f m  (install python3-tk for window)"%(
        math.degrees(sim.g.landing_direction())%360,sim.g.spiral_side(),sim.g.spiral_radius(),sim.g.loops(),sim.miss()),fontsize=9)
    ss,ks=curvature_xy(sim.g); axK.plot(ss,ks,color="#8250df"); axK.grid(alpha=0.3); axK.set_title("Curvature (G1 steps)",fontsize=9)
    axX.plot(sim.alt_log,sim.xte_log,color="#d1242f"); axX.invert_xaxis(); axX.grid(alpha=0.3); axX.set_title("Cross-track",fontsize=9)
    fig.savefig("path_plot_v3.png",dpi=125); print("saved path_plot_v3.png")

if __name__ == "__main__":
    if not INTERACTIVE:
        print("No interactive matplotlib backend found (window can't open here).")
        print("  Linux:  sudo apt-get install python3-tk   then re-run  python3 sim.py")
        save_static()
    else:
        print(f"interactive backend: {matplotlib.get_backend()}  --  opening window"); run_gui()
