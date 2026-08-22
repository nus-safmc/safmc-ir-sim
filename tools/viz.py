"""Build a self-contained HTML replay of a recorded run.

Reads **only** the log (R-OBS-3) -- it never imports the simulator's world or re-runs
anything, which is the property that makes it a real replay rather than a second rendering
path that can silently disagree with the first.

Usage:
    python tools/viz.py runs/my_run -o runs/my_run/replay.html
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from safmc_sim.recorder import COMMAND_NAMES, LIFECYCLE_NAMES, load_run  # noqa: E402


def _round(array: np.ndarray, decimals: int) -> list:
    """Round to a short decimal that serialises compactly.

    Rounding a float32 array in place does not help: ``float32(2.87)`` is 2.869999885559082 as
    a float64, and that is what ``tolist()`` emits. Widening first makes the rounding land on a
    value whose shortest repr really is "2.87", which is a ~5x difference in page size.
    """
    return np.round(array.astype(np.float64), decimals).tolist()


def build_payload(directory: Path, tof_every: int = 10, pose_every: int = 1) -> dict:
    run = load_run(directory)
    header, footer, events = run["header"], run["footer"], run["events"]
    states = run["states"]

    pose = states["pose"][::pose_every]
    lifecycle = states["lifecycle"][::pose_every]
    times = states["time_s"][::pose_every]
    kinds = states["command_kind"][::pose_every]

    tof = run.get("tof")
    tof_frames, tof_ticks, zone_bearings = [], [], []
    if tof is not None:
        stride = max(tof_every // pose_every, 1)
        sampled = tof["ranges_m"][::stride]
        tof_ticks = (tof["ticks"][::stride] // pose_every).astype(int).tolist()
        # inf does not survive JSON; -1 is the agreed "no return" sentinel on the JS side.
        clean = np.where(np.isfinite(sampled), sampled, -1.0)
        tof_frames = _round(clean, 2)
        zone_bearings = _round(np.asarray(tof["zone_bearings_rad"]).reshape(-1), 4)

    return {
        "arena": header["arena"],
        "agents": header["agents"],
        "config": header["config"],
        "seed": header["seed"],
        "meta": header["meta"],
        "score": footer["score"],
        "missionSummary": footer["mission_summary"],
        "lifecycles": footer["lifecycles"],
        "times": _round(times, 2),
        "pose": _round(pose, 2),
        "lifecycleSeries": lifecycle.astype(int).tolist(),
        "commandKind": kinds.astype(int).tolist(),
        "lifecycleNames": LIFECYCLE_NAMES,
        "commandNames": COMMAND_NAMES,
        "events": events,
        "tofTicks": tof_ticks,
        "tof": tof_frames,
        "zoneBearings": zone_bearings,
    }


TEMPLATE = """<meta charset="utf-8">
<title>SAFMC Run Replay</title>
<style>
:root{
  --bg:#f6f7f9; --panel:#ffffff; --ink:#16181d; --muted:#606774; --line:#dfe3e8;
  --accent:#2f6fed; --warn:#c2410c; --bad:#b91c1c; --good:#15803d;
  --wall:#4b5563; --pillar:#64748b; --net:#c3c8d0;
}
:root:not([data-theme="light"]){ @media (prefers-color-scheme: dark){
  --bg:#0f1115; --panel:#171a21; --ink:#e8eaee; --muted:#9aa2b1; --line:#272c36;
  --accent:#6ea0ff; --warn:#fb923c; --bad:#f87171; --good:#4ade80;
  --wall:#8b93a3; --pillar:#7c8798; --net:#3a414e;
}}
:root[data-theme="dark"]{
  --bg:#0f1115; --panel:#171a21; --ink:#e8eaee; --muted:#9aa2b1; --line:#272c36;
  --accent:#6ea0ff; --warn:#fb923c; --bad:#f87171; --good:#4ade80;
  --wall:#8b93a3; --pillar:#7c8798; --net:#3a414e;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:1400px;margin:0 auto;padding:20px}
h1{font-size:19px;margin:0 0 2px;letter-spacing:-.01em}
.sub{color:var(--muted);font-size:13px;margin-bottom:16px}
.grid{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(280px,1fr);gap:16px}
@media(max-width:940px){.grid{grid-template-columns:1fr}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.07em;color:var(--muted);
  margin:0 0 10px;font-weight:600}
svg{width:100%;height:auto;display:block;border-radius:6px;background:var(--bg)}
.controls{display:flex;gap:10px;align-items:center;margin-top:12px;flex-wrap:wrap}
button{background:var(--accent);color:#fff;border:0;border-radius:6px;padding:6px 14px;
  font-size:13px;cursor:pointer;font-weight:500}
button.ghost{background:transparent;color:var(--ink);border:1px solid var(--line)}
input[type=range]{flex:1;min-width:180px;accent-color:var(--accent)}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font-size:13px}
.kv dt{color:var(--muted)}
.kv dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}
.scroll{max-height:260px;overflow-y:auto}
table{width:100%;border-collapse:collapse;font-size:12px}
th,td{text-align:left;padding:3px 6px;border-bottom:1px solid var(--line)}
th{color:var(--muted);font-weight:600;position:sticky;top:0;background:var(--panel)}
td.num{text-align:right;font-variant-numeric:tabular-nums}
.pill{display:inline-block;padding:1px 7px;border-radius:999px;font-size:11px;font-weight:600}
.LANDED{background:color-mix(in srgb,var(--good) 18%,transparent);color:var(--good)}
.CRASHED{background:color-mix(in srgb,var(--bad) 18%,transparent);color:var(--bad)}
.FLYING,.TAKEOFF,.LANDING{background:color-mix(in srgb,var(--accent) 18%,transparent);color:var(--accent)}
.IDLE{background:color-mix(in srgb,var(--muted) 18%,transparent);color:var(--muted)}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:12px;color:var(--muted);margin-top:8px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;
  vertical-align:-1px}
.note{font-size:12px;color:var(--muted);margin-top:10px;line-height:1.5}
</style>
<div class="wrap">
  <h1 id="title">SAFMC Run Replay</h1>
  <div class="sub mono" id="subtitle"></div>
  <div class="grid">
    <div>
      <div class="card">
        <svg id="field" viewBox="0 0 520 520" role="img" aria-label="Arena replay"></svg>
        <div class="controls">
          <button id="play">Play</button>
          <input type="range" id="scrub" min="0" value="0" step="1">
          <span class="mono" id="clock"></span>
          <button class="ghost" id="tofToggle">ToF: on</button>
          <button class="ghost" id="trailToggle">Trails: on</button>
        </div>
        <div class="legend">
          <span><i style="background:var(--wall)"></i>wall</span>
          <span><i style="background:var(--pillar)"></i>pillar</span>
          <span><i style="background:var(--net)"></i>net (south edge)</span>
          <span><i style="background:#0ea5e9"></i>victim</span>
          <span><i style="background:#a855f7"></i>bonus victim</span>
          <span><i style="background:#ef4444"></i>fire</span>
          <span><i style="background:#22c55e"></i>serviced</span>
        </div>
        <div class="note" id="arenaNote"></div>
      </div>
    </div>
    <div>
      <div class="card"><h2>Score</h2><dl class="kv" id="scoreBox"></dl></div>
      <div class="card" style="margin-top:14px"><h2>Agents</h2>
        <div class="scroll"><table id="agentTable"></table></div></div>
      <div class="card" style="margin-top:14px"><h2>Events</h2>
        <div class="scroll"><table id="eventTable"></table></div></div>
    </div>
  </div>
</div>
<script id="payload" type="application/json">__PAYLOAD__</script>
<script>
const D = JSON.parse(document.getElementById('payload').textContent);
const NS='http://www.w3.org/2000/svg', PAD=10, SIZE=500;
const W=D.arena.width_m, H=D.arena.depth_m, S=SIZE/Math.max(W,H);
const sx = x => PAD + x*S;
const sy = y => PAD + (H - y)*S;          // SVG y grows downward; arena y grows north
const svg = document.getElementById('field');
const el = (t,a={}) => { const n=document.createElementNS(NS,t);
  for(const k in a) n.setAttribute(k,a[k]); return n; };

const TARGET_COLOR = {victim:'#0ea5e9', bonus_victim:'#a855f7', fire:'#ef4444'};
const PALETTE = ['#2f6fed','#e11d48','#059669','#d97706','#7c3aed','#0891b2','#be123c',
                 '#4d7c0f','#c2410c','#6d28d9','#0369a1','#a16207'];

/* static layer -------------------------------------------------------------- */
const startBand = el('rect',{x:sx(0),y:sy(D.arena.start_area_depth_m),
  width:W*S,height:D.arena.start_area_depth_m*S,fill:'currentColor','fill-opacity':.05});
svg.appendChild(startBand);
const ua = D.arena.unknown_area;
svg.appendChild(el('rect',{x:sx(ua[0]),y:sy(ua[3]),width:(ua[2]-ua[0])*S,
  height:(ua[3]-ua[1])*S,fill:'#eab308','fill-opacity':.07,
  stroke:'#eab308','stroke-opacity':.35,'stroke-dasharray':'4 3'}));

for(const w of D.arena.walls){
  const dx=w.x2-w.x1, dy=w.y2-w.y1, len=Math.hypot(dx,dy);
  const ang=-Math.atan2(dy,dx)*180/Math.PI;
  const cx=(w.x1+w.x2)/2, cy=(w.y1+w.y2)/2;
  const t=Math.max(w.thickness_m*S,1.6);
  svg.appendChild(el('rect',{x:sx(cx)-len*S/2,y:sy(cy)-t/2,width:len*S,height:t,
    fill:w.kind==='net'?'var(--net)':'var(--wall)',
    transform:`rotate(${ang} ${sx(cx)} ${sy(cy)})`}));
}
for(const p of D.arena.pillars)
  svg.appendChild(el('circle',{cx:sx(p.x),cy:sy(p.y),r:Math.max(p.radius_m*S,2),
    fill:'var(--pillar)'}));

const targetNodes={};
for(const t of D.arena.targets){
  const g=el('g'); const c=TARGET_COLOR[t.kind]||'#888';
  const ring=el('circle',{cx:sx(t.x),cy:sy(t.y),r:1.0*S,fill:c,'fill-opacity':.10,
    stroke:c,'stroke-opacity':.4,'stroke-dasharray':'3 3'});   // the 1 m scoring radius
  const dot=el('circle',{cx:sx(t.x),cy:sy(t.y),r:5,fill:c,stroke:'var(--panel)','stroke-width':1.5});
  g.appendChild(ring); g.appendChild(dot); svg.appendChild(g);
  targetNodes[t.id]={dot,ring};
}
for(const [id,info] of Object.entries(D.missionSummary))
  if(info.serviced && targetNodes[id]){
    targetNodes[id].dot.setAttribute('fill','#22c55e');
    targetNodes[id].ring.setAttribute('stroke','#22c55e');
  }

/* dynamic layer ------------------------------------------------------------- */
const tofLayer=el('g',{stroke:'var(--warn)','stroke-opacity':.45,'stroke-width':.7});
const trailLayer=el('g',{fill:'none','stroke-width':1.2,'stroke-opacity':.55});
const droneLayer=el('g');
svg.appendChild(tofLayer); svg.appendChild(trailLayer); svg.appendChild(droneLayer);

const N=D.agents.length, T=D.times.length;
const trails=D.agents.map((_,i)=>{
  const p=el('path',{stroke:PALETTE[i%PALETTE.length]}); trailLayer.appendChild(p); return p;});
const drones=D.agents.map((_,i)=>{
  const g=el('g');
  const body=el('circle',{r:4,fill:PALETTE[i%PALETTE.length],stroke:'var(--panel)','stroke-width':1});
  const nose=el('line',{stroke:PALETTE[i%PALETTE.length],'stroke-width':1.6});
  g.appendChild(nose); g.appendChild(body); droneLayer.appendChild(g);
  return {g,body,nose};});

let showTof=true, showTrails=true, frame=0, playing=false, timer=null;

function nearestTofFrame(f){
  if(!D.tof.length) return null;
  let best=0;
  for(let i=0;i<D.tofTicks.length;i++){ if(D.tofTicks[i]<=f) best=i; else break; }
  return D.tof[best];
}

function draw(f){
  frame=Math.max(0,Math.min(f,T-1));
  const P=D.pose[frame], L=D.lifecycleSeries[frame];
  for(let i=0;i<N;i++){
    const [x,y,z,th]=P[i], name=D.lifecycleNames[L[i]];
    const cx=sx(x), cy=sy(y);
    drones[i].body.setAttribute('cx',cx); drones[i].body.setAttribute('cy',cy);
    drones[i].nose.setAttribute('x1',cx); drones[i].nose.setAttribute('y1',cy);
    drones[i].nose.setAttribute('x2',cx+Math.cos(th)*9);
    drones[i].nose.setAttribute('y2',cy-Math.sin(th)*9);
    drones[i].body.setAttribute('r', name==='LANDED'?5.5:4);
    drones[i].g.setAttribute('opacity', name==='CRASHED'?0.28:(name==='IDLE'?0.5:1));
    drones[i].body.setAttribute('stroke', name==='CRASHED'?'var(--bad)':'var(--panel)');
    if(showTrails){
      const start=Math.max(0,frame-260); let d='';
      for(let k=start;k<=frame;k+=2){ const q=D.pose[k][i];
        d += (d?'L':'M')+sx(q[0]).toFixed(1)+' '+sy(q[1]).toFixed(1)+' '; }
      trails[i].setAttribute('d',d);
    } else trails[i].setAttribute('d','');
  }
  tofLayer.replaceChildren();
  const scan=showTof?nearestTofFrame(frame):null;
  if(scan){
    for(let i=0;i<N;i++){
      const name=D.lifecycleNames[L[i]];
      if(name!=='ACTIVE') continue;
      const [x,y,,th]=P[i], bins=scan[i]; if(!bins) continue;
      for(let b=0;b<bins.length;b++){
        const r=bins[b]; if(r<0) continue;              // -1 is the no-return sentinel
        const bearing = th + D.zoneBearings[b];         // body-frame, CCW from the nose
        tofLayer.appendChild(el('line',{x1:sx(x),y1:sy(y),
          x2:sx(x+r*Math.cos(bearing)),y2:sy(y+r*Math.sin(bearing))}));
      }
    }
  }
  document.getElementById('clock').textContent =
    `t=${D.times[frame].toFixed(2)}s  tick ${frame}/${T-1}`;
  document.getElementById('scrub').value=frame;
  renderAgents(frame);
}

function renderAgents(f){
  const L=D.lifecycleSeries[f], P=D.pose[f], K=D.commandKind[f];
  let html='<tr><th>agent</th><th>state</th><th>cmd</th><th class="num">x</th>'+
           '<th class="num">y</th><th class="num">z</th></tr>';
  for(let i=0;i<N;i++){
    const s=D.lifecycleNames[L[i]];
    html+=`<tr><td class="mono">${D.agents[i]}</td>`+
      `<td><span class="pill ${s}">${s}</span></td>`+
      `<td class="mono">${D.commandNames[K[i]]||''}</td>`+
      `<td class="num">${P[i][0].toFixed(1)}</td><td class="num">${P[i][1].toFixed(1)}</td>`+
      `<td class="num">${P[i][2].toFixed(2)}</td></tr>`;
  }
  document.getElementById('agentTable').innerHTML=html;
}

/* side panels --------------------------------------------------------------- */
const sc=D.score;
document.getElementById('scoreBox').innerHTML =
  `<dt>raw</dt><dd>${sc.raw_total}</dd>`+
  `<dt>suppressed by fire</dt><dd>${sc.suppressed.length?sc.suppressed.join(', '):'none'}</dd>`+
  `<dt>relay</dt><dd>${sc.relay_formed?sc.relay_chain.join('  ->  '):'not formed'}</dd>`+
  `<dt>multiplier</dt><dd>&times;${sc.multiplier}</dd>`+
  `<dt style="font-weight:600;color:var(--ink)">total</dt>`+
  `<dd style="font-weight:700;font-size:17px">${sc.total}</dd>`;

document.getElementById('eventTable').innerHTML =
  '<tr><th class="num">t</th><th>event</th><th>agent</th><th>detail</th></tr>' +
  D.events.map(e=>{
    const d=e.detail||{};
    const text = d.target_id ? `${d.kind} +${d.points}`
              : d.reason ? d.reason
              : d.rule ? d.rule
              : d.wave!==undefined ? `wave ${d.wave}`
              : Object.keys(d).length ? Object.entries(d).map(([k,v])=>
                  `${k}=${typeof v==='number'?v.toFixed(2):v}`).join(' ') : '';
    const cls = e.kind==='crashed'?'style="color:var(--bad)"'
              : e.kind==='target_serviced'?'style="color:var(--good)"'
              : e.kind==='rule_violation'?'style="color:var(--warn)"':'';
    return `<tr><td class="num mono">${e.sim_time_s.toFixed(1)}</td>`+
           `<td ${cls}>${e.kind}</td><td class="mono">${e.agent_id||''}</td>`+
           `<td class="mono">${text}</td></tr>`;
  }).join('');

const cfg=D.config;
document.getElementById('title').textContent =
  `${cfg.policy} | ${D.agents.length} drones | seed ${D.seed}`;
document.getElementById('subtitle').textContent =
  `${D.times[T-1].toFixed(0)}s simulated | ${T} ticks | collision_behaviour=`+
  `${cfg.collision_behaviour} | ir-sim ${D.meta.versions['ir-sim']}`;
document.getElementById('arenaNote').textContent =
  `Dashed circles are the 1 m scoring radius; a target turns green when a landed drone `+
  `satisfied both the radius and line of sight. Shaded band is the Start Area; dashed `+
  `square is the Unknown Search Area. Orange rays are the 64-bin collapsed ToF scan.`;

/* transport ----------------------------------------------------------------- */
const scrub=document.getElementById('scrub'); scrub.max=T-1;
scrub.addEventListener('input',e=>{stop();draw(+e.target.value);});
function stop(){playing=false;clearInterval(timer);document.getElementById('play').textContent='Play';}
document.getElementById('play').addEventListener('click',()=>{
  if(playing){stop();return;}
  playing=true; document.getElementById('play').textContent='Pause';
  timer=setInterval(()=>{ if(frame>=T-1){stop();return;} draw(frame+2); },33);
});
document.getElementById('tofToggle').addEventListener('click',e=>{
  showTof=!showTof; e.target.textContent='ToF: '+(showTof?'on':'off'); draw(frame);});
document.getElementById('trailToggle').addEventListener('click',e=>{
  showTrails=!showTrails; e.target.textContent='Trails: '+(showTrails?'on':'off'); draw(frame);});
addEventListener('keydown',e=>{
  if(e.key==='ArrowRight'){stop();draw(frame+1);}
  if(e.key==='ArrowLeft'){stop();draw(frame-1);}
  if(e.key===' '){e.preventDefault();document.getElementById('play').click();}
});
draw(0);
</script>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--tof-every", type=int, default=10,
                        help="record every Nth ToF frame into the page (size control)")
    parser.add_argument("--pose-every", type=int, default=1,
                        help="keep every Nth tick (size control for long runs)")
    args = parser.parse_args()

    payload = build_payload(args.run_dir, args.tof_every, args.pose_every)
    output = args.output or (args.run_dir / "replay.html")
    output.write_text(
        TEMPLATE.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":"))),
        encoding="utf-8",
    )
    size_kb = output.stat().st_size // 1024
    print(f"wrote {output} ({size_kb} KB, {len(payload['times'])} frames, "
          f"{len(payload['agents'])} agents)")


if __name__ == "__main__":
    main()
