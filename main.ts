// Push-level A* Sokoban solver. Generated levels are a hallway plus separate
// rooms; boxes never leave their room, so each room is solved independently
// with push-level A* (Hungarian matching heuristic, dead-square and 2x2
// pruning), and the room solutions are joined by plain walks through the
// hallway. A whole-level push A* is the fallback if decomposition fails.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DIRS: [number, number, string][] = [[0,-1,'u'],[0,1,'d'],[-1,0,'l'],[1,0,'r']];

function hungarian(cost: number[][], n: number): number {
  const u = new Array(n+1).fill(0), v = new Array(n+1).fill(0);
  const p = new Array(n+1).fill(0), way = new Array(n+1).fill(0);
  for (let i=1;i<=n;i++){
    p[0]=i; let j0=0; const minv=new Array(n+1).fill(Infinity), used=new Array(n+1).fill(false);
    do{
      used[j0]=true; const i0=p[j0]; let delta=Infinity, j1=0;
      for(let j=1;j<=n;j++) if(!used[j]){
        const cur=cost[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]){minv[j]=cur;way[j]=j0;}
        if(minv[j]<delta){delta=minv[j];j1=j;}
      }
      for(let j=0;j<=n;j++){ if(used[j]){u[p[j]]+=delta;v[j]-=delta;} else minv[j]-=delta; }
      j0=j1;
    }while(p[j0]!==0);
    do{ const j1=way[j0]; p[j0]=p[j1]; j0=j1; }while(j0);
  }
  return -v[0];
}

type Grid = { w:number, h:number, wall:Uint8Array, goals:number[], goalSet:Set<number> };

// Push A* restricted to cells where `walkable` is true; boxes may only be
// pushed onto cells where `boxOk` is true. Returns move string or null.
function pushAStar(g: Grid, startPlayer: number, startBoxes: number[], walkable: Uint8Array, boxOk: Uint8Array, cap: number): string | null {
  const { w, h, wall, goals, goalSet } = g;
  const N = w*h, nb = startBoxes.length;
  if (nb===0) return '';
  const goalDist: Int32Array[] = goals.map(gl=>{
    const d=new Int32Array(N).fill(-1); d[gl]=0; const q=[gl];
    for(let qi=0;qi<q.length;qi++){const c=q[qi]!; const cx=c%w, cy=(c/w)|0;
      for(const [dx,dy] of DIRS){const nx=cx+dx, ny=cy+dy; if(nx<0||ny<0||nx>=w||ny>=h) continue; const ni=ny*w+nx; if(wall[ni]||!boxOk[ni]||d[ni]!==-1) continue; d[ni]=d[c]!+1; q.push(ni);} }
    return d;
  });
  const live = new Uint8Array(N);
  { const q: number[] = [];
    for(const gl of goals){live[gl]=1;q.push(gl);}
    for(let qi=0;qi<q.length;qi++){const c=q[qi]!; const cx=c%w, cy=(c/w)|0;
      for(const [dx,dy] of DIRS){
        const px=cx-dx, py=cy-dy, sx=cx-2*dx, sy=cy-2*dy;
        if(px<0||py<0||px>=w||py>=h||sx<0||sy<0||sx>=w||sy>=h) continue;
        const pi=py*w+px, si=sy*w+sx;
        if(wall[pi]||!boxOk[pi]||wall[si]||!walkable[si]||live[pi]) continue;
        live[pi]=1; q.push(pi);
      }}
  }
  const dead = new Uint8Array(N);
  for(let i=0;i<N;i++) if(!wall[i] && boxOk[i] && !live[i] && !goalSet.has(i)) dead[i]=1;
  for(const b of startBoxes) if(dead[b]) return null;
  const heur=(bs:number[]):number=>{
    const cost:number[][]=new Array(nb);
    for(let i=0;i<nb;i++){cost[i]=new Array(nb); for(let j=0;j<nb;j++){const d=goalDist[j]![bs[i]!]; cost[i][j]=d<0?1000:d;}}
    return hungarian(cost,nb);
  };
  const has2x2=(boxSet:Set<number>):boolean=>{
    for(let y=0;y<h-1;y++) for(let x=0;x<w-1;x++){
      const cells=[y*w+x,y*w+x+1,(y+1)*w+x,(y+1)*w+x+1];
      let all=true,bad=false;
      for(const c of cells){ if(wall[c]||!boxOk[c]) continue; if(boxSet.has(c)){ if(!goalSet.has(c)) bad=true; continue;} all=false; break; }
      if(all&&bad) return true;
    } return false;
  };
  const reach=(start:number, boxSet:Set<number>)=>{
    const prev=new Int32Array(N).fill(-2); prev[start]=-1; const q=[start]; let min=start;
    for(let qi=0;qi<q.length;qi++){const c=q[qi]!; if(c<min)min=c; const cx=c%w, cy=(c/w)|0;
      for(const [dx,dy] of DIRS){const nx=cx+dx, ny=cy+dy; if(nx<0||ny<0||nx>=w||ny>=h) continue; const ni=ny*w+nx; if(wall[ni]||!walkable[ni]||boxSet.has(ni)||prev[ni]!==-2) continue; prev[ni]=c; q.push(ni);} }
    return {prev,min};
  };
  const pathTo=(prev:Int32Array, from:number, to:number):string=>{
    const rev:string[]=[]; let c=to;
    while(c!==from&&c>=0){const p=prev[c]!; const d=c-p; rev.push(d===1?'r':d===-1?'l':d===w?'d':'u'); c=p;}
    return rev.reverse().join('');
  };
  type Node={boxes:number[],region:number,g:number,parent:number,seg:string,actual:number};
  const nodes: Node[]=[];
  const boxes0=startBoxes.slice().sort((a,b)=>a-b);
  const r0=reach(startPlayer,new Set(boxes0));
  nodes.push({boxes:boxes0,region:r0.min,g:0,parent:-1,seg:'',actual:startPlayer});
  const best=new Map<string,number>();
  const keyOf=(bs:number[],r:number)=>bs.join(',')+'|'+r;
  best.set(keyOf(boxes0,r0.min),0);
  const heap:[number,number,number][]=[];
  const less=(a:[number,number,number],b:[number,number,number])=>a[0]<b[0]||(a[0]===b[0]&&(a[1]<b[1]||(a[1]===b[1]&&a[2]<b[2])));
  const hpush=(e:[number,number,number])=>{let i=heap.length;heap.push(e);while(i>0){const p=(i-1)>>1;if(!less(heap[i]!,heap[p]!))break;[heap[i],heap[p]]=[heap[p]!,heap[i]!];i=p;}};
  const hpop=():[number,number,number]=>{const top=heap[0]!;const last=heap.pop()!;if(heap.length){heap[0]=last;for(let i=0;;){let c=2*i+1;if(c>=heap.length)break;if(c+1<heap.length&&less(heap[c+1]!,heap[c]!))c++;if(!less(heap[c]!,heap[i]!))break;[heap[i],heap[c]]=[heap[c]!,heap[i]!];i=c;}}return top;};
  hpush([heur(boxes0),0,0]);
  let expanded=0;
  while(heap.length&&expanded<cap){
    const [,,ni]=hpop(); const node=nodes[ni]!;
    if(best.get(keyOf(node.boxes,node.region))!==node.g) continue;
    if(node.boxes.every(b=>goalSet.has(b))){
      let s=''; for(let t=ni;t>0;t=nodes[t]!.parent) s=nodes[t]!.seg+s; return s;
    }
    expanded++;
    const boxSet=new Set(node.boxes);
    const {prev}=reach(node.actual,boxSet);
    for(let bi=0;bi<nb;bi++){
      const b=node.boxes[bi]!; const bx=b%w, by=(b/w)|0;
      for(const [dx,dy,letter] of DIRS){
        const tx=bx+dx, ty=by+dy; if(tx<0||ty<0||tx>=w||ty>=h) continue;
        const target=ty*w+tx;
        const sx=bx-dx, sy=by-dy; if(sx<0||sy<0||sx>=w||sy>=h) continue;
        const stand=sy*w+sx;
        if(wall[target]||!boxOk[target]||boxSet.has(target)||dead[target]) continue;
        if(prev[stand]===-2) continue;
        const nbs=node.boxes.slice(); nbs[bi]=target; nbs.sort((a,b2)=>a-b2);
        const nBoxSet=new Set(nbs);
        if(has2x2(nBoxSet)) continue;
        const r=reach(b,nBoxSet);
        const nk=keyOf(nbs,r.min); const ng=node.g+1;
        const seen=best.get(nk); if(seen!==undefined&&seen<=ng) continue;
        best.set(nk,ng);
        const seg=pathTo(prev,node.actual,stand)+letter;
        const idx=nodes.length;
        nodes.push({boxes:nbs,region:r.min,g:ng,parent:ni,seg,actual:b});
        hpush([ng+heur(nbs),-ng,idx]);
      }
    }
  }
  return null;
}

// Walk-only BFS path on the full grid avoiding boxes; returns moves or null.
function walkPath(g: Grid, from: number, to: number, boxSet: Set<number>): string | null {
  const { w, h, wall } = g; const N=w*h;
  if(from===to) return '';
  const prev=new Int32Array(N).fill(-2); prev[from]=-1; const q=[from];
  for(let qi=0;qi<q.length;qi++){const c=q[qi]!; const cx=c%w, cy=(c/w)|0;
    for(const [dx,dy] of DIRS){const nx=cx+dx, ny=cy+dy; if(nx<0||ny<0||nx>=w||ny>=h) continue; const ni=ny*w+nx; if(wall[ni]||boxSet.has(ni)||prev[ni]!==-2) continue; prev[ni]=c; q.push(ni);} }
  if(prev[to]===-2) return null;
  const rev:string[]=[]; let c=to;
  while(c!==from){const p=prev[c]!; const d=c-p; rev.push(d===1?'r':d===-1?'l':d===w?'d':'u'); c=p;}
  return rev.reverse().join('');
}

// Apply moves to (player, boxes) state; returns new state or null if illegal.
function applyMoves(g: Grid, player: number, boxes: Set<number>, moves: string): {player:number, boxes:Set<number>} | null {
  const { w, wall } = g; let p=player; const bs=new Set(boxes);
  for(const ch0 of moves){ const ch=ch0.toLowerCase();
    const [dx,dy]=DIRS.find(d=>d[2]===ch)!; const px=p%w;
    if((dx===-1&&px===0)||(dx===1&&px===w-1)) return null;
    const q=p+dy*w+dx; if(wall[q]) return null;
    if(bs.has(q)){ const bx=q%w; if((dx===-1&&bx===0)||(dx===1&&bx===w-1)) return null; const t=q+dy*w+dx; if(wall[t]||bs.has(t)) return null; bs.delete(q); bs.add(t); }
    p=q;
  }
  return {player:p, boxes:bs};
}

function solve(text: string): string {
  const rows=text.split('\n').filter(r=>r.length>0);
  const h=rows.length, w=Math.max(...rows.map(r=>r.length)), N=w*h;
  const wall=new Uint8Array(N); const goals:number[]=[]; const goalSet=new Set<number>();
  let boxes:number[]=[]; let player=0;
  for(let y=0;y<h;y++) for(let x=0;x<w;x++){
    const ch=rows[y]![x]??'#'; const i=y*w+x;
    wall[i]=ch==='#'?1:0;
    if('.*+'.includes(ch)){goals.push(i);goalSet.add(i);}
    if('$*'.includes(ch)) boxes.push(i);
    if('@+'.includes(ch)) player=i;
  }
  const g: Grid={w,h,wall,goals,goalSet};
  if(boxes.length===0) return '';
  // --- decomposition: remove the player's hallway row, find components ---
  const hy=(player/w)|0;
  const comp=new Int32Array(N).fill(-1);
  let ncomp=0;
  for(let i=0;i<N;i++){
    if(wall[i]||comp[i]!==-1||((i/w)|0)===hy) continue;
    const q=[i]; comp[i]=ncomp;
    for(let qi=0;qi<q.length;qi++){const c=q[qi]!; const cx=c%w, cy=(c/w)|0;
      for(const [dx,dy] of DIRS){const nx=cx+dx, ny=cy+dy; if(nx<0||ny<0||nx>=w||ny>=h) continue; const ni=ny*w+nx; if(wall[ni]||comp[ni]!==-1||ny===hy) continue; comp[ni]=ncomp; q.push(ni);} }
    ncomp++;
  }
  type Room={cells:number[], boxes:number[], goals:number[], entry:number};
  const rooms: Room[]=[];
  let decompOk = ncomp>0;
  for(let c=0;c<ncomp && decompOk;c++){
    const cells:number[]=[]; for(let i=0;i<N;i++) if(comp[i]===c) cells.push(i);
    const rb=boxes.filter(b=>comp[b]===c);
    if(rb.length===0) continue;
    const rg=goals.filter(gl=>comp[gl]===c);
    if(rg.length!==rb.length){ decompOk=false; break; }
    // entry: hallway cell adjacent to this component
    let entry=-1;
    for(const cell of cells){ const cx=cell%w, cy=(cell/w)|0;
      for(const [dx,dy] of DIRS){const nx=cx+dx, ny=cy+dy; if(nx<0||ny<0||nx>=w||ny>=h) continue; if(ny===hy && !wall[ny*w+nx]){ entry=ny*w+nx; break; } }
      if(entry>=0) break;
    }
    if(entry<0){ decompOk=false; break; }
    rooms.push({cells, boxes:rb, goals:rg, entry});
  }
  if(decompOk && rooms.length>0 && rooms.reduce((a,r)=>a+r.boxes.length,0)===boxes.length){
    let cur=player; let curBoxes=new Set(boxes); let out='';
    let allOk=true;
    for(const room of rooms){
      const walkable=new Uint8Array(N); const boxOk=new Uint8Array(N);
      for(const cell of room.cells){ walkable[cell]=1; boxOk[cell]=1; }
      walkable[room.entry]=1; // player may step on entry, boxes may not enter it
      // also allow walking on hallway row cells within room solve? No: only entry.
      const sub: Grid={w,h,wall,goals:room.goals,goalSet:new Set(room.goals)};
      const sol=pushAStar(sub, room.entry, room.boxes, walkable, boxOk, 300000);
      if(sol===null){ allOk=false; break; }
      const wpath=walkPath(g, cur, room.entry, curBoxes);
      if(wpath===null){ allOk=false; break; }
      const seg=wpath+sol;
      const applied=applyMoves(g, cur, curBoxes, seg);
      if(!applied){ allOk=false; break; }
      cur=applied.player; curBoxes=applied.boxes; out+=seg;
    }
    if(allOk && [...curBoxes].every(b=>goalSet.has(b))) return out;
  }
  // --- fallback: whole-level push A* ---
  const walkable=new Uint8Array(N); const boxOk=new Uint8Array(N);
  for(let i=0;i<N;i++) if(!wall[i]){ walkable[i]=1; boxOk[i]=1; }
  const sol=pushAStar(g, player, boxes, walkable, boxOk, 120000);
  return sol ?? '';
}

for (const id of readdirSync(I).sort()) {
  let moves='';
  try { moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8')); } catch { moves=''; }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
