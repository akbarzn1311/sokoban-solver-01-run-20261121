// Sokoban solver for make.fun — "Build the best Sokoban solver, 21 November 2026".
//
// Design (from RULES.txt): every level is a hallway (one row, where the player
// starts) with 1-4 rooms off it, each room holding 3-4 boxes and as many goals,
// opening onto the hallway through exactly one door. A box pushed out through a
// door can never come back, so every box stays in its room and a level's
// push-optimal count P* is the sum of its rooms' optima. So:
//   1. detect the hallway row and split the floor into rooms (components once
//      the hallway row is removed; each has exactly one door cell);
//   2. solve each room independently with a push-optimal A* search over pushes
//      (player reachability by BFS, Hungarian assignment heuristic over
//      box-to-goal cell distances, dead-square + frozen-box pruning);
//   3. stitch the per-room move strings together with hallway walks between
//      doors (trying room orders until every walk, including the exit walk out
//      of each room, is possible).
// Deterministic by construction: fixed iteration orders, a monotonic
// tie-break counter in the heap, and work-based expansion caps only — the
// program never reads the wall clock or any environment value as a source of
// variation, so two runs produce byte-identical output.
import { readFileSync, writeFileSync, mkdirSync, readdirSync } from 'node:fs';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR || '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR || '/output';

// Directions in fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

// Work caps (deterministic): if A* ever exceeds the cap on a room, a greedy
// best-first search is tried as a fallback so the instance still gets an
// answer file instead of nothing.
const ASTAR_CAP = 3000000;
const GREEDY_CAP = 3000000;

// ---------------------------------------------------------------- parsing
function parseLevel(text) {
  const rows = text.replace(/\r/g, '').replace(/\n$/, '').split('\n');
  const H = rows.length;
  const W = Math.max(...rows.map((r) => r.length));
  const wall = new Array(W * H).fill(true);
  const boxes = new Set();
  const goals = new Set();
  let player = -1;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const c = rows[y][x] || ' ';
      const i = y * W + x;
      if (c === '#') continue;
      wall[i] = false;
      if (c === '$' || c === '*') boxes.add(i);
      if (c === '.' || c === '*') goals.add(i);
      if (c === '@') player = i;
    }
  }
  return { W, H, wall, boxes, goals, player };
}

// Split the floor into the hallway row (the player's row) and the rooms:
// connected components of floor cells once the hallway row is removed.
function detectRooms(level) {
  const { W, H, wall, player } = level;
  const hallRow = Math.floor(player / W);
  const seen = new Array(W * H).fill(false);
  const rooms = [];
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const i = y * W + x;
      if (y === hallRow || wall[i] || seen[i]) continue;
      const comp = [];
      const stack = [i];
      seen[i] = true;
      while (stack.length) {
        const c = stack.pop();
        comp.push(c);
        const cx = c % W, cy = Math.floor(c / W);
        for (let d = 0; d < 4; d++) {
          const nx = cx + DX[d], ny = cy + DY[d];
          if (ny === hallRow || nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
          const ni = ny * W + nx;
          if (wall[ni] || seen[ni]) continue;
          seen[ni] = true;
          stack.push(ni);
        }
      }
      rooms.push(buildRoom(level, comp, hallRow));
    }
  }
  return { hallRow, rooms };
}

function buildRoom(level, comp, hallRow) {
  const { W, wall, boxes, goals } = level;
  const n = comp.length;
  const indexOf = new Map();
  comp.forEach((cell, k) => indexOf.set(cell, k));
  // Neighbours of each room cell, in room-local indices; n means the entry
  // cell (the hallway cell the room opens onto), -1 means blocked.
  const neigh = [];
  let door = -1;
  let entry = -1;
  for (let k = 0; k < n; k++) {
    const cell = comp[k];
    const cx = cell % W, cy = Math.floor(cell / W);
    const row = [];
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= level.H) { row.push(-1); continue; }
      const ni = ny * W + nx;
      if (indexOf.has(ni)) row.push(indexOf.get(ni));
      else if (ny === hallRow && !wall[ni]) {
        row.push(n); // the entry cell, walkable by the player only
        door = k;
        entry = ni;
      } else row.push(-1);
    }
    neigh.push(row);
  }
  const roomBoxes = [];
  const roomGoals = [];
  for (let k = 0; k < n; k++) {
    if (boxes.has(comp[k])) roomBoxes.push(k);
    if (goals.has(comp[k])) roomGoals.push(k);
  }
  roomBoxes.sort((a, b) => a - b);
  roomGoals.sort((a, b) => a - b);
  // The entry cell (local index n): the player may stand on it, but its only
  // neighbour inside this room's world is the door cell. The hallway and any
  // other room are outside the room solver's domain, so they stay -1 here.
  const entryRow = [-1, -1, -1, -1];
  if (door >= 0) {
    const ex = entry % W, ey = Math.floor(entry / W);
    const dx0 = comp[door] % W, dy0 = Math.floor(comp[door] / W);
    for (let d = 0; d < 4; d++) {
      if (ex + DX[d] === dx0 && ey + DY[d] === dy0) entryRow[d] = door;
    }
  }
  neigh.push(entryRow);
  const room = { cells: comp, indexOf, n, neigh, door, entry, boxes: roomBoxes, goals: roomGoals };
  room.live = computeLive(room);
  room.dist = computeDist(room);
  return room;
}

// Live cells: a cell is live when a box on it can still be pushed (ignoring
// other boxes) onto a goal. Reverse BFS from the goals: a box on cell c could
// have come from p = c - d (pushed in direction d) when the player could stand
// on p - d, i.e. p - d is a room cell or the entry cell.
function computeLive(room) {
  const { n, neigh, goals } = room;
  const live = new Array(n).fill(false);
  const stack = [];
  for (const g of goals) { live[g] = true; stack.push(g); }
  while (stack.length) {
    const c = stack.pop();
    for (let d = 0; d < 4; d++) {
      const p = neigh[c][d ^ 1]; // cell the box came from (opposite direction)
      if (p < 0 || p >= n || live[p]) continue;
      // the player must have stood on the cell behind p in direction d
      const behind = neigh[p][d ^ 1];
      if (behind < 0) continue; // wall: no push into c from p was possible
      live[p] = true;
      stack.push(p);
    }
  }
  return live;
}

// All-pairs cell distances (BFS, ignoring boxes) between room cells.
function computeDist(room) {
  const { n, neigh } = room;
  const dist = [];
  for (let s = 0; s < n; s++) {
    const row = new Array(n).fill(Infinity);
    row[s] = 0;
    const q = [s];
    for (let h = 0; h < q.length; h++) {
      const c = q[h];
      for (let d = 0; d < 4; d++) {
        const nb = neigh[c][d];
        if (nb < 0 || nb >= n || row[nb] !== Infinity) continue;
        row[nb] = row[c] + 1;
        q.push(nb);
      }
    }
    dist.push(row);
  }
  return dist;
}

// Minimum-cost assignment of boxes to goals (Hungarian by brute force over
// the <= 4! permutations). Admissible: each push moves one box one cell, so
// the sum of the individual box-to-goal distances lower-bounds the pushes
// still needed. Infinity when some box cannot reach any goal.
function heuristic(room, boxes) {
  const { goals, dist } = room;
  const k = boxes.length;
  if (k === 0) return 0;
  let best = Infinity;
  const perm = goals.slice();
  // iterate all permutations of the goal list (k <= 4, so <= 24)
  const gen = (a, m) => {
    if (m === 1) {
      let s = 0;
      for (let i = 0; i < k; i++) {
        const d = dist[boxes[i]][a[i]];
        if (d === Infinity) return;
        s += d;
      }
      if (s < best) best = s;
      return;
    }
    for (let i = 0; i < m; i++) {
      gen(a, m - 1);
      const j = m % 2 === 1 ? 0 : i;
      const t = a[j]; a[j] = a[m - 1]; a[m - 1] = t;
    }
  };
  gen(perm, k);
  return best;
}

// ------------------------------------------------------------- BFS helpers
// Reachable cells for the player from `from`, over room cells + the entry
// cell, never stepping on a box cell. `blocked` is a boolean array over the
// n room cells marking box positions.
function reachable(room, from, blocked) {
  const { n, neigh } = room;
  const seen = new Array(n + 1).fill(false);
  seen[from] = true;
  const q = [from];
  for (let h = 0; h < q.length; h++) {
    const c = q[h];
    for (let d = 0; d < 4; d++) {
      const nb = neigh[c][d];
      if (nb < 0 || seen[nb]) continue;
      if (nb < n && blocked[nb]) continue;
      seen[nb] = true;
      q.push(nb);
    }
  }
  return seen;
}

// Shortest walk (as a move string) from `from` to `to` over room cells + the
// entry cell, avoiding box cells. Null when unreachable.
function walkPath(room, from, to, blocked) {
  if (from === to) return '';
  const { n, neigh } = room;
  const prev = new Array(n + 1).fill(-1);
  const prevDir = new Array(n + 1).fill(-1);
  prev[from] = from;
  const q = [from];
  for (let h = 0; h < q.length; h++) {
    const c = q[h];
    for (let d = 0; d < 4; d++) {
      const nb = neigh[c][d];
      if (nb < 0 || prev[nb] !== -1) continue;
      if (nb < n && blocked[nb]) continue;
      prev[nb] = c;
      prevDir[nb] = d;
      if (nb === to) {
        let s = '';
        let cur = to;
        while (cur !== from) { s = CH[prevDir[cur]] + s; cur = prev[cur]; }
        return s;
      }
      q.push(nb);
    }
  }
  return null;
}

// Frozen boxes: a box is frozen when it can never move again, given walls and
// the other frozen boxes as blockers. Fixed point over the box set. A frozen
// box not on a goal is a deadlock.
function hasFrozenDeadlock(room, boxes) {
  const { n, neigh, goals } = room;
  const boxSet = new Set(boxes);
  const goalSet = new Set(goals);
  const frozen = new Set();
  let changed = true;
  while (changed) {
    changed = false;
    for (const b of boxes) {
      if (frozen.has(b) || goalSet.has(b)) continue;
      let stuck = true;
      for (let d = 0; d < 4; d++) {
        const target = neigh[b][d];
        const behind = neigh[b][d ^ 1];
        const targetBlocked = target < 0 || target >= n ? true : (boxSet.has(target) && frozen.has(target));
        const behindBlocked = behind < 0 ? true : (behind < n && boxSet.has(behind) && frozen.has(behind));
        if (!targetBlocked && !behindBlocked) { stuck = false; break; }
      }
      if (stuck) { frozen.add(b); changed = true; }
    }
  }
  for (const b of frozen) if (!goalSet.has(b)) return true;
  return false;
}

// ------------------------------------------------------------ room solver
function cmpNode(x, y) {
  if (x.f !== y.f) return x.f - y.f;
  if (x.h !== y.h) return x.h - y.h;
  return x.seq - y.seq;
}

class Heap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(node) {
    const a = this.a;
    a.push(node);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmpNode(a[i], a[p]) < 0) { const t = a[i]; a[i] = a[p]; a[p] = t; i = p; } else break;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1, r = l + 1;
        let m = i;
        if (l < a.length && cmpNode(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmpNode(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        const t = a[i]; a[i] = a[m]; a[m] = t; i = m;
      }
    }
    return top;
  }
}

// Canonical state key. Boxes are sorted, so the key is injective for a fixed
// box count; the player position is the high digit. Numeric while the room
// is small (a 6x6 slot is at most 36 cells), string otherwise.
function keyOf(room, boxes, player) {
  const k = boxes.length;
  if (room.n < 60) {
    let key = player;
    let mult = 64;
    for (let i = 0; i < k; i++) { key += boxes[i] * mult; mult *= 64; }
    return key;
  }
  return player + '|' + boxes.join(',');
}

// Push-optimal A* over pushes (greedy=false) or best-first by heuristic
// (greedy=true, the fallback). Returns { moves, pushes, finalPlayer } with
// finalPlayer a room-local cell index (n = the entry cell), or null.
function solveRoom(room, greedy) {
  const { n, neigh, boxes: boxes0, goals, live } = room;
  const k = boxes0.length;
  const goalSet = new Set(goals);
  const cap = greedy ? GREEDY_CAP : ASTAR_CAP;
  const startBoxes = boxes0.slice();
  const h0 = heuristic(room, startBoxes);
  if (h0 === Infinity) return null;
  const heap = new Heap();
  const best = new Map();
  const parent = new Map();
  let seq = 0;
  const startKey = keyOf(room, startBoxes, n);
  best.set(startKey, 0);
  heap.push({ f: h0, g: 0, h: h0, seq: seq++, key: startKey, boxes: startBoxes, player: n });
  let expansions = 0;
  while (heap.size) {
    const node = heap.pop();
    if (node.g > best.get(node.key)) continue;
    if (node.boxes.every((b) => goalSet.has(b))) {
      const parts = [];
      let cur = node.key;
      while (parent.has(cur)) {
        const step = parent.get(cur);
        parts.push(step.moves);
        cur = step.prev;
      }
      parts.reverse();
      return { moves: parts.join(''), pushes: node.g, finalPlayer: node.player };
    }
    if (++expansions > cap) return null;
    const blocked = new Array(n).fill(false);
    for (const b of node.boxes) blocked[b] = true;
    const reach = reachable(room, node.player, blocked);
    for (let bi = 0; bi < k; bi++) {
      const b = node.boxes[bi];
      for (let d = 0; d < 4; d++) {
        const t = neigh[b][d];
        if (t < 0 || t >= n) continue; // wall, or out of the room (never push out)
        if (blocked[t]) continue;
        const behind = neigh[b][d ^ 1];
        if (behind < 0 || !reach[behind]) continue;
        const nb = node.boxes.slice();
        nb[bi] = t;
        nb.sort((x, y) => x - y);
        if (nb.some((x) => !live[x])) continue; // a box can never reach a goal
        if (hasFrozenDeadlock(room, nb)) continue;
        const h = heuristic(room, nb);
        if (h === Infinity) continue;
        const walk = walkPath(room, node.player, behind, blocked);
        if (walk === null) continue;
        const g2 = node.g + 1;
        const key2 = keyOf(room, nb, b); // the player ends on the box's old cell
        const prevBest = best.get(key2);
        if (prevBest !== undefined && prevBest <= g2) continue;
        best.set(key2, g2);
        parent.set(key2, { prev: node.key, moves: walk + CH[d] });
        heap.push({ f: greedy ? h : g2 + h, g: g2, h, seq: seq++, key: key2, boxes: nb, player: b });
      }
    }
  }
  return null;
}

// --------------------------------------------------------------- stitching
// Shortest walk (move string) from one level cell to another over floor
// cells, never stepping on a cell in `occupied`. Null when unreachable.
function levelWalk(level, from, to, occupied) {
  if (from === to) return '';
  const { W, H, wall } = level;
  const prev = new Array(W * H).fill(-1);
  const prevDir = new Array(W * H).fill(-1);
  prev[from] = from;
  const q = [from];
  for (let h = 0; h < q.length; h++) {
    const c = q[h];
    const cx = c % W, cy = Math.floor(c / W);
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const ni = ny * W + nx;
      if (wall[ni] || occupied.has(ni) || prev[ni] !== -1) continue;
      prev[ni] = c;
      prevDir[ni] = d;
      if (ni === to) {
        let s = '';
        let cur = to;
        while (cur !== from) { s = CH[prevDir[cur]] + s; cur = prev[cur]; }
        return s;
      }
      q.push(ni);
    }
  }
  return null;
}

function permutations(arr) {
  if (arr.length <= 1) return [arr.slice()];
  const out = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    for (const p of permutations(rest)) out.push([arr[i], ...p]);
  }
  return out;
}

// Try one room order: walk to each room's door, play its solution, walk out.
// Returns the full move string, or null when some walk is impossible (e.g.
// the player cannot leave a room whose door cell ends up blocked by a box —
// that room then has to come last, which another permutation tries).
function tryStitch(level, rooms, solved, perm) {
  const occupied = new Set(level.boxes);
  let cur = level.player;
  let out = '';
  for (const ri of perm) {
    const room = rooms[ri];
    const sol = solved[ri];
    const walk = levelWalk(level, cur, room.entry, occupied);
    if (walk === null) return null;
    out += walk;
    out += sol.moves;
    for (const b of room.boxes) occupied.delete(room.cells[b]);
    for (const g of room.goals) occupied.add(room.cells[g]);
    cur = sol.finalPlayer === room.n ? room.entry : room.cells[sol.finalPlayer];
  }
  return out;
}

function solveLevel(text) {
  const level = parseLevel(text);
  const { rooms } = detectRooms(level);
  const solved = [];
  for (const room of rooms) {
    let res = solveRoom(room, false);
    if (!res) res = solveRoom(room, true);
    solved.push(res);
  }
  if (solved.some((s) => s === null)) return '';
  const order = rooms.map((_, i) => i);
  for (const perm of permutations(order)) {
    const res = tryStitch(level, rooms, solved, perm);
    if (res !== null) return res;
  }
  return '';
}

// ------------------------------------------------------------------- main
function main() {
  let ids = ['i001', 'i002', 'i003', 'i004', 'i005'];
  try {
    const found = readdirSync(INPUT_DIR).filter((n) => /^i[0-9]+$/.test(n)).sort();
    if (found.length) ids = found;
  } catch (e) { /* fall back to the fixed list */ }
  for (const id of ids) {
    let moves = '';
    try {
      const text = readFileSync(`${INPUT_DIR}/${id}/level.txt`, 'utf8');
      moves = solveLevel(text);
    } catch (e) {
      moves = '';
    }
    const dir = `${OUTPUT_DIR}/${id}`;
    mkdirSync(dir, { recursive: true });
    writeFileSync(`${dir}/solution.txt`, moves, 'utf8');
  }
}

main();
