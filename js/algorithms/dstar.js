// ══════════════════════════════════════════════════════════════
// D* LITE PATHFINDING — Dynamic A* Lite (Koenig & Likhachev 2002)
// Searches BACKWARD from goal → start.
// In static environments produces optimal paths like A*.
// In dynamic environments (obstacles added/removed) it can
// efficiently replan by updating only affected nodes.
//
// Key concepts:
//   g(s)   = current cost estimate (backward from goal)
//   rhs(s) = one-step lookahead (locally consistent if g == rhs)
//   key(s) = [min(g,rhs)+h(start,s), min(g,rhs)]
// ══════════════════════════════════════════════════════════════

function dstarFindPath(start, goal, octree) {
  const w = octree.w;
  const IDX = (x, y, z) => x * w * w + y * w + z;
  const sx = Math.round(start.x), sy = Math.round(start.y), sz = Math.round(start.z);
  const gx = Math.round(goal.x),  gy = Math.round(goal.y),  gz = Math.round(goal.z);

  if (octree.isObstacle(sx, sy, sz) || octree.isObstacle(gx, gy, gz)) return null;

  const total    = w * w * w;
  const INF      = 1e9;
  const g        = new Float64Array(total).fill(INF);
  const rhs      = new Float64Array(total).fill(INF);
  const inQueue  = new Uint8Array(total);
  // key1, key2 stored separately for efficiency
  const key1     = new Float64Array(total).fill(INF);
  const key2     = new Float64Array(total).fill(INF);

  const startIdx = IDX(sx, sy, sz);
  const goalIdx  = IDX(gx, gy, gz);

  // k_m is the accumulated heuristic shift (0 for static env)
  let km = 0;

  const decode = idx => ({
    x: Math.floor(idx / (w * w)),
    y: Math.floor((idx % (w * w)) / w),
    z: idx % w
  });

  const h = (idx) => {
    const { x, y, z } = decode(idx);
    return heuristic(sx, sy, sz, x, y, z);
  };

  const calcKey = (idx) => {
    const minGRhs = Math.min(g[idx], rhs[idx]);
    return [minGRhs + h(idx) + km, minGRhs];
  };

  // Priority queue with lexicographic key comparison
  const pq = new MinHeap((a, b) => {
    if (a.k1 !== b.k1) return a.k1 - b.k1;
    return a.k2 - b.k2;
  });

  const insertOrUpdate = (idx) => {
    const [k1v, k2v] = calcKey(idx);
    key1[idx] = k1v;
    key2[idx] = k2v;
    if (!inQueue[idx]) {
      inQueue[idx] = 1;
      pq.push({ idx, k1: k1v, k2: k2v });
    } else {
      // Lazy update: push a new entry (old one will be skipped)
      pq.push({ idx, k1: k1v, k2: k2v });
    }
  };

  // Successors/predecessors are the same (symmetric grid)
  const getNeighbors = (idx) => {
    const { x, y, z } = decode(idx);
    const result = [];
    for (const { dx, dy, dz, cost } of DIRS_26) {
      const nx = x + dx, ny = y + dy, nz = z + dz;
      if (octree.inBounds(nx, ny, nz) && !octree.isObstacle(nx, ny, nz)) {
        result.push({ idx: IDX(nx, ny, nz), cost });
      }
    }
    return result;
  };

  // ── Initialize: rhs(goal) = 0, all others = INF ──
  rhs[goalIdx] = 0;
  insertOrUpdate(goalIdx);

  // ── UpdateVertex: recalculate rhs and reinsert if needed ──
  const updateVertex = (u) => {
    if (u !== goalIdx) {
      let minVal = INF;
      for (const { idx: s, cost } of getNeighbors(u)) {
        const v = g[s] + cost;
        if (v < minVal) minVal = v;
      }
      rhs[u] = minVal;
    }
    inQueue[u] = 0; // Will be re-inserted if inconsistent
    if (g[u] !== rhs[u]) {
      insertOrUpdate(u);
    }
  };

  // ── ComputeShortestPath ──
  const computeShortestPath = () => {
    let iterations = 0;
    const maxIter = total * 4;

    while (!pq.isEmpty() && iterations++ < maxIter) {
      const top = pq.peek();
      const u   = top.idx;

      // Check if this entry is stale (lazy deletion)
      const [ck1, ck2] = calcKey(u);
      if (top.k1 !== key1[u] || top.k2 !== key2[u]) {
        pq.pop();
        continue;
      }

      // Check termination condition
      const [sk1, sk2] = calcKey(startIdx);
      const topK1 = top.k1, topK2 = top.k2;
      if ((topK1 > sk1 || (topK1 === sk1 && topK2 >= sk2)) &&
          rhs[startIdx] === g[startIdx]) break;

      pq.pop();
      inQueue[u] = 0;

      if (g[u] > rhs[u]) {
        // Overconsistent: make consistent
        g[u] = rhs[u];
        for (const { idx: s } of getNeighbors(u)) {
          updateVertex(s);
        }
      } else {
        // Underconsistent: raise g
        g[u] = INF;
        updateVertex(u);
        for (const { idx: s } of getNeighbors(u)) {
          updateVertex(s);
        }
      }
    }
  };

  computeShortestPath();

  // ── Check if path exists ──
  if (g[startIdx] === INF) return null;

  // ── Extract path: greedy descent from start to goal ──
  const path = [];
  let cur = startIdx;
  let steps = 0;
  const maxSteps = total;

  while (cur !== goalIdx && steps++ < maxSteps) {
    path.push(decode(cur));
    const neighbors = getNeighbors(cur);
    if (neighbors.length === 0) return null;

    let best = null, bestCost = INF;
    for (const { idx: ni, cost } of neighbors) {
      const val = g[ni] + cost;
      if (val < bestCost) {
        bestCost = val;
        best = ni;
      }
    }
    if (best === null || bestCost >= INF) return null;
    cur = best;
  }
  path.push(decode(goalIdx));

  return path;
}
