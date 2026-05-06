// ══════════════════════════════════════════════════════════════
// JUMP POINT SEARCH (JPS) 3D — Harabor & Grastien extended to 3D
// JPS prunes the search space by identifying "jump points" —
// nodes where the optimal path must branch. Between jump points,
// we skip entire straight-line runs without expanding individual nodes.
//
// Move types in 3D:
//   Cardinal  (1 axis): 6 directions
//   Diagonal2D (2 axes): 12 directions
//   Diagonal3D (3 axes): 8 directions
//
// Each type has specific pruning rules to eliminate redundant neighbors.
// ══════════════════════════════════════════════════════════════

function jpsFindPath(start, goal, octree) {
  const w   = octree.w;
  const IDX = (x, y, z) => x * w * w + y * w + z;
  const sx  = Math.round(start.x), sy = Math.round(start.y), sz = Math.round(start.z);
  const gx  = Math.round(goal.x),  gy = Math.round(goal.y),  gz = Math.round(goal.z);

  if (octree.isObstacle(sx, sy, sz) || octree.isObstacle(gx, gy, gz)) return null;

  const total  = w * w * w;
  const INF    = 1e9;
  const gScore = new Float32Array(total).fill(INF);
  const fScore = new Float32Array(total).fill(INF);
  const closed = new Uint8Array(total);
  // parentDir[i] = direction index that led to node i
  const parentNode = new Int32Array(total).fill(-2);

  const startIdx = IDX(sx, sy, sz);
  const goalIdx  = IDX(gx, gy, gz);

  gScore[startIdx] = 0;
  fScore[startIdx] = heuristic(sx, sy, sz, gx, gy, gz);
  parentNode[startIdx] = -1;

  const open = new MinHeap((a, b) => a.f - b.f);
  open.push({ idx: startIdx, f: fScore[startIdx], dx: 0, dy: 0, dz: 0 });

  const decode = idx => ({
    x: Math.floor(idx / (w * w)),
    y: Math.floor((idx % (w * w)) / w),
    z: idx % w
  });

  const W = (x, y, z) => octree.isObstacle(x, y, z); // obstacle check shorthand
  const B = (x, y, z) => octree.inBounds(x, y, z);

  // ── Identify forced neighbors for a cardinal move (dx,0,0) type ──
  // Returns list of {fdx, fdy, fdz} forced neighbor directions
  function forcedNeighborsCardinal(x, y, z, dx, dy, dz) {
    const forced = [];
    if (dx !== 0) {
      // Moving along X axis
      if (B(x, y+1, z) && W(x-dx, y+1, z) && !W(x, y+1, z)) forced.push({fdx: dx, fdy: 1, fdz: 0});
      if (B(x, y-1, z) && W(x-dx, y-1, z) && !W(x, y-1, z)) forced.push({fdx: dx, fdy:-1, fdz: 0});
      if (B(x, y, z+1) && W(x-dx, y, z+1) && !W(x, y, z+1)) forced.push({fdx: dx, fdy: 0, fdz: 1});
      if (B(x, y, z-1) && W(x-dx, y, z-1) && !W(x, y, z-1)) forced.push({fdx: dx, fdy: 0, fdz:-1});
      // Diagonal forced (corners of obstacles)
      if (B(x, y+1, z+1) && (W(x-dx, y+1, z+1)||W(x-dx,y+1,z)||W(x-dx,y,z+1)) && !W(x, y+1, z+1))
        forced.push({fdx: dx, fdy: 1, fdz: 1});
      if (B(x, y-1, z+1) && (W(x-dx, y-1, z+1)||W(x-dx,y-1,z)||W(x-dx,y,z+1)) && !W(x, y-1, z+1))
        forced.push({fdx: dx, fdy:-1, fdz: 1});
      if (B(x, y+1, z-1) && (W(x-dx, y+1, z-1)||W(x-dx,y+1,z)||W(x-dx,y,z-1)) && !W(x, y+1, z-1))
        forced.push({fdx: dx, fdy: 1, fdz:-1});
      if (B(x, y-1, z-1) && (W(x-dx, y-1, z-1)||W(x-dx,y-1,z)||W(x-dx,y,z-1)) && !W(x, y-1, z-1))
        forced.push({fdx: dx, fdy:-1, fdz:-1});
    } else if (dy !== 0) {
      // Moving along Y axis
      if (B(x+1, y, z) && W(x+1, y-dy, z) && !W(x+1, y, z)) forced.push({fdx: 1, fdy: dy, fdz: 0});
      if (B(x-1, y, z) && W(x-1, y-dy, z) && !W(x-1, y, z)) forced.push({fdx:-1, fdy: dy, fdz: 0});
      if (B(x, y, z+1) && W(x, y-dy, z+1) && !W(x, y, z+1)) forced.push({fdx: 0, fdy: dy, fdz: 1});
      if (B(x, y, z-1) && W(x, y-dy, z-1) && !W(x, y, z-1)) forced.push({fdx: 0, fdy: dy, fdz:-1});
      if (B(x+1, y, z+1) && (W(x+1,y-dy,z+1)||W(x+1,y-dy,z)||W(x,y-dy,z+1)) && !W(x+1,y,z+1))
        forced.push({fdx: 1, fdy: dy, fdz: 1});
      if (B(x-1, y, z+1) && (W(x-1,y-dy,z+1)||W(x-1,y-dy,z)||W(x,y-dy,z+1)) && !W(x-1,y,z+1))
        forced.push({fdx:-1, fdy: dy, fdz: 1});
      if (B(x+1, y, z-1) && (W(x+1,y-dy,z-1)||W(x+1,y-dy,z)||W(x,y-dy,z-1)) && !W(x+1,y,z-1))
        forced.push({fdx: 1, fdy: dy, fdz:-1});
      if (B(x-1, y, z-1) && (W(x-1,y-dy,z-1)||W(x-1,y-dy,z)||W(x,y-dy,z-1)) && !W(x-1,y,z-1))
        forced.push({fdx:-1, fdy: dy, fdz:-1});
    } else {
      // Moving along Z axis
      if (B(x+1, y, z) && W(x+1, y, z-dz) && !W(x+1, y, z)) forced.push({fdx: 1, fdy: 0, fdz: dz});
      if (B(x-1, y, z) && W(x-1, y, z-dz) && !W(x-1, y, z)) forced.push({fdx:-1, fdy: 0, fdz: dz});
      if (B(x, y+1, z) && W(x, y+1, z-dz) && !W(x, y+1, z)) forced.push({fdx: 0, fdy: 1, fdz: dz});
      if (B(x, y-1, z) && W(x, y-1, z-dz) && !W(x, y-1, z)) forced.push({fdx: 0, fdy:-1, fdz: dz});
      if (B(x+1, y+1, z) && (W(x+1,y+1,z-dz)||W(x+1,y,z-dz)||W(x,y+1,z-dz)) && !W(x+1,y+1,z))
        forced.push({fdx: 1, fdy: 1, fdz: dz});
      if (B(x-1, y+1, z) && (W(x-1,y+1,z-dz)||W(x-1,y,z-dz)||W(x,y+1,z-dz)) && !W(x-1,y+1,z))
        forced.push({fdx:-1, fdy: 1, fdz: dz});
      if (B(x+1, y-1, z) && (W(x+1,y-1,z-dz)||W(x+1,y,z-dz)||W(x,y-1,z-dz)) && !W(x+1,y-1,z))
        forced.push({fdx: 1, fdy:-1, fdz: dz});
      if (B(x-1, y-1, z) && (W(x-1,y-1,z-dz)||W(x-1,y,z-dz)||W(x,y-1,z-dz)) && !W(x-1,y-1,z))
        forced.push({fdx:-1, fdy:-1, fdz: dz});
    }
    return forced;
  }

  // ── Get pruned neighbors for a node at (x,y,z) coming from direction (dx,dy,dz) ──
  function prunedNeighbors(x, y, z, dx, dy, dz) {
    const dirs = [];
    const dxA = dx !== 0 ? 1 : 0, dyA = dy !== 0 ? 1 : 0, dzA = dz !== 0 ? 1 : 0;
    const axisCount = dxA + dyA + dzA;

    if (axisCount === 0) {
      // Start node: explore all 26 directions
      return DIRS_26.map(d => ({ dx: d.dx, dy: d.dy, dz: d.dz }));
    }

    if (axisCount === 1) {
      // Cardinal move: natural neighbor + forced neighbors
      if (!W(x+dx, y+dy, z+dz) && B(x+dx, y+dy, z+dz))
        dirs.push({ dx, dy, dz });
      const forced = forcedNeighborsCardinal(x, y, z, dx, dy, dz);
      dirs.push(...forced.map(f => ({ dx: f.fdx, dy: f.fdy, dz: f.fdz })));
    }

    if (axisCount === 2) {
      // Diagonal 2D move (e.g., dx=1, dy=1, dz=0)
      // Natural: diagonal direction + two component cardinals
      if (!W(x+dx, y+dy, z+dz) && B(x+dx, y+dy, z+dz)) dirs.push({ dx, dy, dz });
      if (dx !== 0 && !W(x+dx, y, z) && B(x+dx, y, z)) dirs.push({ dx, dy: 0, dz: 0 });
      if (dy !== 0 && !W(x, y+dy, z) && B(x, y+dy, z)) dirs.push({ dx: 0, dy, dz: 0 });
      if (dz !== 0 && !W(x, y, z+dz) && B(x, y, z+dz)) dirs.push({ dx: 0, dy: 0, dz });

      // Forced neighbors for diagonal 2D
      if (dx !== 0 && dy !== 0) {
        if (W(x, y, z+1) && B(x+dx, y+dy, z+1)) dirs.push({ dx, dy, dz: 1 });
        if (W(x, y, z-1) && B(x+dx, y+dy, z-1)) dirs.push({ dx, dy, dz: -1 });
      } else if (dx !== 0 && dz !== 0) {
        if (W(x, y+1, z) && B(x+dx, y+1, z+dz)) dirs.push({ dx, dy: 1, dz });
        if (W(x, y-1, z) && B(x+dx, y-1, z+dz)) dirs.push({ dx, dy: -1, dz });
      } else if (dy !== 0 && dz !== 0) {
        if (W(x+1, y, z) && B(x+1, y+dy, z+dz)) dirs.push({ dx: 1, dy, dz });
        if (W(x-1, y, z) && B(x-1, y+dy, z+dz)) dirs.push({ dx: -1, dy, dz });
      }
    }

    if (axisCount === 3) {
      // Diagonal 3D: natural + six 2D diagonals + three cardinals
      if (!W(x+dx, y+dy, z+dz) && B(x+dx, y+dy, z+dz)) dirs.push({ dx, dy, dz });
      if (!W(x+dx, y, z) && B(x+dx, y, z))           dirs.push({ dx, dy: 0, dz: 0 });
      if (!W(x, y+dy, z) && B(x, y+dy, z))           dirs.push({ dx: 0, dy, dz: 0 });
      if (!W(x, y, z+dz) && B(x, y, z+dz))           dirs.push({ dx: 0, dy: 0, dz });
      if (!W(x+dx, y+dy, z) && B(x+dx, y+dy, z))     dirs.push({ dx, dy, dz: 0 });
      if (!W(x+dx, y, z+dz) && B(x+dx, y, z+dz))     dirs.push({ dx, dy: 0, dz });
      if (!W(x, y+dy, z+dz) && B(x, y+dy, z+dz))     dirs.push({ dx: 0, dy, dz });

      // Forced for 3D diagonal
      if (W(x-dx, y, z)) {
        if (B(x-dx, y+dy, z)) dirs.push({ dx: -dx, dy, dz: 0 });
        if (B(x-dx, y, z+dz)) dirs.push({ dx: -dx, dy: 0, dz });
        if (B(x-dx, y+dy, z+dz)) dirs.push({ dx: -dx, dy, dz });
      }
      if (W(x, y-dy, z)) {
        if (B(x+dx, y-dy, z)) dirs.push({ dx, dy: -dy, dz: 0 });
        if (B(x, y-dy, z+dz)) dirs.push({ dx: 0, dy: -dy, dz });
        if (B(x+dx, y-dy, z+dz)) dirs.push({ dx, dy: -dy, dz });
      }
      if (W(x, y, z-dz)) {
        if (B(x+dx, y, z-dz)) dirs.push({ dx, dy: 0, dz: -dz });
        if (B(x, y+dy, z-dz)) dirs.push({ dx: 0, dy, dz: -dz });
        if (B(x+dx, y+dy, z-dz)) dirs.push({ dx, dy, dz: -dz });
      }
    }

    return dirs;
  }

  // ── Recursive jump function ──
  // Returns the jump point coordinate or null
  let jumpCalls = 0;
  const maxJumpCalls = w * w * w * 2;

  function jump(x, y, z, dx, dy, dz) {
    if (jumpCalls++ > maxJumpCalls) return null;
    const nx = x + dx, ny = y + dy, nz = z + dz;
    if (!B(nx, ny, nz) || W(nx, ny, nz)) return null;

    // Found the goal
    if (nx === gx && ny === gy && nz === gz) return { x: nx, y: ny, z: nz };

    const dxA = dx !== 0 ? 1 : 0, dyA = dy !== 0 ? 1 : 0, dzA = dz !== 0 ? 1 : 0;
    const axisCount = dxA + dyA + dzA;

    // Check for forced neighbors at (nx, ny, nz)
    if (axisCount === 1) {
      const forced = forcedNeighborsCardinal(nx, ny, nz, dx, dy, dz);
      if (forced.length > 0) return { x: nx, y: ny, z: nz };
    }

    if (axisCount === 2) {
      // Jump in each component cardinal direction
      if (dx !== 0 && jump(nx, ny, nz, dx, 0, 0) !== null) return { x: nx, y: ny, z: nz };
      if (dy !== 0 && jump(nx, ny, nz, 0, dy, 0) !== null) return { x: nx, y: ny, z: nz };
      if (dz !== 0 && jump(nx, ny, nz, 0, 0, dz) !== null) return { x: nx, y: ny, z: nz };
    }

    if (axisCount === 3) {
      // Jump in each component cardinal direction
      if (jump(nx, ny, nz, dx, 0, 0) !== null) return { x: nx, y: ny, z: nz };
      if (jump(nx, ny, nz, 0, dy, 0) !== null) return { x: nx, y: ny, z: nz };
      if (jump(nx, ny, nz, 0, 0, dz) !== null) return { x: nx, y: ny, z: nz };
      // Jump in 2D diagonal components
      if (jump(nx, ny, nz, dx, dy, 0) !== null) return { x: nx, y: ny, z: nz };
      if (jump(nx, ny, nz, dx, 0, dz) !== null) return { x: nx, y: ny, z: nz };
      if (jump(nx, ny, nz, 0, dy, dz) !== null) return { x: nx, y: ny, z: nz };
    }

    return jump(nx, ny, nz, dx, dy, dz);
  }

  // ── Main JPS loop: A* framework with jump point identification ──
  while (!open.isEmpty()) {
    const { idx: cur, dx: cdx, dy: cdy, dz: cdz } = open.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;

    if (cur === goalIdx) {
      // Reconstruct path by following parent chain
      const path = [];
      let c = cur;
      let safety = 0;
      while (c !== -1 && safety++ < total) {
        const coord = decode(c);
        // If previous path node exists and is not adjacent, interpolate
        if (path.length > 0) {
          const prev = path[path.length - 1];
          const steps = Math.max(
            Math.abs(coord.x - prev.x),
            Math.abs(coord.y - prev.y),
            Math.abs(coord.z - prev.z)
          );
          if (steps > 1) {
            const stepX = (coord.x - prev.x) / steps;
            const stepY = (coord.y - prev.y) / steps;
            const stepZ = (coord.z - prev.z) / steps;
            for (let s = 1; s < steps; s++) {
              path.push({
                x: Math.round(prev.x + stepX * s),
                y: Math.round(prev.y + stepY * s),
                z: Math.round(prev.z + stepZ * s)
              });
            }
          }
        }
        path.push(coord);
        c = parentNode[c];
      }
      path.reverse();
      return path;
    }

    const { x: cx, y: cy, z: cz } = decode(cur);
    const neighbors = prunedNeighbors(cx, cy, cz, cdx, cdy, cdz);

    for (const { dx, dy, dz } of neighbors) {
      jumpCalls = 0; // Reset per neighbor direction
      const jp = jump(cx, cy, cz, dx, dy, dz);
      if (!jp) continue;

      const ni = IDX(jp.x, jp.y, jp.z);
      if (closed[ni]) continue;

      const moveCost = heuristic(cx, cy, cz, jp.x, jp.y, jp.z);
      const ng = gScore[cur] + moveCost;
      if (ng < gScore[ni]) {
        gScore[ni] = ng;
        fScore[ni] = ng + heuristic(jp.x, jp.y, jp.z, gx, gy, gz);
        parentNode[ni] = cur;
        open.push({ idx: ni, f: fScore[ni], dx, dy, dz });
      }
    }
  }

  return null;
}
