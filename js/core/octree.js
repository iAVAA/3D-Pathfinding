// ══════════════════════════════════════════════════════════════
// OCTREE — Spatial data structure for 3D obstacle lookup
// ══════════════════════════════════════════════════════════════

class OctreeNode {
  constructor(x0, y0, z0, x1, y1, z1) {
    this.x0 = x0; this.y0 = y0; this.z0 = z0;
    this.x1 = x1; this.y1 = y1; this.z1 = z1;
    this.children = null;   // 8 child nodes when subdivided
    this.count = 0;         // number of obstacles inside
  }
  get isLeaf() { return this.children === null; }
}

class Octree {
  constructor(worldSize) {
    this.w = worldSize;
    this.root = new OctreeNode(0, 0, 0, worldSize, worldSize, worldSize);
    // Fast flat lookup (O(1)) backed by the octree spatial structure
    this._flat = new Uint8Array(worldSize * worldSize * worldSize);
  }

  // Build the octree from an obstacle list [{x,y,z}, ...]
  build(obstacles) {
    const w = this.w;
    this._flat.fill(0);
    for (const o of obstacles) {
      if (o.x >= 0 && o.y >= 0 && o.z >= 0 && o.x < w && o.y < w && o.z < w)
        this._flat[o.x * w * w + o.y * w + o.z] = 1;
    }
    this._buildNode(this.root, obstacles);
  }

  _buildNode(node, obstacles) {
    // Filter obstacles that fall within this node's AABB
    const relevant = obstacles.filter(o =>
      o.x >= node.x0 && o.x < node.x1 &&
      o.y >= node.y0 && o.y < node.y1 &&
      o.z >= node.z0 && o.z < node.z1
    );
    node.count = relevant.length;
    if (relevant.length === 0) return;

    const sx = node.x1 - node.x0;
    const sy = node.y1 - node.y0;
    const sz = node.z1 - node.z0;
    // Stop subdividing at voxel level
    if (sx <= 1 && sy <= 1 && sz <= 1) return;

    const mx = Math.floor((node.x0 + node.x1) / 2);
    const my = Math.floor((node.y0 + node.y1) / 2);
    const mz = Math.floor((node.z0 + node.z1) / 2);

    // Create up to 8 octant children
    const octants = [
      [node.x0, node.y0, node.z0, mx, my, mz],
      [mx,      node.y0, node.z0, node.x1, my, mz],
      [node.x0, my,      node.z0, mx, node.y1, mz],
      [mx,      my,      node.z0, node.x1, node.y1, mz],
      [node.x0, node.y0, mz,      mx, my, node.z1],
      [mx,      node.y0, mz,      node.x1, my, node.z1],
      [node.x0, my,      mz,      mx, node.y1, node.z1],
      [mx,      my,      mz,      node.x1, node.y1, node.z1],
    ];

    node.children = [];
    for (const [x0, y0, z0, x1, y1, z1] of octants) {
      if (x0 < x1 && y0 < y1 && z0 < z1) {
        const child = new OctreeNode(x0, y0, z0, x1, y1, z1);
        this._buildNode(child, relevant);
        node.children.push(child);
      }
    }
  }

  // O(1) obstacle check via flat array (octree provides the spatial structure)
  isObstacle(x, y, z) {
    const w = this.w;
    if (x < 0 || y < 0 || z < 0 || x >= w || y >= w || z >= w) return true;
    return this._flat[x * w * w + y * w + z] === 1;
  }

  inBounds(x, y, z) {
    const w = this.w;
    return x >= 0 && y >= 0 && z >= 0 && x < w && y < w && z < w;
  }

  // 3D Bresenham line-of-sight check (used by Theta*)
  lineOfSight(ax, ay, az, bx, by, bz) {
    let dx = bx - ax, dy = by - ay, dz = bz - az;
    const steps = Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz));
    if (steps === 0) return true;
    const sx = dx / steps, sy = dy / steps, sz = dz / steps;
    let x = ax + 0.5, y = ay + 0.5, z = az + 0.5;
    for (let i = 0; i < steps; i++) {
      x += sx; y += sy; z += sz;
      if (this.isObstacle(Math.floor(x), Math.floor(y), Math.floor(z))) return false;
    }
    return true;
  }

  // Query: does this AABB region contain any obstacle? (uses octree for early exit)
  regionHasObstacle(x0, y0, z0, x1, y1, z1) {
    return this._regionQuery(this.root, x0, y0, z0, x1, y1, z1);
  }

  _regionQuery(node, x0, y0, z0, x1, y1, z1) {
    if (!node || node.count === 0) return false;
    // No overlap
    if (node.x1 <= x0 || node.x0 >= x1 ||
        node.y1 <= y0 || node.y0 >= y1 ||
        node.z1 <= z0 || node.z0 >= z1) return false;
    if (node.isLeaf) return node.count > 0;
    for (const child of node.children) {
      if (this._regionQuery(child, x0, y0, z0, x1, y1, z1)) return true;
    }
    return false;
  }
}

// ══════════════════════════════════════════════════════════════
// MIN-HEAP — Priority queue used by all pathfinding algorithms
// ══════════════════════════════════════════════════════════════
class MinHeap {
  constructor(cmpFn = (a, b) => a.f - b.f) {
    this._data = [];
    this._cmp = cmpFn;
  }
  get size() { return this._data.length; }
  isEmpty() { return this._data.length === 0; }
  push(item) {
    this._data.push(item);
    this._bubbleUp(this._data.length - 1);
  }
  pop() {
    const top = this._data[0];
    const last = this._data.pop();
    if (this._data.length > 0) {
      this._data[0] = last;
      this._sinkDown(0);
    }
    return top;
  }
  peek() { return this._data[0]; }
  _bubbleUp(i) {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this._cmp(this._data[i], this._data[p]) < 0) {
        [this._data[i], this._data[p]] = [this._data[p], this._data[i]];
        i = p;
      } else break;
    }
  }
  _sinkDown(i) {
    const n = this._data.length;
    while (true) {
      let min = i;
      const l = 2 * i + 1, r = 2 * i + 2;
      if (l < n && this._cmp(this._data[l], this._data[min]) < 0) min = l;
      if (r < n && this._cmp(this._data[r], this._data[min]) < 0) min = r;
      if (min !== i) {
        [this._data[i], this._data[min]] = [this._data[min], this._data[i]];
        i = min;
      } else break;
    }
  }
}

// ══════════════════════════════════════════════════════════════
// SHARED CONSTANTS
// ══════════════════════════════════════════════════════════════

// All 26 neighbor directions with precomputed move costs
const DIRS_26 = [];
for (let dz = -1; dz <= 1; dz++)
  for (let dy = -1; dy <= 1; dy++)
    for (let dx = -1; dx <= 1; dx++) {
      if (dx === 0 && dy === 0 && dz === 0) continue;
      const cost = Math.sqrt(dx * dx + dy * dy + dz * dz);
      DIRS_26.push({ dx, dy, dz, cost });
    }

// 6 orthogonal directions only
const DIRS_6 = [
  { dx: 1, dy: 0, dz: 0, cost: 1 }, { dx: -1, dy: 0, dz: 0, cost: 1 },
  { dx: 0, dy: 1, dz: 0, cost: 1 }, { dx: 0, dy: -1, dz: 0, cost: 1 },
  { dx: 0, dy: 0, dz: 1, cost: 1 }, { dx: 0, dy: 0, dz: -1, cost: 1 },
];

function heuristic(ax, ay, az, bx, by, bz) {
  return Math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2);
}

// Reconstruct path from parent map
function reconstructPath(parentMap, startKey, goalKey, keyToCoord) {
  const path = [];
  let cur = goalKey;
  let iterations = 0;
  while (cur !== null && cur !== undefined && iterations++ < 100000) {
    path.push(keyToCoord(cur));
    if (cur === startKey) break;
    cur = parentMap[cur];
  }
  return path.reverse();
}
