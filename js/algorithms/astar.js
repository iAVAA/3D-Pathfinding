// ══════════════════════════════════════════════════════════════
// A* PATHFINDING — Classic A* with 26-directional movement
// Uses the Octree for collision detection and spatial queries
// ══════════════════════════════════════════════════════════════

function astarFindPath(start, goal, octree) {
	const w = octree.w;
	const IDX = (x, y, z) => x * w * w + y * w + z;
	const sx = Math.round(start.x), sy = Math.round(start.y), sz = Math.round(start.z);
	const gx = Math.round(goal.x), gy = Math.round(goal.y), gz = Math.round(goal.z);

	if (octree.isObstacle(sx, sy, sz) || octree.isObstacle(gx, gy, gz)) return null;

	const total = w * w * w;
	const gScore = new Float32Array(total).fill(Infinity);
	const fScore = new Float32Array(total).fill(Infinity);
	const parent = new Int32Array(total).fill(-2);  // -2 = unvisited, -1 = no parent (start)
	const closed = new Uint8Array(total);

	const startIdx = IDX(sx, sy, sz);
	const goalIdx = IDX(gx, gy, gz);

	gScore[startIdx] = 0;
	fScore[startIdx] = heuristic(sx, sy, sz, gx, gy, gz);
	parent[startIdx] = -1;

	const open = new MinHeap((a, b) => a.f - b.f);
	open.push({ idx: startIdx, f: fScore[startIdx] });

	while (!open.isEmpty()) {
		const { idx: cur } = open.pop();
		if (closed[cur]) continue;
		closed[cur] = 1;

		if (cur === goalIdx) {
			const path = [];
			let c = cur;
			while (c !== -1) {
				path.push({
					x: Math.floor(c / (w * w)),
					y: Math.floor((c % (w * w)) / w),
					z: c % w
				});
				c = parent[c];
			}
			return path.reverse();
		}

		const cx = Math.floor(cur / (w * w));
		const cy = Math.floor((cur % (w * w)) / w);
		const cz = cur % w;

		for (const { dx, dy, dz, cost } of DIRS_26) {
			const nx = cx + dx, ny = cy + dy, nz = cz + dz;
			if (!octree.inBounds(nx, ny, nz) || octree.isObstacle(nx, ny, nz)) continue;
			const ni = IDX(nx, ny, nz);
			if (closed[ni]) continue;

			const ng = gScore[cur] + cost;
			if (ng < gScore[ni]) {
				parent[ni] = cur;
				gScore[ni] = ng;
				fScore[ni] = ng + heuristic(nx, ny, nz, gx, gy, gz);
				open.push({ idx: ni, f: fScore[ni] });
			}
		}
	}

	return null; // No path found
}
