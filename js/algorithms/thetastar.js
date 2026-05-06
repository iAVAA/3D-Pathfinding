// ══════════════════════════════════════════════════════════════
// THETA* PATHFINDING — Any-Angle Path Planning
// Extends A* with line-of-sight checks to produce smoother,
// shorter paths by bypassing intermediate grid nodes.
//
// When a neighbor n is discovered via node s, Theta* checks:
//   - If parent(s) has line-of-sight to n → use parent(s) as n's parent
//   - Otherwise → use s as n's parent (standard A*)
// This allows paths that are not constrained to grid edges.
// ══════════════════════════════════════════════════════════════

function thetastarFindPath(start, goal, octree) {
	const w = octree.w;
	const IDX = (x, y, z) => x * w * w + y * w + z;
	const sx = Math.round(start.x), sy = Math.round(start.y), sz = Math.round(start.z);
	const gx = Math.round(goal.x), gy = Math.round(goal.y), gz = Math.round(goal.z);

	if (octree.isObstacle(sx, sy, sz) || octree.isObstacle(gx, gy, gz)) return null;

	const total = w * w * w;
	const gScore = new Float32Array(total).fill(Infinity);
	const fScore = new Float32Array(total).fill(Infinity);
	// parentIdx[i] = index of parent node, or -1 if start
	const parentIdx = new Int32Array(total).fill(-2);
	const closed = new Uint8Array(total);

	const startIdx = IDX(sx, sy, sz);
	const goalIdx = IDX(gx, gy, gz);

	gScore[startIdx] = 0;
	fScore[startIdx] = heuristic(sx, sy, sz, gx, gy, gz);
	parentIdx[startIdx] = -1;

	const open = new MinHeap((a, b) => a.f - b.f);
	open.push({ idx: startIdx, f: fScore[startIdx] });

	// Helper: decode index back to (x,y,z)
	const decode = idx => ({
		x: Math.floor(idx / (w * w)),
		y: Math.floor((idx % (w * w)) / w),
		z: idx % w
	});

	while (!open.isEmpty()) {
		const { idx: cur } = open.pop();
		if (closed[cur]) continue;
		closed[cur] = 1;

		if (cur === goalIdx) {
			// Reconstruct path
			const path = [];
			let c = cur;
			while (c !== -1) {
				path.push(decode(c));
				c = parentIdx[c];
			}
			return path.reverse();
		}

		const { x: cx, y: cy, z: cz } = decode(cur);
		const par = parentIdx[cur]; // parent of cur (-1 if start)

		for (const { dx, dy, dz } of DIRS_26) {
			const nx = cx + dx, ny = cy + dy, nz = cz + dz;
			if (!octree.inBounds(nx, ny, nz) || octree.isObstacle(nx, ny, nz)) continue;
			const ni = IDX(nx, ny, nz);
			if (closed[ni]) continue;

			// ── Path 2 (Theta* key step): try grandparent → neighbor ──
			let bestG = Infinity;
			let bestPar = cur;

			if (par !== -1) {
				const { x: px, y: py, z: pz } = decode(par);
				// Check line-of-sight from grandparent to neighbor
				if (octree.lineOfSight(px, py, pz, nx, ny, nz)) {
					const dist = heuristic(px, py, pz, nx, ny, nz);
					const candidateG = gScore[par] + dist;
					if (candidateG < bestG) {
						bestG = candidateG;
						bestPar = par;
					}
				}
			}

			// ── Path 1 (standard A*): cur → neighbor ──
			{
				const moveCost = heuristic(cx, cy, cz, nx, ny, nz);
				const candidateG = gScore[cur] + moveCost;
				if (candidateG < bestG) {
					bestG = candidateG;
					bestPar = cur;
				}
			}

			if (bestG < gScore[ni]) {
				parentIdx[ni] = bestPar;
				gScore[ni] = bestG;
				fScore[ni] = bestG + heuristic(nx, ny, nz, gx, gy, gz);
				open.push({ idx: ni, f: fScore[ni] });
			}
		}
	}

	return null;
}
