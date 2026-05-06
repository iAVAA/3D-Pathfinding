// ══════════════════════════════════════════════════════════════
// UTILS — Shared utilities for 3D Pathfinding
// Contains MinHeap priority queue, 26-directional move table, and heuristics.
// ══════════════════════════════════════════════════════════════

class MinHeap {
	constructor(cmp = (a, b) => a.f - b.f) {
		this._d = [];
		this._c = cmp;
	}

	get size() { return this._d.length; }
	isEmpty() { return !this._d.length; }
	peek() { return this._d[0]; }

	push(item) {
		this._d.push(item);
		this._up(this._d.length - 1);
	}

	pop() {
		const top = this._d[0];
		const last = this._d.pop();
		if (this._d.length) { this._d[0] = last; this._dn(0); }
		return top;
	}

	_up(i) {
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (this._c(this._d[i], this._d[p]) < 0) {
				[this._d[i], this._d[p]] = [this._d[p], this._d[i]];
				i = p;
			} else break;
		}
	}

	_dn(i) {
		const n = this._d.length;
		while (true) {
			let m = i, l = 2 * i + 1, r = 2 * i + 2;
			if (l < n && this._c(this._d[l], this._d[m]) < 0) m = l;
			if (r < n && this._c(this._d[r], this._d[m]) < 0) m = r;
			if (m !== i) { [this._d[i], this._d[m]] = [this._d[m], this._d[i]]; i = m; }
			else break;
		}
	}
}

// 26-directional movement in 3D grid
const DIRS_26 = [];
for (let dz = -1; dz <= 1; dz++) {
	for (let dy = -1; dy <= 1; dy++) {
		for (let dx = -1; dx <= 1; dx++) {
			if (!dx && !dy && !dz) continue;
			DIRS_26.push({ dx, dy, dz, cost: Math.sqrt(dx * dx + dy * dy + dz * dz) });
		}
	}
}

// Alias to ensure compatibility with code that uses DIRS26
const DIRS26 = DIRS_26;

// Euclidean distance heuristics
function H(ax, ay, az, bx, by, bz) {
	return Math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2);
}

function heuristic(ax, ay, az, bx, by, bz) {
	return H(ax, ay, az, bx, by, bz);
}
