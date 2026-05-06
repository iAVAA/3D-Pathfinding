// ══════════════════════════════════════════════════════════════
// THREE.JS SETUP
// ══════════════════════════════════════════════════════════════

const wrapper = document.getElementById('canvas-wrapper');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
wrapper.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x020409, 0.018);
const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 1000);

const ambient = new THREE.AmbientLight(0x0a1628, 1.5);
const dirLight = new THREE.DirectionalLight(0x00f5d4, 0.6);
const pointLight = new THREE.PointLight(0xf72585, 0.8, 80);
dirLight.position.set(30, 40, 20);
dirLight.castShadow = true;
pointLight.position.set(-10, 15, -10);
scene.add(ambient, dirLight, pointLight);

// Orbit controls (manual)
let isDragging = false, prevMouse = { x: 0, y: 0 };
let spherical = { theta: 0.6, phi: 1.0, r: 35 };

function updateCamera() {
	const c = worldSize / 2;
	camera.position.set(
		c + spherical.r * Math.sin(spherical.phi) * Math.sin(spherical.theta),
		c + spherical.r * Math.cos(spherical.phi),
		c + spherical.r * Math.sin(spherical.phi) * Math.cos(spherical.theta)
	);
	camera.lookAt(c, c / 2, c);
}

renderer.domElement.addEventListener('mousedown', e => {
	isDragging = true;
	prevMouse = { x: e.clientX, y: e.clientY };
});
window.addEventListener('mouseup', () => isDragging = false);
window.addEventListener('mousemove', e => {
	if (!isDragging) return;
	spherical.theta -= (e.clientX - prevMouse.x) * 0.008;
	spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi + (e.clientY - prevMouse.y) * 0.008));
	prevMouse = { x: e.clientX, y: e.clientY };
	updateCamera();
});
renderer.domElement.addEventListener('wheel', e => {
	spherical.r = Math.max(5, Math.min(120, spherical.r + e.deltaY * 0.05));
	updateCamera();
});

function resize() {
	const w = wrapper.clientWidth, h = wrapper.clientHeight;
	renderer.setSize(w, h);
	camera.aspect = w / h;
	camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();


// ══════════════════════════════════════════════════════════════
// STATE
// ══════════════════════════════════════════════════════════════

let worldSize = 10;
let obstacles = [];
let startPt = { x: 0, y: 0, z: 0 };
let goalPt = { x: 9, y: 9, z: 9 };
let currentAlgo = 'astar';
let allResults = {};

const ALGO_CFG = {
	astar: { name: 'A*', color: 0xffe74c, emissive: 0xaa7700 },
	thetastar: { name: 'Theta*', color: 0x00f5d4, emissive: 0x007760 },
	dstar: { name: 'D* Lite', color: 0xf72585, emissive: 0x990050 },
	jps: { name: 'JPS', color: 0xff9f1c, emissive: 0xaa5500 },
};

const ALGO_FNS = {
	astar: astarFindPath,
	thetastar: thetastarFindPath,
	dstar: dstarFindPath,
	jps: jpsFindPath,
};


// ══════════════════════════════════════════════════════════════
// MATERIALS + GEOMETRY
// ══════════════════════════════════════════════════════════════

const matObs = new THREE.MeshPhongMaterial({ color: 0x3a0ca3, emissive: 0x1a0460, transparent: true, opacity: 0.85, shininess: 80 });
const matStart = new THREE.MeshPhongMaterial({ color: 0x39ff14, emissive: 0x1aaa00, shininess: 100 });
const matGoal = new THREE.MeshPhongMaterial({ color: 0xff3864, emissive: 0xaa0022, shininess: 100 });
const cubeGeo = new THREE.BoxGeometry(0.85, 0.85, 0.85);
const edgeGeo = new THREE.EdgesGeometry(cubeGeo);
const markerGeo = new THREE.SphereGeometry(0.5, 16, 16);

// Scene object groups
const grp = {
	obstacles: new THREE.Group(),
	path: new THREE.Group(),
	pathBalls: new THREE.Group(),
	grid: null, floor: null, start: null, goal: null,
};
scene.add(grp.obstacles, grp.path, grp.pathBalls);


// ══════════════════════════════════════════════════════════════
// SCENE BUILDERS
// ══════════════════════════════════════════════════════════════

function buildGrid() {
	if (grp.grid) scene.remove(grp.grid);
	if (grp.floor) scene.remove(grp.floor);

	const n = worldSize, pts = [];
	for (let i = 0; i <= n; i++) {
		pts.push(0, -0.01, i, n, -0.01, i);
		pts.push(i, -0.01, 0, i, -0.01, n);
	}
	[[0, 0], [n, 0], [0, n], [n, n]].forEach(([ex, ez]) => pts.push(ex, -0.01, ez, ex, n, ez));
	for (let i = 0; i <= n; i++) {
		pts.push(0, n, i, n, n, i);
		pts.push(i, n, 0, i, n, n);
	}

	const geo = new THREE.BufferGeometry();
	geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
	grp.grid = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x0f3460, transparent: true, opacity: 0.5 }));
	scene.add(grp.grid);

	grp.floor = new THREE.Mesh(
		new THREE.PlaneGeometry(n, n),
		new THREE.MeshPhongMaterial({ color: 0x040c20, transparent: true, opacity: 0.6, side: THREE.DoubleSide })
	);
	grp.floor.rotation.x = -Math.PI / 2;
	grp.floor.position.set(n / 2, -0.015, n / 2);
	scene.add(grp.floor);
}

function rebuildObsMeshes() {
	while (grp.obstacles.children.length) grp.obstacles.remove(grp.obstacles.children[0]);
	for (const o of obstacles) {
		const m = new THREE.Mesh(cubeGeo, matObs);
		m.position.set(o.x + 0.5, o.y + 0.5, o.z + 0.5);
		m.castShadow = true;
		m.add(new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({ color: 0x7209b7 })));
		grp.obstacles.add(m);
	}
	document.getElementById('hudObs').textContent = obstacles.length;
}

function rebuildMarker(type) {
	const pt = type === 'start' ? startPt : goalPt;
	const mat = type === 'start' ? matStart : matGoal;
	if (grp[type]) scene.remove(grp[type]);

	const g = new THREE.Group();
	g.add(new THREE.Mesh(markerGeo, mat));

	const ring = new THREE.Mesh(
		new THREE.RingGeometry(0.6, 0.7, 32),
		new THREE.MeshBasicMaterial({ color: mat.color, side: THREE.DoubleSide, transparent: true, opacity: 0.5 })
	);
	ring.rotation.x = -Math.PI / 2;
	g.add(ring);

	g.position.set(pt.x + 0.5, pt.y + 0.5, pt.z + 0.5);
	g.userData = { type, baseY: pt.y + 0.5 };
	scene.add(g);
	grp[type] = g;
}

function buildPathMesh(path, algoKey) {
	while (grp.path.children.length) grp.path.remove(grp.path.children[0]);
	while (grp.pathBalls.children.length) grp.pathBalls.remove(grp.pathBalls.children[0]);
	if (!path || path.length < 2) return;

	const cfg = ALGO_CFG[algoKey] || ALGO_CFG.astar;
	const pts = path.map(p => new THREE.Vector3(p.x + 0.5, p.y + 0.5, p.z + 0.5));
	const tube = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), path.length * 3, 0.08, 8, false);

	grp.path.add(new THREE.Mesh(tube, new THREE.MeshPhongMaterial({ color: cfg.color, emissive: cfg.emissive, transparent: true, opacity: 0.85 })));

	const ballGeo = new THREE.SphereGeometry(0.14, 8, 8);
	for (let i = 1; i < path.length - 1; i++) {
		const ball = new THREE.Mesh(ballGeo, new THREE.MeshPhongMaterial({ color: cfg.color, emissive: cfg.emissive, shininess: 120 }));
		ball.position.copy(pts[i]);
		ball.userData.delay = i * 0.06;
		grp.pathBalls.add(ball);
	}
}


// ══════════════════════════════════════════════════════════════
// LOG
// ══════════════════════════════════════════════════════════════

function log(msg, cls = 'log-info') {
	const el = document.getElementById('logContent');
	const line = document.createElement('div');
	line.className = cls;
	const now = new Date();
	const ts = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
	line.textContent = `[${ts}] ${msg}`;
	el.appendChild(line);
	el.scrollTop = el.scrollHeight;
}


// ══════════════════════════════════════════════════════════════
// ALGORITHM SELECTOR
// ══════════════════════════════════════════════════════════════

function selectAlgo(key) {
	if (!ALGO_CFG[key]) return;
	currentAlgo = key;

	// Update tab highlight
	document.querySelectorAll('.algo-tab').forEach(t => t.classList.remove('active'));
	const tab = document.getElementById('tab-' + key);
	if (tab) tab.classList.add('active');

	// Update description text
	document.querySelectorAll('.ad').forEach(el => el.classList.remove('active'));
	const desc = document.getElementById('desc-' + key);
	if (desc) desc.classList.add('active');

	// Switch 3D path visualization
	const res = allResults[key];
	if (res && res.path && res.path.length > 1) {
		buildPathMesh(res.path, key);
		const len = res.path.length - 1;
		document.getElementById('hudPath').textContent = len + ' steps';
		document.getElementById('pathStats').style.display = 'block';
		document.getElementById('statsText').textContent = `[${ALGO_CFG[key].name}]  ${len} STEPS  ·  ${res.time}ms`;
	} else if (Object.keys(allResults).length > 0) {
		while (grp.path.children.length) grp.path.remove(grp.path.children[0]);
		while (grp.pathBalls.children.length) grp.pathBalls.remove(grp.pathBalls.children[0]);
		document.getElementById('hudPath').textContent = 'N/A';
		document.getElementById('pathStats').style.display = 'block';
		document.getElementById('statsText').textContent = `[${ALGO_CFG[key].name}]  NO PATH FOUND  ·  ${res ? res.time : '?'}ms`;
	}

	renderComparison();
}


// ══════════════════════════════════════════════════════════════
// COMPARISON TABLE
// ══════════════════════════════════════════════════════════════

function renderComparison() {
	const panel = document.getElementById('comparisonPanel');
	if (!panel) return;

	const keys = Object.keys(allResults);
	if (!keys.length) { panel.style.display = 'none'; return; }
	panel.style.display = 'block';

	let bestT = Infinity, minP = Infinity;
	for (const k of keys) {
		const r = allResults[k];
		if (r.time < bestT) bestT = r.time;
		if (r.path && r.path.length - 1 < minP) minP = r.path.length - 1;
	}

	let html = `
    <div class="cmp-title">// Algorithm Comparison</div>
    <table class="cmp-table">
      <thead><tr><th>Algorithm</th><th>Time</th><th>Steps</th><th>Ok</th></tr></thead>
      <tbody>
  `;

	for (const k of Object.keys(ALGO_CFG)) {
		const r = allResults[k];
		if (!r) continue;
		const cfg = ALGO_CFG[k];
		const has = r.path && r.path.length > 0;
		const steps = has ? r.path.length - 1 : '—';
		const col = '#' + cfg.color.toString(16).padStart(6, '0');
		html += `
      <tr class="cmp-row ${k === currentAlgo ? 'cmp-active' : ''}" onclick="selectAlgo('${k}')">
        <td><span class="cmp-dot" style="background:${col}"></span>${cfg.name}</td>
        <td class="${r.time === bestT ? 'cmp-best' : ''}">${r.time}ms</td>
        <td class="${has && steps === minP ? 'cmp-best' : ''}">${steps}</td>
        <td class="${has ? 'cmp-found' : 'cmp-nope'}">${has ? '✓' : '✗'}</td>
      </tr>`;
	}

	html += '</tbody></table>';
	panel.innerHTML = html;
}


// ══════════════════════════════════════════════════════════════
// RUNNERS
// ══════════════════════════════════════════════════════════════

function runSelected() { runAlgos([currentAlgo]); }
function runAll() { runAlgos(Object.keys(ALGO_FNS)); }

function runAlgos(keys) {
	if (H(startPt.x, startPt.y, startPt.z, goalPt.x, goalPt.y, goalPt.z) < 0.5) {
		log('Start and Goal overlap!', 'log-err');
		return;
	}

	document.getElementById('loadingOverlay').style.display = 'flex';
	document.getElementById('runBtn').disabled = true;
	document.getElementById('runAllBtn').disabled = true;
	log('Computing: ' + keys.map(k => ALGO_CFG[k].name).join(', ') + '...', 'log-info');

	setTimeout(() => {
		const oct = new Octree(worldSize);
		oct.build(obstacles);

		for (const key of keys) {
			const t0 = performance.now();
			let path = null;
			try { path = ALGO_FNS[key](startPt, goalPt, oct); } catch (e) { console.error(key, e); }
			const ms = parseFloat((performance.now() - t0).toFixed(2));
			allResults[key] = { path, time: ms };
			log(`${ALGO_CFG[key].name}: ${path ? path.length - 1 + ' steps' : 'no path found'} in ${ms}ms`, path ? 'log-ok' : 'log-err');
		}

		document.getElementById('loadingOverlay').style.display = 'none';
		document.getElementById('runBtn').disabled = false;
		document.getElementById('runAllBtn').disabled = false;

		const res = allResults[currentAlgo];
		if (res && res.path && res.path.length > 1) {
			buildPathMesh(res.path, currentAlgo);
			document.getElementById('hudPath').textContent = res.path.length - 1 + ' steps';
			document.getElementById('pathStats').style.display = 'block';
			document.getElementById('statsText').textContent = `[${ALGO_CFG[currentAlgo].name}]  ${res.path.length - 1} STEPS  -  ${res.time}ms`;
		} else {
			while (grp.path.children.length) grp.path.remove(grp.path.children[0]);
			while (grp.pathBalls.children.length) grp.pathBalls.remove(grp.pathBalls.children[0]);
			document.getElementById('hudPath').textContent = 'N/A';
		}

		renderComparison();
	}, 30);
}


// ══════════════════════════════════════════════════════════════
// UI ACTIONS
// ══════════════════════════════════════════════════════════════

function applyWorld() {
	const n = parseInt(document.getElementById('worldSize').value);
	if (isNaN(n) || n < 3 || n > 30) { log('Invalid size (3-30)', 'log-err'); return; }

	worldSize = n;
	obstacles = [];
	allResults = {};
	rebuildObsMeshes();
	startPt = { x: 0, y: 0, z: 0 };
	goalPt = { x: n - 1, y: n - 1, z: n - 1 };
	document.getElementById('gx').value = n - 1;
	document.getElementById('gy').value = n - 1;
	document.getElementById('gz').value = n - 1;

	buildGrid();
	rebuildMarker('start');
	rebuildMarker('goal');
	clearPath();
	updateObsList();
	spherical.r = n * 2.8;
	updateCamera();
	document.getElementById('hudSize').textContent = `${n} * ${n} * ${n}`;
	document.getElementById('hudPath').textContent = '—';
	log(`World ${n} * ${n} * ${n} initialized`, 'log-ok');
}

function addObstacle() {
	const x = parseInt(document.getElementById('ox').value);
	const y = parseInt(document.getElementById('oy').value);
	const z = parseInt(document.getElementById('oz').value);

	if ([x, y, z].some(v => isNaN(v) || v < 0 || v >= worldSize)) {
		log(`Coordinates out of bounds [0, ${worldSize - 1}]`, 'log-err'); return;
	}
	if (obstacles.some(o => o.x === x && o.y === y && o.z === z)) {
		log('Obstacle already exists at this position', 'log-err'); return;
	}
	if ((x === startPt.x && y === startPt.y && z === startPt.z) ||
		(x === goalPt.x && y === goalPt.y && z === goalPt.z)) {
		log('Cannot place an obstacle on start/goal', 'log-err'); return;
	}

	obstacles.push({ x, y, z });
	rebuildObsMeshes();
	updateObsList();
	log(`Obstacle added (${x}, ${y}, ${z})`, 'log-ok');
	clearPath();
}

function removeObstacle(i) {
	const o = obstacles[i];
	obstacles.splice(i, 1);
	rebuildObsMeshes();
	updateObsList();
	log(`Obstacle removed (${o.x}, ${o.y}, ${o.z})`, 'log-info');
	clearPath();
}

function clearObstacles() {
	obstacles = [];
	rebuildObsMeshes();
	updateObsList();
	log('All obstacles cleared', 'log-info');
	clearPath();
}

function updateObsList() {
	const list = document.getElementById('obstacleList');
	list.innerHTML = '';
	obstacles.forEach((o, i) => {
		const div = document.createElement('div');
		div.className = 'obs-item';
		div.innerHTML = `<span>(${o.x}, ${o.y}, ${o.z})</span><button class="obs-rm" onclick="removeObstacle(${i})">✕</button>`;
		list.appendChild(div);
	});
}

function setPoint(type) {
	const px = type === 'start' ? 'sx' : 'gx';
	const py = type === 'start' ? 'sy' : 'gy';
	const pz = type === 'start' ? 'sz' : 'gz';
	const x = parseInt(document.getElementById(px).value);
	const y = parseInt(document.getElementById(py).value);
	const z = parseInt(document.getElementById(pz).value);

	if ([x, y, z].some(v => isNaN(v) || v < 0 || v >= worldSize)) {
		log('Coordinates out of bounds', 'log-err'); return;
	}
	if (obstacles.some(o => o.x === x && o.y === y && o.z === z)) {
		log(`${type === 'start' ? 'Start' : 'Goal'}: overlaps with an obstacle!`, 'log-err'); return;
	}

	if (type === 'start') startPt = { x, y, z };
	else goalPt = { x, y, z };

	rebuildMarker(type);
	log(`${type === 'start' ? 'Start' : 'Goal'} → (${x}, ${y}, ${z})`, 'log-ok');
	clearPath();
}

function clearPath() {
	while (grp.path.children.length) grp.path.remove(grp.path.children[0]);
	while (grp.pathBalls.children.length) grp.pathBalls.remove(grp.pathBalls.children[0]);
	document.getElementById('pathStats').style.display = 'none';
	document.getElementById('hudPath').textContent = '—';
	allResults = {};
	renderComparison();
}


// ══════════════════════════════════════════════════════════════
// ANIMATION LOOP
// ══════════════════════════════════════════════════════════════

const clock = new THREE.Clock();

function animate() {
	requestAnimationFrame(animate);
	const t = clock.getElapsedTime();

	if (grp.start) {
		grp.start.position.y = grp.start.userData.baseY + Math.sin(t * 1.8) * 0.12;
		grp.start.children[1].rotation.y = t * 0.8;
	}
	if (grp.goal) {
		grp.goal.position.y = grp.goal.userData.baseY + Math.sin(t * 2.1 + 1) * 0.12;
		grp.goal.children[1].rotation.y = -t;
	}

	grp.pathBalls.children.forEach(ball => {
		const s = 0.7 + 0.5 * Math.abs(Math.sin(t * 3 - ball.userData.delay * 8));
		ball.scale.setScalar(s);
		ball.material.emissiveIntensity = s;
	});

	grp.obstacles.children.forEach((m, i) => {
		m.material.emissiveIntensity = 0.3 + 0.15 * Math.sin(t * 1.2 + i * 0.7);
	});

	pointLight.position.x = Math.sin(t * 0.3) * (worldSize * 0.8);
	pointLight.position.z = Math.cos(t * 0.3) * (worldSize * 0.8);

	renderer.render(scene, camera);
}


// ══════════════════════════════════════════════════════════════
// INIT
// ══════════════════════════════════════════════════════════════

applyWorld();
selectAlgo('astar');
log('System initialized', 'log-ok');
log('Select an algorithm and press ▶', 'log-info');
animate();
