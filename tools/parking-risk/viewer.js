const COLOR_HEX = {
  BLUE: '#3050ff', RED: '#ff0000', CYAN: '#00ffff',
  PURPLE: '#6000b0', GREEN: '#00ff00', ORANGE: '#ff8000',
  PINK: '#f060c0', GREY: '#707070', LIGHTBLUE: '#70c0ff',
  BROWN: '#603000', DEFAULT: '#c0c0c0',
};

const els = {
  app: document.querySelector('.app'),
  meta: document.getElementById('meta'),
  fileInput: document.getElementById('fileInput'),
  groupTabs: document.getElementById('groupTabs'),
  reportList: document.getElementById('reportList'),
  boards: document.getElementById('boards'),
  panViewport: document.getElementById('panViewport'),
  panSurface: document.getElementById('panSurface'),
  axisTopInner: document.getElementById('axisTopInner'),
  axisLeftInner: document.getElementById('axisLeftInner'),
  hoverInfo: document.getElementById('hoverInfo'),
  hoverTip: document.getElementById('hoverTip'),
  togglePanel: document.getElementById('togglePanel'),
  resetView: document.getElementById('resetView'),
  zoomInfo: document.getElementById('zoomInfo'),
  showSafe: document.getElementById('showSafe'),
  showCaution: document.getElementById('showCaution'),
  showDanger: document.getElementById('showDanger'),
  showGoals: document.getElementById('showGoals'),
  showObjects: document.getElementById('showObjects'),
};

const BASE_CELL = 24;
const MIN_ZOOM = 0.3;
const MAX_ZOOM = 4.0;
const STORAGE_KEY = 'parkingRiskLastReport';
const DIR_HANDLE_KEY = 'parkingRiskReportsDir';

const state = {
  reports: window.PARKING_RISK_REPORTS || [],
  activeIndex: 0,
  group: 'all',
  zoom: 1,
  pan: { x: 0, y: 0 },
  dirHandle: null,
};

init();

function init() {
  scanReportsDir().then(function() {
    renderReportList();
    var lastName = localStorage.getItem(STORAGE_KEY);
    var idx = 0;
    if (lastName && state.reports.length > 0) {
      var found = state.reports.findIndex(function(r) { return r.name === lastName; });
      if (found >= 0) idx = found;
    }
    if (state.reports.length > 0) loadReport(idx);
    else showEmpty();
  });

  els.fileInput.addEventListener('change', function(e) {
    var file = e.target.files && e.target.files[0];
    if (file) loadLvlFile(file);
  });

  els.groupTabs.addEventListener('click', function(e) {
    var btn = e.target.closest('.groupBtn');
    if (!btn) return;
    state.group = btn.dataset.group;
    var all = els.groupTabs.querySelectorAll('.groupBtn');
    for (var i = 0; i < all.length; i++) all[i].classList.remove('active');
    btn.classList.add('active');
    renderReportList();
  });

  for (var k = 0; k < [els.showSafe, els.showCaution, els.showDanger, els.showGoals, els.showObjects].length; k++) {
    [els.showSafe, els.showCaution, els.showDanger, els.showGoals, els.showObjects][k].addEventListener('change', renderBoard);
  }
  els.togglePanel.addEventListener('click', function() {
    els.app.classList.toggle('collapsed');
    requestAnimationFrame(resetView);
  });
  els.resetView.addEventListener('click', resetView);
  els.boards.addEventListener('dblclick', resetView);

  els.boards.addEventListener('mousemove', function(e) {
    var cell = e.target.closest('.cell');
    if (!cell || !cell.dataset.r) { els.hoverTip.classList.remove('show'); return; }
    var report = currentReport();
    var r = Number(cell.dataset.r), c = Number(cell.dataset.c);
    var info = cellInfo(report, r, c);
    var rect = els.boards.getBoundingClientRect();
    els.hoverTip.textContent = info.tooltip;
    els.hoverTip.style.left = (e.clientX - rect.left + 14) + 'px';
    els.hoverTip.style.top = (e.clientY - rect.top + 14) + 'px';
    els.hoverTip.classList.add('show');
    els.hoverInfo.textContent = 'Java coordinates: (' + r + ', ' + c + ') - ' + info.summary;
  });
  els.boards.addEventListener('mouseleave', function() { els.hoverTip.classList.remove('show'); });

  els.boards.addEventListener('wheel', function(e) {
    if (!currentReport()) return;
    e.preventDefault();
    zoomAt(e.clientX, e.clientY, e.deltaY < 0 ? 1.1 : 1 / 1.1);
  }, { passive: false });

  var dragging = false, last = { x: 0, y: 0 };
  els.boards.addEventListener('mousedown', function(e) {
    if (e.button !== 0 || !currentReport()) return;
    dragging = true; last.x = e.clientX; last.y = e.clientY;
    els.boards.classList.add('grabbing'); e.preventDefault();
  });
  window.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    state.pan.x += e.clientX - last.x; state.pan.y += e.clientY - last.y;
    last.x = e.clientX; last.y = e.clientY;
    applyPan();
  });
  window.addEventListener('mouseup', function() { dragging = false; els.boards.classList.remove('grabbing'); });
}

function currentReport() { return state.reports[state.activeIndex] || null; }

function renderReportList() {
  var rows = state.reports.map(function(r, i) { return { report: r, index: i }; });
  if (state.group !== 'all') {
    rows = rows.filter(function(x) {
      var p = (x.report.path || '').replace(/\\/g, '/');
      // Show if path matches the group, OR if it's an ad-hoc report (no known group prefix)
      if (p.indexOf(state.group + '/') === 0 || p.indexOf(state.group + '\\') === 0) return true;
      if (p.indexOf('levels/') !== 0 && p.indexOf('levels\\') !== 0 &&
          p.indexOf('complevels/') !== 0 && p.indexOf('complevels\\') !== 0) return true;
      return false;
    });
  }
  els.reportList.innerHTML = '';
  if (rows.length === 0) return;
  for (var i = 0; i < rows.length; i++) {
    var row = document.createElement('div');
    row.className = 'reportRow' + (rows[i].index === state.activeIndex ? ' active' : '');
    row.innerHTML = '<span>' + escapeHtml(rows[i].report.name) + '</span><span class="path">' + escapeHtml(rows[i].report.path) + '</span>';
    row.addEventListener('click', (function(idx) { return function() { loadReport(idx); }; })(rows[i].index));
    els.reportList.appendChild(row);
  }
}

function loadReport(index) {
  state.activeIndex = index;
  var r = currentReport();
  if (r) localStorage.setItem(STORAGE_KEY, r.name);
  renderReportList(); renderMeta(); renderBoard(); resetView();
}

function renderMeta() {
  var r = currentReport();
  if (!r) return;
  var s = r.analysis.summary;
  var h = '<b>' + escapeHtml(r.name) + '</b><br>' +
    escapeHtml(r.path) + '<br>' + r.rows + ' x ' + r.cols + '<br>' +
    'safe ' + s.safe + ' / caution ' + s.caution + ' / danger ' + s.danger + '<br>' +
    'static doors ' + s.staticArticulation + ', current doors ' + s.dynamicArticulation;
  if (s.occupiedGateClusters) h += '<br>occupied gates ' + s.occupiedGateClusters;
  els.meta.innerHTML = h;
}

function showEmpty() { els.panSurface.innerHTML = ''; }

// ---- .lvl loading + in-browser analysis ----

async function loadLvlFile(file) {
  try {
    var text = await file.text();
    var level = parseLvl(text, file.name.replace(/\.lvl$/i, ''));
    var report = analyzeLevel(level);
    state.reports.unshift(report);
    state.activeIndex = 0;
    localStorage.setItem(STORAGE_KEY, report.name);
    renderReportList(); renderMeta(); renderBoard(); resetView();
    await saveReportToDir(report);
  } catch (err) {
    els.meta.innerHTML = '<b>Error:</b> ' + escapeHtml(err.message || String(err));
    console.error(err);
  }
}

// ---- Scan reports/ directory for saved files ----

async function scanReportsDir() {
  try {
    var stored = await readDirHandleFromDB();
    if (!stored) return;
    if (await stored.requestPermission({ mode: 'readwrite' }) !== 'granted') return;
    state.dirHandle = stored;

    // Merge already-loaded names for dedup
    var seen = new Set(state.reports.map(function(r) { return r.name; }));
    var entries = [];
    var iter = stored.values();
    for await (var entry of iter) {
      if (entry.kind === 'file' && entry.name.indexOf('parking-risk-') === 0 && entry.name.indexOf('.js') > 0) {
        entries.push(entry);
      }
    }
    // Sort newest first
    entries.sort(function(a, b) { return b.name.localeCompare(a.name); });

    for (var i = 0; i < entries.length; i++) {
      try {
        var file = await entries[i].getFile();
        var text = await file.text();
        // Extract JSON from the JS assignment
        var match = text.match(/window\.__PARKING_RISK_REPORT__\s*=\s*([\s\S]*?);\s*$/);
        if (!match) continue;
        var report = JSON.parse(match[1]);
        if (report && report.name && !seen.has(report.name)) {
          seen.add(report.name);
          state.reports.push(report);
        }
      } catch (_) { /* skip unreadable files */ }
    }
  } catch (_) { /* dir not accessible */ }
}

// ---- File System Access API — save directly to reports/ ----

async function getReportsDir() {
  if (state.dirHandle) return state.dirHandle;
  try {
    var stored = await readDirHandleFromDB();
    if (stored) {
      if (await stored.requestPermission({ mode: 'readwrite' }) === 'granted') {
        state.dirHandle = stored; return stored;
      }
    }
  } catch (_) {}
  try {
    var handle = await window.showDirectoryPicker({ mode: 'readwrite' });
    state.dirHandle = handle;
    await writeDirHandleToDB(handle);
    return handle;
  } catch (_) { return null; }
}

function dirHandleDB() {
  return new Promise(function(resolve, reject) {
    var req = indexedDB.open('parking-risk', 1);
    req.onupgradeneeded = function() { req.result.createObjectStore('handles'); };
    req.onsuccess = function() { resolve(req.result); };
    req.onerror = function() { reject(req.error); };
  });
}

async function readDirHandleFromDB() {
  var db = await dirHandleDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction('handles', 'readonly');
    var req = tx.objectStore('handles').get(DIR_HANDLE_KEY);
    req.onsuccess = function() { resolve(req.result); };
    req.onerror = function() { reject(req.error); };
    tx.oncomplete = function() { db.close(); };
  });
}

async function writeDirHandleToDB(handle) {
  var db = await dirHandleDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction('handles', 'readwrite');
    tx.objectStore('handles').put(handle, DIR_HANDLE_KEY);
    tx.oncomplete = function() { db.close(); resolve(); };
    tx.onerror = function() { reject(tx.error); };
  });
}

async function saveReportToDir(report) {
  var name = report.name || 'unknown';
  var filename = 'parking-risk-' + name + '.js';
  var payload = JSON.stringify(report, null, 2);
  var content = '// parking-risk report — auto-generated\n// level: ' + name + '\nwindow.__PARKING_RISK_REPORT__ = ' + payload + ';\n';

  var dir = await getReportsDir();
  if (dir) {
    try {
      var fileHandle = await dir.getFileHandle(filename, { create: true });
      var writable = await fileHandle.createWritable();
      await writable.write(content);
      await writable.close();
      return;
    } catch (_) {}
  }
  // Fallback download
  var blob = new Blob([content], { type: 'application/javascript' });
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url; a.download = filename;
  document.body.appendChild(a); a.click();
  document.body.removeChild(a); URL.revokeObjectURL(url);
}

// ---- .lvl parser ----

function parseLvl(text, fallbackName) {
  var lines = text.split(/\r?\n/);
  var section = null, levelName = fallbackName;
  var colorLines = [], initialLines = [], goalLines = [];

  for (var i = 0; i < lines.length; i++) {
    var line = lines[i];
    if (line[0] === '#') { section = line.slice(1).trim().toLowerCase(); if (section === 'end') break; continue; }
    if (section === 'levelname' && line.trim()) levelName = line.trim();
    else if (section === 'colors') colorLines.push(line);
    else if (section === 'initial') initialLines.push(line);
    else if (section === 'goal') goalLines.push(line);
  }

  while (initialLines.length && !initialLines[initialLines.length - 1].trim()) initialLines.pop();
  while (goalLines.length && !goalLines[goalLines.length - 1].trim()) goalLines.pop();
  if (!initialLines.length) throw new Error('No #initial section');
  if (!goalLines.length) throw new Error('No #goal section');

  var rows = Math.max(initialLines.length, goalLines.length);
  var cols = Math.max(maxLen(initialLines), maxLen(goalLines));

  var agentColors = {}, boxColors = {};
  for (var ci = 0; ci < colorLines.length; ci++) {
    var raw = colorLines[ci], colonIdx = raw.indexOf(':');
    if (colonIdx < 0) continue;
    var color = raw.slice(0, colonIdx).trim().toUpperCase();
    var items = raw.slice(colonIdx + 1).split(',');
    for (var ti = 0; ti < items.length; ti++) {
      var ch = items[ti].trim()[0]; if (!ch) continue;
      if (ch >= '0' && ch <= '9') agentColors[ch] = color;
      else if (ch >= 'A' && ch <= 'Z') boxColors[ch] = color;
    }
  }

  var walls = new Set(), wallList = [], agents = [], boxes = [];
  var agentGoals = [], boxGoals = [], goalCells = new Set(), goalList = [];
  var immovable = new Set(), immovableList = [], boxSet = new Set(), agentSet = new Set();

  for (var r = 0; r < rows; r++) {
    var iline = r < initialLines.length ? initialLines[r] : '';
    var gline = r < goalLines.length ? goalLines[r] : '';
    for (var c = 0; c < cols; c++) {
      var ic = c < iline.length ? iline[c] : ' ';
      var gc = c < gline.length ? gline[c] : ' ';
      if (ic === '+' || gc === '+') { walls.add(key(r, c)); wallList.push({ r: r, c: c }); }
      if (ic >= '0' && ic <= '9') { agents.push({ r: r, c: c, id: ic, color: agentColors[ic] || 'DEFAULT' }); agentSet.add(key(r, c)); }
      else if (ic >= 'A' && ic <= 'Z') { boxes.push({ r: r, c: c, type: ic, color: boxColors[ic] || 'DEFAULT' }); boxSet.add(key(r, c)); }
      if (gc >= '0' && gc <= '9') { agentGoals.push({ r: r, c: c, id: gc }); goalCells.add(key(r, c)); goalList.push({ r: r, c: c }); }
      else if (gc >= 'A' && gc <= 'Z') { boxGoals.push({ r: r, c: c, type: gc }); goalCells.add(key(r, c)); goalList.push({ r: r, c: c }); }
    }
  }

  var matchColors = new Set(agents.map(function(a) { return a.color; }));
  for (var bi = 0; bi < boxes.length; bi++) {
    if (!matchColors.has(boxes[bi].color)) { immovable.add(key(boxes[bi].r, boxes[bi].c)); immovableList.push({ r: boxes[bi].r, c: boxes[bi].c }); }
  }

  return {
    name: levelName, path: fallbackName + '.lvl', rows: rows, cols: cols,
    walls: wallList, agents: agents, boxes: boxes, agentGoals: agentGoals, boxGoals: boxGoals,
    goalCells: goalList, immovableBoxes: immovableList, agentColors: agentColors, boxColors: boxColors,
    _sets: { walls: walls, goals: goalCells, immovable: immovable, boxes: boxSet, agents: agentSet },
  };
}

function maxLen(arr) { var m = 0; for (var i = 0; i < arr.length; i++) { if (arr[i].length > m) m = arr[i].length; } return m; }

// ---- Parking risk analysis ----

function neighbors(r, c) { return [[r - 1, c], [r + 1, c], [r, c - 1], [r, c + 1]]; }
function inBounds(r, c, rows, cols) { return r >= 0 && r < rows && c >= 0 && c < cols; }
function passable(r, c, rows, cols, walls, extraWalls) {
  var kk = key(r, c);
  return inBounds(r, c, rows, cols) && !walls.has(kk) && !extraWalls.has(kk);
}

function articulationPoints(rows, cols, walls, extraWalls) {
  var cells = [], index = new Map();
  for (var r = 0; r < rows; r++)
    for (var c = 0; c < cols; c++)
      if (passable(r, c, rows, cols, walls, extraWalls)) { index.set(key(r, c), cells.length); cells.push([r, c]); }
  var n = cells.length;
  if (n <= 2) return new Set();

  var adj = new Array(n); for (var i = 0; i < n; i++) adj[i] = [];
  for (var i = 0; i < n; i++) {
    var cr = cells[i][0], cc = cells[i][1];
    var nbrs = neighbors(cr, cc);
    for (var j = 0; j < nbrs.length; j++) { var jj = index.get(key(nbrs[j][0], nbrs[j][1])); if (jj !== undefined) adj[i].push(jj); }
  }

  var disc = new Array(n).fill(-1), low = new Array(n).fill(-1), parent = new Array(n).fill(-1);
  var aps = new Set(), timer = 0;

  function dfs(u, root) {
    disc[u] = low[u] = timer++;
    var children = 0;
    for (var vi = 0; vi < adj[u].length; vi++) {
      var v = adj[u][vi];
      if (disc[v] === -1) {
        parent[v] = u; children++;
        dfs(v, root);
        low[u] = Math.min(low[u], low[v]);
        if (u !== root && low[v] >= disc[u]) aps.add(key(cells[u][0], cells[u][1]));
      } else if (v !== parent[u]) low[u] = Math.min(low[u], disc[v]);
    }
    if (u === root && children >= 2) aps.add(key(cells[u][0], cells[u][1]));
  }

  for (var i = 0; i < n; i++) if (disc[i] === -1) dfs(i, i);
  return aps;
}

function componentSizesAfterRemoval(r, c, rows, cols, walls, extraWalls) {
  var blocked = new Set(extraWalls); blocked.add(key(r, c));
  var starts = [];
  var nbrs = neighbors(r, c);
  for (var i = 0; i < nbrs.length; i++) if (passable(nbrs[i][0], nbrs[i][1], rows, cols, walls, blocked)) starts.push(nbrs[i]);
  var seen = new Set(), sizes = [];
  for (var si = 0; si < starts.length; si++) {
    var sk = key(starts[si][0], starts[si][1]); if (seen.has(sk)) continue;
    var q = [starts[si]]; seen.add(sk);
    var size = 0;
    for (var head = 0; head < q.length; head++) {
      var pr = q[head][0], pc = q[head][1]; size++;
      var pnbrs = neighbors(pr, pc);
      for (var ni = 0; ni < pnbrs.length; ni++) {
        var nk = key(pnbrs[ni][0], pnbrs[ni][1]); if (seen.has(nk)) continue;
        if (!passable(pnbrs[ni][0], pnbrs[ni][1], rows, cols, walls, blocked)) continue;
        seen.add(nk); q.push(pnbrs[ni]);
      }
    }
    sizes.push(size);
  }
  sizes.sort(function(a, b) { return b - a; });
  return sizes;
}

function isCorner(r, c, rows, cols, walls, extraWalls) {
  var n = !passable(r - 1, c, rows, cols, walls, extraWalls), s = !passable(r + 1, c, rows, cols, walls, extraWalls);
  var w = !passable(r, c - 1, rows, cols, walls, extraWalls), e = !passable(r, c + 1, rows, cols, walls, extraWalls);
  return (n || s) && (w || e);
}

function countFreeNeighbors(r, c, rows, cols, walls, extraWalls) {
  var n = 0, nbrs = neighbors(r, c);
  for (var i = 0; i < nbrs.length; i++) if (passable(nbrs[i][0], nbrs[i][1], rows, cols, walls, extraWalls)) n++;
  return n;
}

function analyzeLevel(level) {
  var rows = level.rows, cols = level.cols;
  var sets = level._sets;
  var walls = sets.walls, goals = sets.goals, immovable = sets.immovable, boxSet = sets.boxes, agentSet = sets.agents;

  var staticAP = articulationPoints(rows, cols, walls, immovable);
  var dynamicAP = articulationPoints(rows, cols, walls, boxSet);

  var candidates = [], occupiedRisks = [];
  var counts = { safe: 0, caution: 0, danger: 0, blocked: 0, staticArticulation: staticAP.size, dynamicArticulation: dynamicAP.size, occupiedGateClusters: 0 };

  for (var r = 0; r < rows; r++) {
    for (var c = 0; c < cols; c++) {
      var kk = key(r, c);
      if (walls.has(kk)) continue;

      if (boxSet.has(kk) || agentSet.has(kk) || goals.has(kk)) {
        counts.blocked++;
        var occReasons = [], occSeverity = 'safe', occScore = 0;
        var occStaticParts = [], occCurrentOpenParts = [];
        if (staticAP.has(kk)) { occSeverity = 'danger'; occScore = Math.max(occScore, 95); occReasons.push('static-door: occupied cell is an articulation point'); occStaticParts = occStaticParts.concat(componentSizesAfterRemoval(r, c, rows, cols, walls, immovable)); }
        if (dynamicAP.has(kk)) { if (occSeverity !== 'danger') occSeverity = 'caution'; occScore = Math.max(occScore, 75); occReasons.push('current-door: occupied cell is a door with current box layout'); occCurrentOpenParts = occCurrentOpenParts.concat(componentSizesAfterRemoval(r, c, rows, cols, walls, new Set())); }
        if (!occReasons.length) occReasons.push('occupied-but-passable');
        var occupant = {};
        for (var bi = 0; bi < level.boxes.length; bi++) { if (level.boxes[bi].r === r && level.boxes[bi].c === c) { occupant = { kind: 'box', label: level.boxes[bi].type }; break; } }
        for (var ai = 0; ai < level.agents.length; ai++) { if (level.agents[ai].r === r && level.agents[ai].c === c) { occupant = { kind: 'agent', label: level.agents[ai].id }; break; } }
        if (goals.has(kk)) occupant = { kind: 'goal', label: 'goal' };
        if (occSeverity === 'danger') counts.occupiedGateClusters++;
        counts[occSeverity]++;
        occupiedRisks.push({ r: r, c: c, severity: occSeverity, score: occScore, reasons: occReasons, staticSplit: occStaticParts, currentOpenSplit: occCurrentOpenParts, occupant: occupant });
        continue;
      }

      var reasons = [], severity = 'safe', score = 0;
      var staticParts = [], dynamicParts = [];
      if (staticAP.has(kk)) { severity = 'danger'; score = Math.max(score, 95); reasons.push('static-door: removing this cell splits the wall-only map'); staticParts = componentSizesAfterRemoval(r, c, rows, cols, walls, immovable); }
      if (dynamicAP.has(kk)) { if (severity !== 'danger') severity = 'caution'; score = Math.max(score, 75); reasons.push('current-door: with initial boxes as blockers, this cell splits walking space'); dynamicParts = componentSizesAfterRemoval(r, c, rows, cols, walls, boxSet); }
      if (isCorner(r, c, rows, cols, walls, immovable)) { if (severity === 'safe') severity = 'caution'; score = Math.max(score, 60); reasons.push('corner: easy to create a stuck box if it is not a real goal'); }
      if (!reasons.length) reasons.push('no static risk found');
      counts[severity]++;
      candidates.push({ r: r, c: c, severity: severity, score: score, freeNeighbors: countFreeNeighbors(r, c, rows, cols, walls, boxSet), reasons: reasons, staticSplit: staticParts, dynamicSplit: dynamicParts });
    }
  }

  var result = {};
  var keys = Object.keys(level);
  for (var ki = 0; ki < keys.length; ki++) { if (keys[ki] !== '_sets') result[keys[ki]] = level[keys[ki]]; }
  result.analysis = {
    coordinateSystem: 'Java zero-based grid coordinates: top-left level grid cell is (0,0)',
    staticArticulation: setToSortedList(staticAP),
    dynamicArticulation: setToSortedList(dynamicAP),
    candidates: candidates, occupiedRisks: occupiedRisks, summary: counts,
  };
  return result;
}

function setToSortedList(s) {
  var arr = [];
  s.forEach(function(k) { var p = k.split(','); arr.push({ r: Number(p[0]), c: Number(p[1]) }); });
  arr.sort(function(a, b) { return a.r - b.r || a.c - b.c; });
  return arr;
}

// ---- Board rendering ----

function rebuildAxes(report) {
  els.axisTopInner.innerHTML = ''; els.axisLeftInner.innerHTML = '';
  els.axisTopInner.style.gridTemplateColumns = 'repeat(' + report.cols + ', var(--axis))';
  els.axisTopInner.style.gridTemplateRows = 'var(--axis)';
  els.axisLeftInner.style.gridTemplateColumns = 'var(--axis)';
  els.axisLeftInner.style.gridTemplateRows = 'repeat(' + report.rows + ', var(--axis))';
  for (var c = 0; c < report.cols; c++) els.axisTopInner.appendChild(div('axisCell', String(c)));
  for (var r = 0; r < report.rows; r++) els.axisLeftInner.appendChild(div('axisCell', String(r)));
}

function renderBoard() {
  var report = currentReport();
  if (!report) return showEmpty();
  rebuildAxes(report);

  var walls = posSet(report.walls);
  var goals = posSet(report.goalCells);
  var agents = new Map(report.agents.map(function(a) { return [key(a.r, a.c), a]; }));
  var boxes = new Map(report.boxes.map(function(b) { return [key(b.r, b.c), b]; }));
  var agentGoals = new Map(report.agentGoals.map(function(g) { return [key(g.r, g.c), g]; }));
  var boxGoals = new Map(report.boxGoals.map(function(g) { return [key(g.r, g.c), g]; }));
  var candidateMap = new Map(report.analysis.candidates.map(function(c) { return [key(c.r, c.c), c]; }));
  var occupiedRiskMap = new Map((report.analysis.occupiedRisks || []).map(function(c) { return [key(c.r, c.c), c]; }));

  var board = document.createElement('div');
  board.className = 'board';
  board.style.gridTemplateColumns = 'repeat(' + report.cols + ', var(--cell))';

  for (var r = 0; r < report.rows; r++) {
    for (var c = 0; c < report.cols; c++) {
      var kk = key(r, c);
      var cell = document.createElement('div');
      cell.className = 'cell'; cell.dataset.r = r; cell.dataset.c = c;

      if (walls.has(kk)) { cell.classList.add('wall'); board.appendChild(cell); continue; }

      var goal = boxGoals.get(kk) || agentGoals.get(kk);
      if (goal && els.showGoals.checked) { cell.classList.add('goal'); cell.appendChild(goalCharEl(goal.type || goal.id)); }

      var candidate = candidateMap.get(kk);
      if (candidate && shouldShowSeverity(candidate.severity)) cell.classList.add(candidate.severity);
      var or = occupiedRiskMap.get(kk);
      if (or && shouldShowSeverity(or.severity)) cell.classList.add(or.severity);

      if (els.showObjects.checked) {
        var box = boxes.get(kk), agent = agents.get(kk);
        if (box) cell.appendChild(token(box.type, box.color, 'box'));
        if (agent) cell.appendChild(token(agent.id, agent.color, 'agent'));
      }
      board.appendChild(cell);
    }
  }
  els.panSurface.innerHTML = '';
  els.panSurface.appendChild(board);
}

function shouldShowSeverity(s) {
  return (s === 'safe' && els.showSafe.checked) || (s === 'caution' && els.showCaution.checked) || (s === 'danger' && els.showDanger.checked);
}

function cellInfo(report, r, c) {
  var kk = key(r, c);
  var walls = posSet(report.walls);
  if (walls.has(kk)) return { summary: 'wall', tooltip: '(' + r + ', ' + c + ')\nwall' };
  var byKey = new Map(report.analysis.candidates.map(function(x) { return [key(x.r, x.c), x]; }));
  var occByKey = new Map((report.analysis.occupiedRisks || []).map(function(x) { return [key(x.r, x.c), x]; }));
  var cand = byKey.get(kk);
  if (cand) {
    var st = [];
    if (cand.staticSplit.length) st.push('static split: ' + cand.staticSplit.join(' / '));
    if (cand.dynamicSplit.length) st.push('current split: ' + cand.dynamicSplit.join(' / '));
    return { summary: cand.severity + ' parking candidate', tooltip: '(' + r + ', ' + c + ')\n' + cand.severity.toUpperCase() + ' score=' + cand.score + '\nfree neighbors now: ' + cand.freeNeighbors + '\n' + cand.reasons.join('\n') + (st.length ? '\n' + st.join('\n') : '') };
  }
  var or = occByKey.get(kk);
  if (or) {
    var st2 = [];
    if (or.staticSplit.length) st2.push('static split: ' + or.staticSplit.join(' / '));
    if (or.currentOpenSplit.length) st2.push('opens current regions: ' + or.currentOpenSplit.join(' / '));
    var occ = or.occupant || {};
    return { summary: or.severity + ' occupied ' + (occ.kind || 'cell'), tooltip: '(' + r + ', ' + c + ')\n' + or.severity.toUpperCase() + ' occupied ' + (occ.kind || 'cell') + ' ' + (occ.label || '') + '\nscore=' + or.score + '\n' + or.reasons.join('\n') + (st2.length ? '\n' + st2.join('\n') : '') };
  }
  var goal = posSet(report.goalCells).has(kk);
  var box = report.boxes.find(function(x) { return x.r === r && x.c === c; });
  var agent = report.agents.find(function(x) { return x.r === r && x.c === c; });
  var parts = [];
  if (goal) parts.push('goal cell: no parking');
  if (box) parts.push('box ' + box.type);
  if (agent) parts.push('agent ' + agent.id);
  return { summary: parts.length ? parts.join(', ') : 'non-candidate floor', tooltip: '(' + r + ', ' + c + ')\n' + (parts.length ? parts.join('\n') : 'non-candidate floor') };
}

// ---- Zoom / Pan ----

function applyZoom() {
  var cz = Math.round(BASE_CELL * state.zoom);
  document.documentElement.style.setProperty('--cell', cz + 'px');
  document.documentElement.style.setProperty('--axis', cz + 'px');
  els.zoomInfo.textContent = Math.round(state.zoom * 100) + '%';
}
function applyPan() {
  els.panSurface.style.transform = 'translate(' + state.pan.x + 'px, ' + state.pan.y + 'px)';
  els.axisTopInner.style.transform = 'translateX(' + state.pan.x + 'px)';
  els.axisLeftInner.style.transform = 'translateY(' + state.pan.y + 'px)';
}
function resetView() {
  state.zoom = 1; applyZoom();
  var board = els.panSurface.firstElementChild;
  if (!board) { state.pan.x = 0; state.pan.y = 0; applyPan(); return; }
  requestAnimationFrame(function() {
    state.pan.x = Math.round((els.panViewport.clientWidth - board.offsetWidth) / 2);
    state.pan.y = Math.round((els.panViewport.clientHeight - board.offsetHeight) / 2);
    applyPan();
  });
}
function zoomAt(cx, cy, f) {
  var next = clamp(state.zoom * f, MIN_ZOOM, MAX_ZOOM), real = next / state.zoom;
  if (real === 1) return;
  var rect = els.panViewport.getBoundingClientRect();
  state.pan.x = (cx - rect.left) - ((cx - rect.left) - state.pan.x) * real;
  state.pan.y = (cy - rect.top) - ((cy - rect.top) - state.pan.y) * real;
  state.zoom = next; applyZoom(); applyPan();
}

// ---- Helpers ----

function token(text, color, kind) {
  var el = document.createElement('div');
  el.className = 'token ' + kind;
  el.textContent = text;
  el.style.background = COLOR_HEX[color] || COLOR_HEX.DEFAULT;
  return el;
}
function goalCharEl(text) { var el = document.createElement('div'); el.className = 'goalChar'; el.textContent = text; return el; }
function div(cls, text) { var el = document.createElement('div'); el.className = cls; el.textContent = text || ''; return el; }
function posSet(items) { return new Set(items.map(function(p) { return key(p.r, p.c); })); }
function key(r, c) { return r + ',' + c; }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function escapeHtml(s) { return String(s).replace(/[&<>"']/g, function(ch) { return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[ch]; }); }
