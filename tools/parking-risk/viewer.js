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
  directoryInput: document.getElementById('directoryInput'),
  levelSearchInput: document.getElementById('levelSearchInput'),
  prevLevelBtn: document.getElementById('prevLevelBtn'),
  nextLevelBtn: document.getElementById('nextLevelBtn'),
  clearLibraryBtn: document.getElementById('clearLibraryBtn'),
  libraryInfo: document.getElementById('libraryInfo'),
  levelList: document.getElementById('levelList'),
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
  notificationToast: document.getElementById('notificationToast'),
  notificationTitle: document.getElementById('notificationTitle'),
  notificationMessage: document.getElementById('notificationMessage'),
  notificationCloseBtn: document.getElementById('notificationCloseBtn'),
  buildVersion: document.getElementById('buildVersion'),
};

const BASE_CELL = 24;
const MIN_ZOOM = 0.3;
const MAX_ZOOM = 4.0;
const STORAGE_KEY = 'parkingRiskLastReport';
const DIR_HANDLE_KEY = 'parkingRiskReportsDir';
const VIEWER_VERSION = 'v1.0.0';
const PARKING_RISK_DB_NAME = 'parking-risk';
const PARKING_RISK_DB_VERSION = 2;
const HANDLE_STORE_NAME = 'handles';
const VIEWER_STORE_NAME = 'viewer-state';
const LEVEL_LIBRARY_KEY = 'level-library';
const LEVEL_LIBRARY_LOCAL_STORAGE_KEY = 'parking-risk:level-library';
var notificationTimer = null;

const state = {
  reports: window.PARKING_RISK_REPORTS || [],
  activeIndex: 0,
  levelEntries: [],
  selectedEntryId: null,
  openDirectorySource: null,
  directoryAutoOpen: true,
  zoom: 1,
  pan: { x: 0, y: 0 },
  dirHandle: null,
};

init();

function init() {
  if (els.buildVersion) els.buildVersion.textContent = VIEWER_VERSION;
  initInputs();
  initNotifications();
  initBoardNavigation();

  scanReportsDir().then(function() {
    return loadReportsDirManifest();
  }).then(function() {
    return loadManifestReports();
  }).then(function() {
    syncInitialReportEntries();
  }).then(function() {
    return restoreStoredEntries();
  }).then(function() {
    renderLevelList();
    var lastName = localStorage.getItem(STORAGE_KEY);
    var idx = 0;
    if (lastName && state.reports.length > 0) {
      var found = state.reports.findIndex(function(r) { return r.name === lastName; });
      if (found >= 0) idx = found;
    }
    if (state.reports.length > 0) loadReport(idx);
    else showEmpty();
  }).catch(function(err) {
    showError(errorMessage(err), 'Unable to initialize viewer');
    showEmpty();
  });
}

function syncInitialReportEntries() {
  for (var i = 0; i < state.reports.length; i++) syncReportEntry(state.reports[i]);
}

function initInputs() {
  els.fileInput.addEventListener('change', function(e) {
    var file = e.target.files && e.target.files[0];
    if (file) loadSingleFile(file, requestReportsDirFromUserGesture());
    els.fileInput.value = '';
  });

  els.directoryInput.addEventListener('change', function(e) {
    var files = Array.prototype.slice.call(e.target.files || []).filter(isLevelFile);
    if (files.length > 0) loadDirectoryFiles(files, '', requestReportsDirFromUserGesture());
    else showError('No .lvl files were found in that directory.', 'Nothing to load');
    els.directoryInput.value = '';
  });

  els.levelSearchInput.addEventListener('input', renderLevelList);
  els.prevLevelBtn.addEventListener('click', function() { switchLevel(-1); });
  els.nextLevelBtn.addEventListener('click', function() { switchLevel(1); });
  els.clearLibraryBtn.addEventListener('click', clearLevelLibrary);
  els.levelList.addEventListener('click', function(e) {
    var deleteButton = closestElement(e.target, 'button[data-entry-delete-id]');
    if (deleteButton) {
      e.preventDefault();
      deleteEntryById(deleteButton.dataset.entryDeleteId);
      return;
    }

    var levelButton = closestElement(e.target, 'button[data-entry-id]');
    if (levelButton) {
      e.preventDefault();
      var entry = state.levelEntries.find(function(item) { return item.id === levelButton.dataset.entryId; });
      loadEntryById(levelButton.dataset.entryId, entry && entry.content ? requestReportsDirFromUserGesture() : null);
      return;
    }

    var toggleButton = closestElement(e.target, 'button[data-directory-source]');
    if (toggleButton) {
      e.preventDefault();
      toggleDirectory(toggleButton.dataset.directorySource);
    }
  });

  for (var k = 0; k < [els.showSafe, els.showCaution, els.showDanger, els.showGoals, els.showObjects].length; k++) {
    [els.showSafe, els.showCaution, els.showDanger, els.showGoals, els.showObjects][k].addEventListener('change', renderBoard);
  }
}

function initNotifications() {
  els.notificationCloseBtn.addEventListener('click', clearNotification);
  window.addEventListener('error', function(event) {
    showError(event.message || 'Unexpected viewer error.', 'Viewer error');
  });
  window.addEventListener('unhandledrejection', function(event) {
    showError(errorMessage(event.reason), 'Viewer error');
  });
}

function initBoardNavigation() {
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
    els.hoverInfo.textContent = '(' + r + ', ' + c + ') - ' + info.summary;
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

function upsertReport(report, preferFront) {
  if (!report || !report.name) return -1;
  var idx = state.reports.findIndex(function(r) { return r.name === report.name; });
  if (idx >= 0) {
    state.reports[idx] = report;
    if (preferFront && idx !== 0) {
      state.reports.splice(idx, 1);
      state.reports.unshift(report);
      return 0;
    }
    return idx;
  }
  if (preferFront) {
    state.reports.unshift(report);
    return 0;
  }
  state.reports.push(report);
  return state.reports.length - 1;
}

function makeEntryFromFile(file, content, fallbackSource) {
  var path = file.webkitRelativePath || file.name;
  var source = directoryRootName(file) || fallbackSource || 'File';
  return normalizeEntry({
    name: stem(file.name),
    fileName: file.name,
    path: path,
    source: source,
    content: content,
    kind: 'level',
    persistent: true,
  });
}

function makeEntryFromReport(report) {
  var path = report.path || (report.name + '.lvl');
  return normalizeEntry({
    name: report.name || stem(path),
    fileName: fileName(path) || ((report.name || 'report') + '.lvl'),
    path: path,
    source: sourceFromPath(path) || 'Reports',
    content: '',
    reportName: report.name,
    kind: 'report',
    persistent: false,
  });
}

function syncReportEntry(report) {
  if (!report || !report.name) return;
  var reportEntry = makeEntryFromReport(report);
  var reportKey = entryKey(reportEntry);
  var existing = state.levelEntries.find(function(entry) { return entryKey(entry) === reportKey; });
  if (existing && existing.content) {
    existing.reportName = report.name;
    renderLevelList();
    return;
  }
  addLevelEntries([reportEntry], { persist: false });
}

function addLevelEntries(entries, options) {
  options = options || {};
  var shouldPersist = options.persist !== false;
  var selectedKey = state.selectedEntryId ? entryKey(state.levelEntries.find(function(entry) { return entry.id === state.selectedEntryId; })) : '';
  var byKey = new Map();
  for (var i = 0; i < state.levelEntries.length; i++) {
    var current = normalizeEntry(state.levelEntries[i]);
    byKey.set(entryKey(current), current);
  }
  for (var j = 0; j < entries.length; j++) {
    var normalized = normalizeEntry(entries[j]);
    byKey.set(entryKey(normalized), normalized);
  }
  state.levelEntries = Array.from(byKey.values()).sort(function(a, b) {
    return a.path.localeCompare(b.path, undefined, { sensitivity: 'base' });
  });
  if (selectedKey && !state.levelEntries.some(function(entry) { return entry.id === state.selectedEntryId; })) {
    var replacement = state.levelEntries.find(function(entry) { return entryKey(entry) === selectedKey; });
    if (replacement) state.selectedEntryId = replacement.id;
  }
  renderLevelList();
  if (shouldPersist) {
    persistStoredEntries().catch(function(error) {
      console.warn('Unable to store level library:', error);
      showError('The directory was read, but the browser could not remember it for the next refresh.', 'Storage unavailable');
    });
  }
}

function normalizeEntry(entry) {
  var path = String(entry.path || entry.fileName || entry.name || 'level.lvl');
  var displayFileName = entry.fileName || fileName(path) || ((entry.name || stem(path)) + '.lvl');
  var source = sourceFromPath(path) || entry.source || 'Levels';
  var normalized = {
    name: entry.name || stem(displayFileName),
    fileName: displayFileName,
    path: path,
    source: source,
    content: String(entry.content || ''),
    reportName: entry.reportName || '',
    kind: entry.kind || (entry.reportName ? 'report' : 'level'),
    persistent: entry.persistent !== false,
  };
  return Object.assign(normalized, { id: levelEntryId(normalized) });
}

function levelEntryId(entry) {
  return 'level:' + entryKey(entry);
}

function entryKey(entry) {
  if (!entry) return '';
  var path = canonicalPath(entry.path || entry.fileName || entry.name);
  var grouped = path.match(/(?:^|\/)(levels|complevels|complevels26)\/(.+\.lvl)$/);
  if (grouped) return grouped[1] + '/' + grouped[2];
  var source = normalizePathPart(entry.source || 'Levels');
  return source + '/' + (path || normalizePathPart(entry.fileName || entry.name || 'level.lvl'));
}

function sourceFromPath(path) {
  var grouped = canonicalPath(path).match(/(?:^|\/)(levels|complevels|complevels26)\/.+\.lvl$/);
  return grouped ? grouped[1] : '';
}

function canonicalPath(path) {
  return normalizePathPart(path).replace(/^\.\//, '');
}

function normalizePathPart(value) {
  return String(value || '')
    .replace(/\\/g, '/')
    .replace(/\/+/g, '/')
    .replace(/^\/+|\/+$/g, '')
    .toLowerCase();
}

function renderLevelList() {
  var filtered = filteredLevelEntries();
  var groups = groupedEntries(filtered);
  syncOpenDirectorySource(groups);

  els.levelList.innerHTML = '';
  for (var i = 0; i < groups.length; i++) {
    var group = groups[i];
    var isOpen = group.source === state.openDirectorySource;
    var card = document.createElement('section');
    card.className = 'directoryCard' + (isOpen ? ' open' : ' collapsed');

    var header = document.createElement('div');
    header.className = 'directoryCardHeader';
    var title = document.createElement('strong');
    title.textContent = group.source;
    var count = document.createElement('span');
    count.textContent = group.entries.length + ' level' + (group.entries.length === 1 ? '' : 's');
    var toggle = document.createElement('button');
    toggle.type = 'button';
    toggle.className = 'directoryToggleBtn';
    toggle.dataset.directorySource = group.source;
    toggle.setAttribute('aria-expanded', String(isOpen));
    toggle.title = isOpen ? 'Collapse directory' : 'Show directory';
    toggle.textContent = isOpen ? 'v' : '>';
    header.appendChild(title);
    header.appendChild(count);
    header.appendChild(toggle);

    var rows = document.createElement('div');
    rows.className = 'directoryLevels';
    rows.hidden = !isOpen;
    for (var j = 0; j < group.entries.length; j++) {
      var entry = group.entries[j];
      var row = document.createElement('div');
      row.className = 'levelRow' + (entry.id === state.selectedEntryId ? ' active' : '');
      row.title = entry.path;

      var button = document.createElement('button');
      button.type = 'button';
      button.className = 'levelRowMain';
      button.dataset.entryId = entry.id;
      var name = document.createElement('strong');
      name.textContent = entry.fileName || fileName(entry.path) || (entry.name + '.lvl');
      button.appendChild(name);

      var deleteButton = document.createElement('button');
      deleteButton.type = 'button';
      deleteButton.className = 'levelDeleteBtn';
      deleteButton.dataset.entryDeleteId = entry.id;
      deleteButton.title = 'Remove ' + name.textContent + ' from list';
      deleteButton.setAttribute('aria-label', 'Remove ' + name.textContent + ' from list');
      deleteButton.textContent = 'x';

      row.appendChild(button);
      row.appendChild(deleteButton);
      rows.appendChild(row);
    }

    card.appendChild(header);
    card.appendChild(rows);
    els.levelList.appendChild(card);
  }

  if (state.levelEntries.length === 0) {
    els.libraryInfo.textContent = 'Add a directory to list .lvl files here.';
  } else if (filtered.length === 0) {
    els.libraryInfo.textContent = 'No matches in ' + state.levelEntries.length + ' read levels.';
  } else if (groups.length > 1) {
    var openGroup = groups.find(function(group) { return group.source === state.openDirectorySource; });
    els.libraryInfo.textContent = openGroup
      ? filtered.length + '/' + state.levelEntries.length + ' matches. Showing ' + openGroup.source + '.'
      : filtered.length + '/' + state.levelEntries.length + ' matches. All directories collapsed.';
  } else {
    els.libraryInfo.textContent = filtered.length + '/' + state.levelEntries.length + ' levels shown.';
  }
  updateLevelNavButtons(filtered);
}

function filteredLevelEntries() {
  var query = normalizeText(els.levelSearchInput.value);
  return state.levelEntries.filter(function(entry) { return levelMatchesQuery(entry, query); });
}

function levelMatchesQuery(entry, query) {
  if (!query) return true;
  return normalizeText(entry.name + ' ' + entry.path + ' ' + entry.source).indexOf(query) >= 0;
}

function groupedEntries(entries) {
  var bySource = new Map();
  for (var i = 0; i < entries.length; i++) {
    var source = entries[i].source || 'Levels';
    if (!bySource.has(source)) bySource.set(source, []);
    bySource.get(source).push(entries[i]);
  }
  return Array.from(bySource.entries())
    .sort(function(a, b) { return a[0].localeCompare(b[0], undefined, { sensitivity: 'base' }); })
    .map(function(item) {
      return {
        source: item[0],
        entries: item[1].sort(function(a, b) {
          return (a.fileName || a.path).localeCompare(b.fileName || b.path, undefined, { sensitivity: 'base' });
        }),
      };
    });
}

function syncOpenDirectorySource(groups) {
  if (groups.length === 0) {
    state.openDirectorySource = null;
    return;
  }
  if (state.openDirectorySource && groups.some(function(group) { return group.source === state.openDirectorySource; })) return;
  if (!state.directoryAutoOpen) {
    state.openDirectorySource = null;
    return;
  }
  if (state.selectedEntryId) {
    var selectedGroup = groups.find(function(group) {
      return group.entries.some(function(entry) { return entry.id === state.selectedEntryId; });
    });
    if (selectedGroup) {
      state.openDirectorySource = selectedGroup.source;
      return;
    }
  }
  state.openDirectorySource = groups[0].source;
}

function toggleDirectory(source) {
  if (!source) return;
  if (state.openDirectorySource === source) {
    state.openDirectorySource = null;
    state.directoryAutoOpen = false;
  } else {
    state.openDirectorySource = source;
    state.directoryAutoOpen = true;
  }
  renderLevelList();
}

function updateLevelNavButtons(filtered) {
  filtered = filtered || filteredLevelEntries();
  var disabled = filtered.length <= 1;
  els.prevLevelBtn.disabled = disabled;
  els.nextLevelBtn.disabled = disabled;
}

function switchLevel(direction) {
  var filtered = filteredLevelEntries();
  if (filtered.length === 0) {
    showError('No levels match the current filter.', 'No level to switch');
    return;
  }
  var currentIndex = filtered.findIndex(function(entry) { return entry.id === state.selectedEntryId; });
  var nextIndex = currentIndex < 0
    ? (direction < 0 ? filtered.length - 1 : 0)
    : (currentIndex + direction + filtered.length) % filtered.length;
  loadEntry(filtered[nextIndex], filtered[nextIndex].content ? requestReportsDirFromUserGesture() : null);
}

function scrollActiveLevelIntoView() {
  requestAnimationFrame(function() {
    var active = els.levelList.querySelector('.levelRow.active');
    if (active && active.scrollIntoView) active.scrollIntoView({ block: 'nearest', inline: 'nearest' });
  });
}

function clearLevelLibrary() {
  state.levelEntries = state.levelEntries.filter(function(entry) { return entry.kind === 'report'; });
  state.selectedEntryId = null;
  state.openDirectorySource = null;
  state.directoryAutoOpen = true;
  renderLevelList();
  persistStoredEntries().catch(function(error) {
    console.warn('Unable to clear stored level library:', error);
  });
}

function deleteEntryById(entryId) {
  var entry = state.levelEntries.find(function(item) { return item.id === entryId; });
  if (!entry) {
    showError('This level entry is no longer available.', 'Unable to delete level');
    return;
  }
  var label = entry.fileName || fileName(entry.path) || entry.name || 'this level';
  if (!window.confirm('Remove ' + label + ' from the level list?\n\nThis does not delete the .lvl file or report from disk.')) return;

  var filteredBefore = filteredLevelEntries();
  var deletedIndex = filteredBefore.findIndex(function(item) { return item.id === entryId; });
  var wasSelected = state.selectedEntryId === entryId;
  state.levelEntries = state.levelEntries.filter(function(item) { return item.id !== entryId; });
  if (wasSelected) state.selectedEntryId = null;
  if (!state.levelEntries.some(function(item) { return item.source === state.openDirectorySource; })) {
    state.openDirectorySource = null;
    state.directoryAutoOpen = true;
  }

  persistStoredEntries().catch(function(error) {
    console.warn('Unable to store level library after deletion:', error);
  });

  if (wasSelected) {
    var filteredAfter = filteredLevelEntries();
    if (filteredAfter.length > 0) {
      var nextIndex = deletedIndex < 0 ? 0 : Math.min(deletedIndex, filteredAfter.length - 1);
      loadEntry(filteredAfter[nextIndex], filteredAfter[nextIndex].content ? requestReportsDirFromUserGesture() : null);
    } else {
      renderLevelList();
    }
  } else {
    renderLevelList();
  }
}

function loadReport(index, entryId) {
  state.activeIndex = index;
  var r = currentReport();
  if (r) {
    localStorage.setItem(STORAGE_KEY, r.name);
    if (entryId) state.selectedEntryId = entryId;
    else selectEntryForReport(r);
  }
  renderLevelList(); renderMeta(); renderBoard(); resetView();
}

function selectEntryForReport(report) {
  var reportKey = entryKey(makeEntryFromReport(report));
  var entry = state.levelEntries.find(function(item) { return entryKey(item) === reportKey; });
  if (entry) {
    state.selectedEntryId = entry.id;
    state.openDirectorySource = entry.source || state.openDirectorySource;
    state.directoryAutoOpen = true;
  }
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

function cacheBust(path) {
  return path + (path.indexOf('?') >= 0 ? '&' : '?') + 'v=' + Date.now();
}

// ---- .lvl loading + in-browser analysis ----

async function loadSingleFile(file, reportsDirPromise) {
  try {
    var text = await file.text();
    var entry = makeEntryFromFile(file, text, 'File');
    addLevelEntries([entry]);
    await loadEntry(entry, reportsDirPromise);
  } catch (err) {
    showError(errorMessage(err), 'Unable to load level');
    console.error(err);
  }
}

async function loadDirectoryFiles(files, sourceLabel, reportsDirPromise) {
  try {
    var entries = await Promise.all(files.map(async function(file) {
      var text = await file.text();
      return makeEntryFromFile(file, text, sourceLabel || directoryRootName(file) || 'Directory');
    }));
    addLevelEntries(entries);
    if (entries.length > 0) {
      await loadEntry(entries[0], reportsDirPromise);
      showNotice(entries.length + ' .lvl files added to the level list.', 'Directory read');
    }
  } catch (err) {
    showError(errorMessage(err), 'Unable to load directory');
    console.error(err);
  }
}

async function loadEntryById(entryId, reportsDirPromise) {
  var entry = state.levelEntries.find(function(item) { return item.id === entryId; });
  if (!entry) {
    showError('This level entry is no longer available. Read the directory again.', 'Unable to read level');
    return;
  }
  await loadEntry(entry, reportsDirPromise);
}

async function loadEntry(entry, reportsDirPromise) {
  var normalized = normalizeEntry(entry);
  if (!normalized.content.trim()) {
    var reportIndex = state.reports.findIndex(function(report) { return report.name === normalized.reportName; });
    if (reportIndex >= 0) {
      loadReport(reportIndex, normalized.id);
    } else {
      showError((normalized.fileName || normalized.name) + ' has no readable .lvl content. Read the file or directory again.', 'Unable to read level');
    }
    return;
  }

  try {
    var level = parseLvl(normalized.content, normalized.path);
    var report = analyzeLevel(level);
    report.path = normalized.path;
    report.source = normalized.source;
    state.selectedEntryId = normalized.id;
    state.openDirectorySource = normalized.source || state.openDirectorySource;
    state.directoryAutoOpen = true;
    state.activeIndex = upsertReport(report, true);
    localStorage.setItem(STORAGE_KEY, report.name);
    syncReportEntry(report);
    renderLevelList(); renderMeta(); renderBoard(); resetView(); scrollActiveLevelIntoView();
    await saveReportToDir(report, reportsDirPromise);
  } catch (err) {
    showError(errorMessage(err), 'Unable to compute parking risk');
    console.error(err);
  }
}

async function restoreStoredEntries() {
  try {
    var entries = await readStoredEntries();
    if (entries.length > 0) addLevelEntries(entries, { persist: false });
  } catch (error) {
    console.warn('Unable to restore level library:', error);
  }
}

async function persistStoredEntries() {
  var entries = state.levelEntries
    .filter(function(entry) { return entry.persistent !== false && entry.content; })
    .map(function(entry) {
      return {
        id: entry.id,
        name: entry.name,
        fileName: entry.fileName,
        path: entry.path,
        source: entry.source,
        content: entry.content,
        persistent: true,
        kind: 'level',
        reportName: entry.reportName || '',
      };
    });

  var record = { entries: entries, savedAt: new Date().toISOString() };
  try {
    await writeStoredEntriesToIndexedDb(record);
  } catch (error) {
    if (!localStorageAvailable()) throw error;
    localStorage.setItem(LEVEL_LIBRARY_LOCAL_STORAGE_KEY, JSON.stringify(record));
  }
}

async function readStoredEntries() {
  try {
    var record = await readStoredEntriesFromIndexedDb();
    if (Array.isArray(record && record.entries)) return record.entries.map(normalizeEntry);
  } catch (error) {
    console.warn('IndexedDB level library unavailable:', error);
  }

  if (!localStorageAvailable()) return [];
  var raw = localStorage.getItem(LEVEL_LIBRARY_LOCAL_STORAGE_KEY);
  if (!raw) return [];
  var record = JSON.parse(raw);
  return Array.isArray(record && record.entries) ? record.entries.map(normalizeEntry) : [];
}

function openParkingRiskDB() {
  return new Promise(function(resolve, reject) {
    if (!('indexedDB' in window)) {
      reject(new Error('IndexedDB is not available.'));
      return;
    }
    var req = indexedDB.open(PARKING_RISK_DB_NAME, PARKING_RISK_DB_VERSION);
    req.onupgradeneeded = function() {
      var db = req.result;
      if (!db.objectStoreNames.contains(HANDLE_STORE_NAME)) db.createObjectStore(HANDLE_STORE_NAME);
      if (!db.objectStoreNames.contains(VIEWER_STORE_NAME)) db.createObjectStore(VIEWER_STORE_NAME);
    };
    req.onsuccess = function() { resolve(req.result); };
    req.onerror = function() { reject(req.error || new Error('Unable to open parking-risk storage.')); };
  });
}

async function writeStoredEntriesToIndexedDb(record) {
  var db = await openParkingRiskDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction(VIEWER_STORE_NAME, 'readwrite');
    tx.objectStore(VIEWER_STORE_NAME).put(record, LEVEL_LIBRARY_KEY);
    tx.oncomplete = function() { db.close(); resolve(); };
    tx.onerror = function() {
      db.close();
      reject(tx.error || new Error('Unable to store level library.'));
    };
  });
}

async function readStoredEntriesFromIndexedDb() {
  var db = await openParkingRiskDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction(VIEWER_STORE_NAME, 'readonly');
    var request = tx.objectStore(VIEWER_STORE_NAME).get(LEVEL_LIBRARY_KEY);
    request.onsuccess = function() { resolve(request.result || null); };
    request.onerror = function() { reject(request.error || new Error('Unable to read level library.')); };
    tx.oncomplete = function() { db.close(); };
    tx.onerror = function() {
      db.close();
      reject(tx.error || new Error('Unable to read level library.'));
    };
  });
}

// ---- Scan reports/ directory for saved files ----

async function scanReportsDir() {
  try {
    var stored = await readDirHandleFromDB();
    if (!stored) return;
    if (await stored.requestPermission({ mode: 'readwrite' }) !== 'granted') return;
    var reportsDir = await resolveReportsDirHandle(stored, false);
    if (!reportsDir) return;
    state.dirHandle = reportsDir;

    var entries = [];
    var iter = reportsDir.values();
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
        // Load auto-saved manifest from reports directory
        if (entries[i].name === 'parking-risk-manifest.js') {
          var mfMatch = text.match(/window\.PARKING_RISK_MANIFEST\s*=\s*(\{[\s\S]*?\});\s*$/);
          if (mfMatch) {
            try {
              var dirManifest = JSON.parse(mfMatch[1]);
              var current = window.PARKING_RISK_MANIFEST || {};
              var mfKeys = Object.keys(dirManifest);
              for (var k = 0; k < mfKeys.length; k++) { current[mfKeys[k]] = dirManifest[mfKeys[k]]; }
              window.PARKING_RISK_MANIFEST = current;
            } catch (_) {}
          }
          continue;
        }
        // Extract JSON from the JS assignment
        var match = text.match(/window\.__PARKING_RISK_REPORT__\s*=\s*([\s\S]*?);\s*$/);
        if (!match) continue;
        var report = JSON.parse(match[1]);
        upsertReport(report, false);
        syncReportEntry(report);
      } catch (_) { /* skip unreadable files */ }
    }
  } catch (_) { /* dir not accessible */ }
}

// ---- File System Access API - save directly to reports/ ----

// Load the auto-updated manifest from reports/ if present.
// The reports/ copy is updated automatically when a new report is saved via the viewer.
function loadReportsDirManifest() {
  return new Promise(function(resolve) {
    var topManifest = window.PARKING_RISK_MANIFEST || {};
    var script = document.createElement('script');
    script.src = cacheBust('reports/parking-risk-manifest.js');
    script.onload = function() {
      var dirManifest = window.PARKING_RISK_MANIFEST || {};
      // Merge: reports-dir manifest takes precedence, keep top-level entries as fallback
      var topKeys = Object.keys(topManifest);
      for (var k = 0; k < topKeys.length; k++) {
        if (!dirManifest[topKeys[k]]) dirManifest[topKeys[k]] = topManifest[topKeys[k]];
      }
      window.PARKING_RISK_MANIFEST = dirManifest;
      document.head.removeChild(script);
      resolve();
    };
    script.onerror = function() {
      document.head.removeChild(script);
      resolve();
    };
    document.head.appendChild(script);
  });
}

function loadManifestReports() {
  var manifest = window.PARKING_RISK_MANIFEST;
  if (!manifest) return Promise.resolve();
  var entries = [];
  var keys = Object.keys(manifest);
  for (var i = 0; i < keys.length; i++) {
    entries.push({ name: keys[i], path: manifest[keys[i]] });
  }
  if (entries.length === 0) return Promise.resolve();
  function loadNext(idx) {
    if (idx >= entries.length) return Promise.resolve();
    return new Promise(function(resolve) {
      var script = document.createElement('script');
      script.src = cacheBust(entries[idx].path);
      script.onload = function() {
        var report = window.__PARKING_RISK_REPORT__;
        upsertReport(report, false);
        syncReportEntry(report);
        window.__PARKING_RISK_REPORT__ = undefined;
        document.head.removeChild(script);
        loadNext(idx + 1).then(resolve);
      };
      script.onerror = function() {
        document.head.removeChild(script);
        loadNext(idx + 1).then(resolve);
      };
      document.head.appendChild(script);
    });
  }
  return loadNext(0);
}

async function resolveReportsDirHandle(handle, createIfMissing) {
  if (!handle || handle.kind !== 'directory') return null;
  if (handle.name === 'reports') return handle;
  if (await isParkingRiskToolDir(handle)) {
    try {
      return await handle.getDirectoryHandle('reports', { create: Boolean(createIfMissing) });
    } catch (_) {
      return null;
    }
  }
  try {
    var parkingRisk = await handle.getDirectoryHandle('parking-risk', { create: false });
    if (await isParkingRiskToolDir(parkingRisk)) {
      return await parkingRisk.getDirectoryHandle('reports', { create: Boolean(createIfMissing) });
    }
  } catch (_) {}
  return null;
}

async function isParkingRiskToolDir(handle) {
  if (!handle || handle.kind !== 'directory' || handle.name !== 'parking-risk') return false;
  try {
    await handle.getFileHandle('viewer.js', { create: false });
    await handle.getFileHandle('index.html', { create: false });
    return true;
  } catch (_) {
    return false;
  }
}

async function getReportsDir() {
  if (state.dirHandle) return state.dirHandle;
  try {
    var stored = await readDirHandleFromDB();
    if (stored) {
      if (await stored.requestPermission({ mode: 'readwrite' }) === 'granted') {
        var storedReportsDir = await resolveReportsDirHandle(stored, false);
        if (storedReportsDir) {
          state.dirHandle = storedReportsDir;
          return storedReportsDir;
        }
      }
    }
  } catch (_) {}
  return null;
}

function requestReportsDirFromUserGesture() {
  if (state.dirHandle) return Promise.resolve(state.dirHandle);
  if (typeof window.showDirectoryPicker !== 'function') return Promise.resolve(null);
  try {
    return window.showDirectoryPicker({ mode: 'readwrite', id: 'parking-risk-tool-reports' })
      .then(async function(handle) {
        var reportsDir = await resolveReportsDirHandle(handle, true);
        if (!reportsDir) {
          window.alert('Select tools/parking-risk, or select tools/parking-risk/reports. The report was not saved.');
          return null;
        }
        state.dirHandle = reportsDir;
        await writeDirHandleToDB(reportsDir);
        return reportsDir;
      })
      .catch(function() { return null; });
  } catch (_) {
    return Promise.resolve(null);
  }
}

function dirHandleDB() {
  return openParkingRiskDB();
}

async function readDirHandleFromDB() {
  var db = await dirHandleDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction(HANDLE_STORE_NAME, 'readonly');
    var req = tx.objectStore(HANDLE_STORE_NAME).get(DIR_HANDLE_KEY);
    req.onsuccess = function() { resolve(req.result); };
    req.onerror = function() { reject(req.error); };
    tx.oncomplete = function() { db.close(); };
  });
}

async function writeDirHandleToDB(handle) {
  var db = await dirHandleDB();
  return new Promise(function(resolve, reject) {
    var tx = db.transaction(HANDLE_STORE_NAME, 'readwrite');
    tx.objectStore(HANDLE_STORE_NAME).put(handle, DIR_HANDLE_KEY);
    tx.oncomplete = function() { db.close(); resolve(); };
    tx.onerror = function() { reject(tx.error); };
  });
}

async function saveReportToDir(report, reportsDirPromise) {
  var name = report.name || 'unknown';
  var filename = 'parking-risk-' + safeFileStem(name) + '.js';
  var payload = JSON.stringify(report, null, 2);
  var content = '// parking-risk report - auto-generated\n// level: ' + name + '\nwindow.__PARKING_RISK_REPORT__ = ' + payload + ';\n';

  var dir = reportsDirPromise ? await reportsDirPromise : await getReportsDir();
  if (!dir) dir = await getReportsDir();
  if (dir) {
    try {
      var fileHandle = await dir.getFileHandle(filename, { create: true });
      var writable = await fileHandle.createWritable();
      await writable.write(content);
      await writable.close();

      // Update in-memory manifest and save to reports directory
      var manifest = window.PARKING_RISK_MANIFEST || {};
      manifest[name] = 'reports/' + filename;
      window.PARKING_RISK_MANIFEST = manifest;
      try {
        var mfContent = '// Parking Risk manifest - maps level name to report file path.\n// Auto-updated on save.\nwindow.PARKING_RISK_MANIFEST = ' + JSON.stringify(manifest, null, 2) + ';\n';
        var mfHandle = await dir.getFileHandle('parking-risk-manifest.js', { create: true });
        var mfWritable = await mfHandle.createWritable();
        await mfWritable.write(mfContent);
        await mfWritable.close();
      } catch (_) {}

      appendMetaLine('Saved to reports/' + filename);
      showNotice('Saved to tools/parking-risk/reports/' + filename, 'Report saved');
      return;
    } catch (err) {
      state.dirHandle = null;
      console.warn('Unable to save report:', err);
      showError('The report was computed, but the viewer could not write to tools/parking-risk/reports. Select the parking-risk folder again on the next save.', 'Report not saved');
    }
  }
  showError('Grant access to tools/parking-risk or tools/parking-risk/reports to write this report locally.', 'Report not saved');
  appendMetaLine('Report not saved. Grant access to tools/parking-risk or tools/parking-risk/reports to write reports/' + filename + '.');
}

function appendMetaLine(text) {
  if (els.meta) els.meta.innerHTML += '<br>' + escapeHtml(text);
}

// ---- .lvl parser ----

function parseLvl(text, sourcePath) {
  var fallbackPath = sourcePath || 'level.lvl';
  var lines = text.split(/\r?\n/);
  var section = null, levelName = stem(fallbackPath);
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
    name: levelName, path: fallbackPath, rows: rows, cols: cols,
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
function errorMessage(error) { return error && error.message ? error.message : String(error || 'Unknown error.'); }

function showError(message, title) {
  showNotification(message, title || 'Something needs attention', 0);
}

function showNotice(message, title) {
  showNotification(message, title || 'Done', 2000);
}

function showNotification(message, title, timeoutMs) {
  if (!els.notificationToast) return;
  if (notificationTimer) window.clearTimeout(notificationTimer);
  notificationTimer = null;
  els.notificationTitle.textContent = title;
  els.notificationMessage.textContent = message;
  els.notificationToast.hidden = false;
  if (timeoutMs) notificationTimer = window.setTimeout(clearNotification, timeoutMs);
}

function clearNotification() {
  if (!els.notificationToast) return;
  if (notificationTimer) window.clearTimeout(notificationTimer);
  notificationTimer = null;
  els.notificationToast.hidden = true;
  els.notificationTitle.textContent = 'Something needs attention';
  els.notificationMessage.textContent = '';
}

function localStorageAvailable() {
  try {
    var keyName = 'parking-risk:storage-test';
    localStorage.setItem(keyName, '1');
    localStorage.removeItem(keyName);
    return true;
  } catch (_) {
    return false;
  }
}

function directoryRootName(file) {
  var path = file.webkitRelativePath || '';
  var parts = path.split('/');
  return parts[0] || '';
}

function isLevelFile(file) {
  return Boolean(file && file.name && file.name.toLowerCase().endsWith('.lvl'));
}

function stem(path) {
  var f = fileName(path);
  return f.replace(/\.[^.]+$/, '') || f || 'Level';
}

function fileName(path) {
  return String(path || '').split(/[\\/]/).pop() || '';
}

function normalizeText(value) {
  return String(value || '').trim().toLowerCase();
}

function closestElement(target, selector) {
  var node = target;
  while (node && node !== document) {
    if (node instanceof Element && node.matches && node.matches(selector)) return node;
    node = node.parentElement || node.parentNode;
  }
  return null;
}

function safeFileStem(value) {
  var stemValue = String(value || 'unknown')
    .replace(/[<>:"/\\|?*\x00-\x1F]/g, '_')
    .replace(/\s+/g, '_')
    .replace(/_+/g, '_')
    .replace(/^_+|_+$/g, '');
  return stemValue || 'unknown';
}
