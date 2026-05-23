// Exact palette extracted from server.jar:
// dk/dtu/compute/mavis/domain/gridworld/hospital/Colors.class static initializer.
// Do not adjust - these are the canonical Mavis RGB values.
const colors = {
  BLUE:      '#3050ff',  // (48, 80, 255)
  RED:       '#ff0000',
  CYAN:      '#00ffff',
  PURPLE:    '#6000b0',
  GREEN:     '#00ff00',
  ORANGE:    '#ff8000',
  PINK:      '#f060c0',
  GREY:      '#707070',
  LIGHTBLUE: '#70c0ff',
  BROWN:     '#603000',
  DEFAULT:   '#c0c0c0',  // matches LIGHT_GRAY floor as a sane fallback
};

let replay = null;
let step = 0;
let timer = null;
let highlightCoord = null;
let notificationTimer = null;
let tutorialStepIndex = 0;
let indexedReplayEvents = new Map();
let indexedReplayEventTotal = 0;
let focusedAgentId = null;
let lockViewEnabled = true;
let agentPanelHeight = 260;

const TUTORIAL_STORAGE_KEY = 'aims-replay-viewer:onboarding-seen';
const FONT_SIZE_STORAGE_KEY = 'aims-replay-viewer:font-size-px';
const LOG_FONT_SIZE_STORAGE_KEY = 'aims-replay-viewer:log-font-size-px';
const LAST_REPLAY_DB_NAME = 'aims-replay-viewer';
const LAST_REPLAY_DB_VERSION = 1;
const LAST_REPLAY_STORE_NAME = 'viewer-state';
const LAST_REPLAY_KEY = 'last-opened-replay';
const LAST_REPLAY_LOCAL_STORAGE_KEY = 'aims-replay-viewer:last-opened-replay';
const DEFAULT_UI_FONT_SIZE = 12;
const DEFAULT_LOG_FONT_SIZE = 11;
const MIN_UI_FONT_SIZE = 9;
const MAX_UI_FONT_SIZE = 24;
const MIN_LOG_FONT_SIZE = 8;
const MAX_LOG_FONT_SIZE = 24;
const DEFAULT_AGENT_PANEL_HEIGHT = 260;
const MAX_AGENT_PANEL_HEIGHT = 420;
const tutorialSteps = [
  {
    title: 'Load a replay',
    body: 'The newest generated replay loads automatically when available. You can also use Load replay JSON or drag a JSON file onto the drop zone.'
  },
  {
    title: 'Play the run',
    body: 'Use Play, Previous, Next, the slider, or the step number field to move through the replay one server frame at a time.'
  },
  {
    title: 'Navigate the board',
    body: 'Drag empty board space to pan, use the mouse wheel to zoom, and use the lower-right reset button to restore the default view.'
  },
  {
    title: 'Track objects',
    body: 'Type values like agent0 or boxA to highlight an object and its trail through the replay. Use Highlight coordinate to mark a specific row,column cell.'
  },
  {
    title: 'Inspect agent intent',
    body: 'Hover an agent token to see field-labeled planner intent: phase, subgoal, subgoal type, progress, planned action, server action, and movement facts.'
  },
  {
    title: 'Monitor one agent',
    body: 'Use the lower-right Agent menu to pin one agent. The monitor keeps action, intent, and movement facts visible; use Lock View at the bottom of the monitor to follow that agent on the board.'
  },
  {
    title: 'Adjust text',
    body: 'Use the lower-right px control to open Display settings. Interface font size changes the page chrome; Log font size changes Highlights and the pinned agent monitor.'
  },
  {
    title: 'Review highlights',
    body: 'Use the lower-right Highlights button to inspect high-signal current-step events such as rejected actions, regressions, blockers, conflicts, and subgoal evaluations.'
  }
];

const els = {
  app:           document.querySelector('.app'),
  boardWrap:     document.querySelector('.boardWrap'),
  meta:          document.getElementById('meta'),
  fileInput:     document.getElementById('fileInput'),
  dropZone:      document.getElementById('dropZone'),
  playBtn:       document.getElementById('playBtn'),
  prevBtn:       document.getElementById('prevBtn'),
  nextBtn:       document.getElementById('nextBtn'),
  slider:        document.getElementById('stepSlider'),
  stepInput:     document.getElementById('stepInput'),
  stepText:      document.getElementById('stepText'),
  trackInput:    document.getElementById('trackInput'),
  coordInput:    document.getElementById('coordInput'),
  coordGoBtn:    document.getElementById('coordGoBtn'),
  coordClearBtn: document.getElementById('coordClearBtn'),
  autoPlayInput: document.getElementById('autoPlayInput'),
  speedInput:    document.getElementById('speedInput'),
  actions:       document.getElementById('actions'),
  hoverInfo:     document.getElementById('hoverInfo'),
  boards:        document.getElementById('boards'),
  panViewport:   document.getElementById('panViewport'),
  panSurface:    document.getElementById('panSurface'),
  axisTopInner:  document.getElementById('axisTopInner'),
  axisLeftInner: document.getElementById('axisLeftInner'),
  hoverTip:      document.getElementById('hoverTip'),
  togglePanel:   document.getElementById('togglePanel'),
  statusInfo:    document.getElementById('statusInfo'),
  zoomInfo:      document.getElementById('zoomInfo'),
  resetViewBtn:  document.getElementById('resetView'),
  stepEventsBtn: document.getElementById('stepEventsBtn'),
  agentMenuBtn: document.getElementById('agentMenuBtn'),
  agentMenuPanel: document.getElementById('agentMenuPanel'),
  agentMenuList: document.getElementById('agentMenuList'),
  agentActionPanel: document.getElementById('agentActionPanel'),
  agentActionClose: document.getElementById('agentActionClose'),
  agentActionTitle: document.getElementById('agentActionTitle'),
  agentActionBody: document.getElementById('agentActionBody'),
  lockViewInput: document.getElementById('lockViewInput'),
  fontSettingsBtn: document.getElementById('fontSettingsBtn'),
  fontSettingsDialog: document.getElementById('fontSettingsDialog'),
  uiFontSizeInput: document.getElementById('uiFontSizeInput'),
  logFontSizeInput: document.getElementById('logFontSizeInput'),
  fontSettingsResetBtn: document.getElementById('fontSettingsResetBtn'),
  fontSettingsCloseBtn: document.getElementById('fontSettingsCloseBtn'),
  stepInspector: document.getElementById('stepInspector'),
  stepInspectorClose: document.getElementById('stepInspectorClose'),
  stepInspectorBody: document.getElementById('stepInspectorBody'),
  helpBtn:       document.getElementById('helpBtn'),
  helpDialog:    document.getElementById('helpDialog'),
  helpCloseBtn:  document.getElementById('helpCloseBtn'),
  openTutorialBtn: document.getElementById('openTutorialBtn'),
  onboardingDialog: document.getElementById('onboardingDialog'),
  onboardingSteps: document.getElementById('onboardingSteps'),
  onboardingStepCount: document.getElementById('onboardingStepCount'),
  onboardingStepTitle: document.getElementById('onboardingStepTitle'),
  onboardingStepBody: document.getElementById('onboardingStepBody'),
  tutorialSkipBtn: document.getElementById('tutorialSkipBtn'),
  tutorialBackBtn: document.getElementById('tutorialBackBtn'),
  tutorialNextBtn: document.getElementById('tutorialNextBtn'),
  notificationToast: document.getElementById('notificationToast'),
  notificationTitle: document.getElementById('notificationTitle'),
  notificationMessage: document.getElementById('notificationMessage'),
  notificationCloseBtn: document.getElementById('notificationCloseBtn'),
};

// Fixed cell size. Zoom multiplies this; effective cell = BASE_CELL * zoom.
const BASE_CELL = 30;
const MIN_ZOOM  = 0.3;
const MAX_ZOOM  = 4.0;

const state = {
  zoom: 1,
  pan:  { x: 0, y: 0 },
};

const panDrag = {
  active: false,
  pointerId: null,
  startX: 0,
  startY: 0,
  originX: 0,
  originY: 0,
};

init();

function init() {
  if (els.helpBtn && els.helpDialog && els.helpCloseBtn) {
    els.helpBtn.addEventListener('click', () => {
      els.helpDialog.hidden = false;
    });
    els.helpCloseBtn.addEventListener('click', closeHelpDialog);
    if (els.openTutorialBtn) {
      els.openTutorialBtn.addEventListener('click', () => {
        closeHelpDialog();
        showTutorial();
      });
    }
    els.helpDialog.addEventListener('click', e => {
      if (e.target === els.helpDialog) closeHelpDialog();
    });
    window.addEventListener('keydown', e => {
      if (e.key === 'Escape' && !els.helpDialog.hidden) closeHelpDialog();
      if (e.key === 'Escape' && els.onboardingDialog && !els.onboardingDialog.hidden) finishTutorial();
      if (e.key === 'Escape' && els.fontSettingsDialog && !els.fontSettingsDialog.hidden) hideFontSettingsDialog();
    });
  }

  initTutorial();
  initNotifications();
  initFontSettings();

  els.fileInput.addEventListener('change', e => {
    const file = e.target.files && e.target.files[0];
    if (file) loadFile(file);
  });
  ['dragenter', 'dragover'].forEach(type => els.dropZone.addEventListener(type, e => {
    e.preventDefault(); els.dropZone.classList.add('drag');
  }));
  ['dragleave', 'drop'].forEach(type => els.dropZone.addEventListener(type, e => {
    e.preventDefault(); els.dropZone.classList.remove('drag');
  }));
  els.dropZone.addEventListener('drop', e => {
    const file = e.dataTransfer.files && e.dataTransfer.files[0];
    if (file) loadFile(file);
  });
  els.playBtn.addEventListener('click', togglePlay);
  els.prevBtn.addEventListener('click', () => setStep(step - 1));
  els.nextBtn.addEventListener('click', () => setStep(step + 1));
  els.slider.addEventListener('input', e => setStep(Number(e.target.value)));
  els.stepInput.addEventListener('change', e => setStep(Number(e.target.value)));
  els.trackInput.addEventListener('input', render);
  els.coordGoBtn.addEventListener('click', () => {
    const coord = parseCoord(els.coordInput.value);
    if (coord) {
      highlightCoord = coord;
      centerOn(coord.r, coord.c);
    } else {
      highlightCoord = null;
    }
    render();
  });
  els.coordClearBtn.addEventListener('click', () => {
    highlightCoord = null;
    render();
  });
  els.coordInput.addEventListener('keydown', e => {
    if (e.key === 'Enter') els.coordGoBtn.click();
  });

  if (els.togglePanel) {
    els.togglePanel.addEventListener('click', () => {
      els.app.classList.toggle('collapsed');
      requestAnimationFrame(resetView);
    });
  }

  els.resetViewBtn.addEventListener('click', resetView);
  els.stepEventsBtn.addEventListener('click', toggleStepInspector);
  els.agentMenuBtn.addEventListener('click', toggleAgentMenu);
  els.agentMenuList.addEventListener('click', event => {
    if (!(event.target instanceof Element)) return;
    const button = event.target.closest('button[data-agent-id]');
    if (!button) return;
    const raw = button.dataset.agentId;
    selectAgentFocus(Number(raw));
  });
  els.agentActionClose.addEventListener('click', clearAgentFocus);
  els.lockViewInput.addEventListener('change', () => {
    setLockView(els.lockViewInput.checked);
  });
  els.fontSettingsBtn.addEventListener('click', showFontSettingsDialog);
  els.fontSettingsCloseBtn.addEventListener('click', hideFontSettingsDialog);
  els.fontSettingsResetBtn.addEventListener('click', resetFontSettings);
  els.fontSettingsDialog.addEventListener('click', event => {
    if (event.target === els.fontSettingsDialog) hideFontSettingsDialog();
  });
  els.stepInspectorClose.addEventListener('click', closeStepInspector);
  document.addEventListener('click', event => {
    if (!(event.target instanceof Element) || !event.target.closest('#agentMenuPanel, #agentMenuBtn')) {
      closeAgentMenu();
    }
  });
  els.boards.addEventListener('dblclick', resetView);
  els.boards.addEventListener('pointerdown', startPanDrag);
  els.boards.addEventListener('pointermove', updatePanDrag);
  els.boards.addEventListener('pointerup', endPanDrag);
  els.boards.addEventListener('pointercancel', endPanDrag);
  els.boards.addEventListener('lostpointercapture', endPanDrag);

  // Floating coord tooltip
  els.boards.addEventListener('mousemove', e => {
    if (panDrag.active) return;
    const cell = e.target.closest('.cell');
    if (!cell || !cell.dataset.r) {
      els.hoverTip.classList.remove('show');
      els.hoverTip.classList.remove('eventTip');
      return;
    }
    const r = cell.dataset.r;
    const c = cell.dataset.c;
    const rect = els.boards.getBoundingClientRect();
    const agentToken = e.target.closest('.agentToken');
    const agentIdText = agentToken?.dataset.agentId ?? cell.dataset.agentId;
    const agentId = agentIdText !== undefined ? Number(agentIdText) : null;
    if (Number.isInteger(agentId)) {
      els.hoverTip.classList.add('eventTip');
      els.hoverTip.innerHTML = renderAgentHoverTip(agentId, Number(r), Number(c));
    } else {
      els.hoverTip.classList.remove('eventTip');
      els.hoverTip.textContent = `(${r}, ${c})`;
    }
    els.hoverTip.style.left = `${e.clientX - rect.left + 14}px`;
    els.hoverTip.style.top  = `${e.clientY - rect.top  + 14}px`;
    els.hoverTip.classList.add('show');
    els.hoverInfo.textContent = `(${r}, ${c})`;
  });
  els.boards.addEventListener('mouseleave', () => {
    els.hoverTip.classList.remove('show');
    els.hoverTip.classList.remove('eventTip');
  });

  // Wheel = zoom around cursor
  els.boards.addEventListener('wheel', e => {
    if (!replay) return;
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.1 : 1 / 1.1;
    zoomAt(e.clientX, e.clientY, factor);
  }, { passive: false });

  window.addEventListener('resize', () => {
    if (state.zoom === 1) resetView();
  });

  applyZoom();
  applyPan();

  restoreInitialReplay();

  if (localStorageAvailable() && localStorage.getItem(TUTORIAL_STORAGE_KEY) !== '1') {
    showTutorial();
  }
}

function closeHelpDialog() {
  els.helpDialog.hidden = true;
}

function initTutorial() {
  if (!els.onboardingDialog || !els.onboardingSteps) return;
  els.onboardingDialog.addEventListener('click', e => {
    if (e.target === els.onboardingDialog) finishTutorial();
  });
  els.tutorialSkipBtn.addEventListener('click', finishTutorial);
  els.tutorialBackBtn.addEventListener('click', previousTutorialStep);
  els.tutorialNextBtn.addEventListener('click', nextTutorialStep);
  renderTutorial();
}

function showTutorial() {
  if (!els.onboardingDialog) return;
  tutorialStepIndex = 0;
  renderTutorial();
  els.onboardingDialog.hidden = false;
}

function finishTutorial() {
  if (localStorageAvailable()) {
    localStorage.setItem(TUTORIAL_STORAGE_KEY, '1');
  }
  els.onboardingDialog.hidden = true;
}

function previousTutorialStep() {
  tutorialStepIndex = Math.max(0, tutorialStepIndex - 1);
  renderTutorial();
}

function nextTutorialStep() {
  if (tutorialStepIndex >= tutorialSteps.length - 1) {
    finishTutorial();
    return;
  }
  tutorialStepIndex++;
  renderTutorial();
}

function renderTutorial() {
  if (!els.onboardingSteps) return;
  els.onboardingSteps.innerHTML = '';
  for (let i = 0; i < tutorialSteps.length; i++) {
    const item = document.createElement('li');
    if (i === tutorialStepIndex) item.classList.add('active');
    if (i < tutorialStepIndex) item.classList.add('done');
    const number = document.createElement('span');
    number.textContent = String(i + 1);
    item.appendChild(number);
    item.appendChild(document.createTextNode(tutorialSteps[i].title));
    els.onboardingSteps.appendChild(item);
  }
  const current = tutorialSteps[tutorialStepIndex];
  els.onboardingStepCount.textContent = `Step ${tutorialStepIndex + 1} of ${tutorialSteps.length}`;
  els.onboardingStepTitle.textContent = current.title;
  els.onboardingStepBody.textContent = current.body;
  els.tutorialBackBtn.disabled = tutorialStepIndex === 0;
  els.tutorialNextBtn.textContent = tutorialStepIndex === tutorialSteps.length - 1 ? 'Finish' : 'Next';
}

function initNotifications() {
  if (!els.notificationToast || !els.notificationCloseBtn) return;
  els.notificationCloseBtn.addEventListener('click', clearNotification);
}

function initFontSettings() {
  const savedUi = localStorageAvailable() ? Number(localStorage.getItem(FONT_SIZE_STORAGE_KEY)) : NaN;
  const savedLog = localStorageAvailable() ? Number(localStorage.getItem(LOG_FONT_SIZE_STORAGE_KEY)) : NaN;
  const uiSize = clampUIFontSize(Number.isFinite(savedUi) ? savedUi : DEFAULT_UI_FONT_SIZE);
  const logSize = clampLogFontSize(Number.isFinite(savedLog) ? savedLog : DEFAULT_LOG_FONT_SIZE);
  applyUIFontSize(uiSize);
  applyLogFontSize(logSize);
  els.uiFontSizeInput.value = String(uiSize);
  els.logFontSizeInput.value = String(logSize);
  els.uiFontSizeInput.addEventListener('input', () => {
    const next = clampUIFontSize(Number(els.uiFontSizeInput.value));
    applyUIFontSize(next);
  });
  els.uiFontSizeInput.addEventListener('change', () => {
    const next = clampUIFontSize(Number(els.uiFontSizeInput.value));
    els.uiFontSizeInput.value = String(next);
    applyUIFontSize(next);
  });
  els.logFontSizeInput.addEventListener('input', () => {
    const next = clampLogFontSize(Number(els.logFontSizeInput.value));
    applyLogFontSize(next);
  });
  els.logFontSizeInput.addEventListener('change', () => {
    const next = clampLogFontSize(Number(els.logFontSizeInput.value));
    els.logFontSizeInput.value = String(next);
    applyLogFontSize(next);
  });
}

function showFontSettingsDialog() {
  closeAgentMenu();
  els.fontSettingsDialog.hidden = false;
  els.uiFontSizeInput.focus();
  els.uiFontSizeInput.select();
}

function hideFontSettingsDialog() {
  els.fontSettingsDialog.hidden = true;
}

function resetFontSettings() {
  els.uiFontSizeInput.value = String(DEFAULT_UI_FONT_SIZE);
  els.logFontSizeInput.value = String(DEFAULT_LOG_FONT_SIZE);
  applyUIFontSize(DEFAULT_UI_FONT_SIZE);
  applyLogFontSize(DEFAULT_LOG_FONT_SIZE);
}

function clampUIFontSize(value) {
  if (!Number.isFinite(value)) return DEFAULT_UI_FONT_SIZE;
  return Math.max(MIN_UI_FONT_SIZE, Math.min(MAX_UI_FONT_SIZE, Math.round(value)));
}

function clampLogFontSize(value) {
  if (!Number.isFinite(value)) return DEFAULT_LOG_FONT_SIZE;
  return Math.max(MIN_LOG_FONT_SIZE, Math.min(MAX_LOG_FONT_SIZE, Math.round(value)));
}

function applyUIFontSize(size) {
  const next = clampUIFontSize(size);
  document.documentElement.style.setProperty('--ui-font-size', `${next}px`);
  els.fontSettingsBtn.textContent = `${next}px`;
  resetAgentPanelHeight();
  updateAgentActionPanelLayout();
  if (localStorageAvailable()) {
    localStorage.setItem(FONT_SIZE_STORAGE_KEY, String(next));
  }
}

function applyLogFontSize(size) {
  const next = clampLogFontSize(size);
  document.documentElement.style.setProperty('--log-font-size', `${next}px`);
  resetAgentPanelHeight();
  updateAgentActionPanelLayout();
  if (localStorageAvailable()) {
    localStorage.setItem(LOG_FONT_SIZE_STORAGE_KEY, String(next));
  }
}

function showError(message, title = 'Something needs attention') {
  if (!els.notificationToast) return;
  els.notificationTitle.textContent = title;
  els.notificationMessage.textContent = message;
  els.notificationToast.hidden = false;
  if (notificationTimer) window.clearTimeout(notificationTimer);
  notificationTimer = window.setTimeout(clearNotification, 9000);
}

function clearNotification() {
  if (!els.notificationToast) return;
  els.notificationToast.hidden = true;
  els.notificationTitle.textContent = 'Something needs attention';
  els.notificationMessage.textContent = '';
  if (notificationTimer) window.clearTimeout(notificationTimer);
  notificationTimer = null;
}

function localStorageAvailable() {
  try {
    const key = 'aims-replay-viewer:storage-test';
    localStorage.setItem(key, '1');
    localStorage.removeItem(key);
    return true;
  } catch (_) {
    return false;
  }
}

async function restoreInitialReplay() {
  try {
    const saved = await readLastOpenedReplay();
    if (saved?.text) {
      const data = JSON.parse(saved.text);
      loadReplayData(data, { validated: false });
      return;
    }
  } catch (error) {
    console.warn('Unable to restore last opened replay:', error);
  }

  if (window.DEFAULT_REPLAY) {
    try {
      loadReplayData(window.DEFAULT_REPLAY);
    } catch (error) {
      showError(errorMessage(error), 'Unable to load replay');
      render();
    }
  } else {
    render();
  }
}

async function saveLastOpenedReplay(text, name = '') {
  const record = {
    text,
    name,
    savedAt: new Date().toISOString(),
  };
  try {
    await writeReplayStorage(record);
  } catch (error) {
    if (!localStorageAvailable()) throw error;
    localStorage.setItem(LAST_REPLAY_LOCAL_STORAGE_KEY, JSON.stringify(record));
  }
}

async function readLastOpenedReplay() {
  try {
    const record = await readReplayStorage();
    if (record?.text) return record;
  } catch (error) {
    console.warn('IndexedDB replay restore unavailable:', error);
  }
  if (!localStorageAvailable()) return null;
  const raw = localStorage.getItem(LAST_REPLAY_LOCAL_STORAGE_KEY);
  return raw ? JSON.parse(raw) : null;
}

function openReplayStorage() {
  return new Promise((resolve, reject) => {
    if (!('indexedDB' in window)) {
      reject(new Error('IndexedDB is not available.'));
      return;
    }
    const request = indexedDB.open(LAST_REPLAY_DB_NAME, LAST_REPLAY_DB_VERSION);
    request.onupgradeneeded = () => {
      const db = request.result;
      if (!db.objectStoreNames.contains(LAST_REPLAY_STORE_NAME)) {
        db.createObjectStore(LAST_REPLAY_STORE_NAME);
      }
    };
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(request.error || new Error('Unable to open replay storage.'));
  });
}

async function writeReplayStorage(record) {
  const db = await openReplayStorage();
  return new Promise((resolve, reject) => {
    const tx = db.transaction(LAST_REPLAY_STORE_NAME, 'readwrite');
    tx.objectStore(LAST_REPLAY_STORE_NAME).put(record, LAST_REPLAY_KEY);
    tx.oncomplete = () => {
      db.close();
      resolve();
    };
    tx.onerror = () => {
      db.close();
      reject(tx.error || new Error('Unable to save replay storage.'));
    };
  });
}

async function readReplayStorage() {
  const db = await openReplayStorage();
  return new Promise((resolve, reject) => {
    const tx = db.transaction(LAST_REPLAY_STORE_NAME, 'readonly');
    const request = tx.objectStore(LAST_REPLAY_STORE_NAME).get(LAST_REPLAY_KEY);
    request.onsuccess = () => resolve(request.result || null);
    request.onerror = () => reject(request.error || new Error('Unable to read replay storage.'));
    tx.oncomplete = () => db.close();
    tx.onerror = () => {
      db.close();
      reject(tx.error || new Error('Unable to read replay storage.'));
    };
  });
}

// ---- Zoom / Pan -----------------------------------------------------------

function applyZoom() {
  const cell = Math.round(BASE_CELL * state.zoom);
  document.documentElement.style.setProperty('--cell', `${cell}px`);
  document.documentElement.style.setProperty('--axis', `${cell}px`);
  els.zoomInfo.textContent = `${Math.round(state.zoom * 100)}%`;
}

function applyPan() {
  const tx = `${state.pan.x}px`;
  const ty = `${state.pan.y}px`;
  els.panSurface.style.transform   = `translate(${tx}, ${ty})`;
  els.axisTopInner.style.transform  = `translateX(${tx})`;
  els.axisLeftInner.style.transform = `translateY(${ty})`;
}

function startPanDrag(e) {
  if (!replay || els.boards.classList.contains('empty')) return;
  if (e.button !== 0) return;
  panDrag.active = true;
  panDrag.pointerId = e.pointerId;
  panDrag.startX = e.clientX;
  panDrag.startY = e.clientY;
  panDrag.originX = state.pan.x;
  panDrag.originY = state.pan.y;
  els.boards.classList.add('panning');
  els.hoverTip.classList.remove('show');
  if (els.boards.setPointerCapture) {
    try {
      els.boards.setPointerCapture(e.pointerId);
    } catch (_) {
      // Dragging still works without pointer capture; capture only keeps it smooth off-board.
    }
  }
  e.preventDefault();
}

function updatePanDrag(e) {
  if (!panDrag.active || e.pointerId !== panDrag.pointerId) return;
  state.pan.x = Math.round(panDrag.originX + e.clientX - panDrag.startX);
  state.pan.y = Math.round(panDrag.originY + e.clientY - panDrag.startY);
  applyPan();
  e.preventDefault();
}

function endPanDrag(e) {
  if (!panDrag.active) return;
  if (e.pointerId !== undefined && panDrag.pointerId !== null && e.pointerId !== panDrag.pointerId) return;
  panDrag.active = false;
  panDrag.pointerId = null;
  els.boards.classList.remove('panning');
  if (e.pointerId !== undefined && els.boards.releasePointerCapture && els.boards.hasPointerCapture?.(e.pointerId)) {
    els.boards.releasePointerCapture(e.pointerId);
  }
}

function centerOn(r, c, resetZoom = true) {
  if (!replay) return;
  if (resetZoom) {
    state.zoom = 1;
    applyZoom();
  }
  requestAnimationFrame(() => {
    const cellSize = Math.round(BASE_CELL * state.zoom);
    const cellCenterX = 1 + c * (cellSize + 1) + cellSize / 2;
    const cellCenterY = 1 + r * (cellSize + 1) + cellSize / 2;
    state.pan.x = Math.round(els.panViewport.clientWidth / 2 - cellCenterX);
    state.pan.y = Math.round(els.panViewport.clientHeight / 2 - cellCenterY);
    applyPan();
  });
}

function followFocusedAgent() {
  if (!lockViewEnabled || !replay || focusedAgentId == null) return;
  const frame = replay.frames[step];
  const agent = frame?.agents?.find(item => item.id === focusedAgentId);
  if (!agent) return;
  centerOn(agent.r, agent.c, false);
}

function resetView() {
  state.zoom = 1;
  applyZoom();
  if (!replay) { state.pan.x = state.pan.y = 0; applyPan(); return; }
  requestAnimationFrame(() => {
    const card = els.panSurface.firstElementChild;
    if (!card) { state.pan.x = state.pan.y = 0; applyPan(); return; }
    const boardW = card.offsetWidth;
    const boardH = card.offsetHeight;
    state.pan.x = Math.round((els.panViewport.clientWidth  - boardW) / 2);
    state.pan.y = Math.round((els.panViewport.clientHeight - boardH) / 2);
    applyPan();
  });
}

function zoomAt(clientX, clientY, factor) {
  const newZoom = clamp(state.zoom * factor, MIN_ZOOM, MAX_ZOOM);
  const realFactor = newZoom / state.zoom;
  if (realFactor === 1) return;

  const rect = els.panViewport.getBoundingClientRect();
  const mx = clientX - rect.left;
  const my = clientY - rect.top;
  state.pan.x = mx - (mx - state.pan.x) * realFactor;
  state.pan.y = my - (my - state.pan.y) * realFactor;
  state.zoom = newZoom;

  applyZoom();
  applyPan();
}

function rebuildAxes(rows, cols) {
  els.axisTopInner.innerHTML = '';
  els.axisLeftInner.innerHTML = '';
  els.axisTopInner.style.gridTemplateColumns = `repeat(${cols}, var(--axis))`;
  els.axisTopInner.style.gridTemplateRows    = `var(--axis)`;
  els.axisLeftInner.style.gridTemplateColumns = `var(--axis)`;
  els.axisLeftInner.style.gridTemplateRows    = `repeat(${rows}, var(--axis))`;
  for (let c = 0; c < cols; c++) els.axisTopInner.appendChild(div('axisCell', String(c)));
  for (let r = 0; r < rows; r++) els.axisLeftInner.appendChild(div('axisCell', String(r)));
}

// ---- Data loading ----------------------------------------------------------

async function loadFile(file) {
  try {
    const text = await file.text();
    const data = JSON.parse(text);
    validateReplay(data);
    if (timer && !window.confirm('Playback is running. Load a new replay and stop the current playback?')) {
      return;
    }
    loadReplayData(data, { validated: true });
    saveLastOpenedReplay(text, file.name).catch(error => {
      console.warn('Unable to remember last opened replay:', error);
    });
  } catch (error) {
    showError(errorMessage(error), 'Unable to load replay');
  } finally {
    els.fileInput.value = '';
  }
}

function loadReplayData(data, options = {}) {
  if (!options.validated) validateReplay(data);
  stopPlay();
  replay = data;
  indexReplayEvents();
  step = 0;
  els.slider.max = replay.frames.length - 1;
  els.stepInput.max = replay.frames.length - 1;
  const s = replay.summary || {};
  els.meta.innerHTML = `<b>${escapeHtml(replay.level.name)}</b><br>${replay.level.rows}x${replay.level.cols}, ${replay.frames.length} frames<br>${escapeHtml(s.outcome || 'unknown')} | ${s.executedSteps ?? 0} steps | ${s.satisfiedBoxGoals ?? 0}/${s.totalBoxGoals ?? 0} box goals<br>${escapeHtml(replay.generatedAt || '')}`;
  populateAgentFocusMenu();
  buildBoard();
  render();
  if (els.autoPlayInput.checked) startPlay();
}

function validateReplay(data) {
  if (!data || typeof data !== 'object') {
    throw new Error('The selected file is not a replay object.');
  }
  if (!data.level || typeof data.level !== 'object') {
    throw new Error('Missing required field: level.');
  }
  if (!Array.isArray(data.frames)) {
    throw new Error('Missing required field: frames.');
  }
  if (data.frames.length === 0) {
    throw new Error('Replay has no frames.');
  }
  const level = data.level;
  for (const field of ['name', 'rows', 'cols', 'walls', 'boxGoals', 'agentGoals', 'agentColors', 'boxColors']) {
    if (!(field in level)) throw new Error(`Missing required field: level.${field}.`);
  }
  if (!Array.isArray(level.walls) || level.walls.length !== level.rows) {
    throw new Error('level.walls must contain one row string per level row.');
  }
  if (!Array.isArray(level.boxGoals) || !Array.isArray(level.agentGoals)) {
    throw new Error('level.boxGoals and level.agentGoals must be arrays.');
  }
  for (let i = 0; i < data.frames.length; i++) {
    const frame = data.frames[i];
    if (!frame || typeof frame !== 'object') throw new Error(`Frame ${i} is not an object.`);
    if (!Array.isArray(frame.agents)) throw new Error(`Frame ${i} is missing agents array.`);
    if (!Array.isArray(frame.boxes)) throw new Error(`Frame ${i} is missing boxes array.`);
    if (!Array.isArray(frame.actions)) throw new Error(`Frame ${i} is missing actions array.`);
    if (!Array.isArray(frame.accepted)) throw new Error(`Frame ${i} is missing accepted array.`);
  }
}

// ---- Board construction (axes + panSurface) --------------------------------

function buildBoard() {
  const { rows, cols } = replay.level;
  rebuildAxes(rows, cols);

  els.boards.classList.remove('empty');
  for (const e of els.boardWrap.querySelectorAll(':scope > .empty')) e.remove();
}

// ---- Render current frame --------------------------------------------------

function render() {
  if (!replay) {
    els.panSurface.innerHTML = '';
    els.boards.classList.add('empty');
    for (const e of els.boardWrap.querySelectorAll(':scope > .empty')) e.remove();
    const empty = document.createElement('div');
    empty.className = 'empty';
    empty.textContent = 'Drop a replay JSON file to begin.';
    els.boardWrap.appendChild(empty);
    els.statusInfo.textContent = '';
    els.agentMenuBtn.hidden = true;
    closeAgentMenu();
    els.agentActionPanel.hidden = true;
    els.boardWrap.classList.remove('agentPanelOpen', 'agentMenuOpen');
    return;
  }
  els.boards.classList.remove('empty');
  for (const e of els.boardWrap.querySelectorAll(':scope > .empty')) e.remove();
  step = Math.max(0, Math.min(step, replay.frames.length - 1));
  const frame = replay.frames[step];
  const prev = replay.frames[Math.max(0, step - 1)];
  els.slider.value = step;
  els.stepInput.value = step;
  els.stepText.textContent = `/ ${replay.frames.length - 1}`;

  const { rows, cols } = replay.level;
  const goals = goalMap();
  const agents = new Map(frame.agents.map(a => [`${a.r},${a.c}`, a]));
  const boxes = new Map(frame.boxes.map(b => [`${b.r},${b.c}`, b]));
  const changed = changedCells(prev, frame);
  const track = trackedCells();
  const trail = trailCells();
  const coord = highlightCoord;

  // Rebuild the board DOM on panSurface
  const card = document.createElement('div');
  card.className = 'boardCard';

  const board = document.createElement('div');
  board.className = 'board';
  board.style.gridTemplateColumns = `repeat(${cols}, var(--cell))`;

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const key = `${r},${c}`;
      const cell = document.createElement('div');
      cell.className = 'cell';
      cell.dataset.r = r;
      cell.dataset.c = c;

      if (replay.level.walls[r][c] === '+') cell.classList.add('wall');

      // Goal background - match level-viewer's goal-fill / goal-solved
      if (goals.has(key)) {
        const goalType = goals.get(key);
        const satisfied = isGoalSatisfiedInFrame(frame, r, c, goalType);
        cell.classList.add(satisfied ? 'goal-solved' : 'goal-fill');
        const gc = document.createElement('span');
        gc.className = 'goalChar';
        gc.textContent = goalType;
        cell.appendChild(gc);
      }

      // Highlights (order matters - later ones paint on top)
      if (trail.has(key)) cell.classList.add('trail');
      if (changed.has(key)) cell.classList.add('changed');
      if (track.has(key)) cell.classList.add('track');
      if (coord && coord.r === r && coord.c === c) cell.classList.add('coord');

      // Tokens
      if (boxes.has(key)) cell.appendChild(token(boxes.get(key).type, boxColor(boxes.get(key).type)));
      if (agents.has(key)) {
        const agent = agents.get(key);
        const agentEl = token(String(agent.id), agentColor(agent.id), 'agentToken');
        agentEl.dataset.agentId = String(agent.id);
        cell.dataset.agentId = String(agent.id);
        cell.appendChild(agentEl);
      }

      board.appendChild(cell);
    }
  }

  card.appendChild(board);
  els.panSurface.innerHTML = '';
  els.panSurface.appendChild(card);

  els.statusInfo.textContent = '';

  renderActions(frame);
  renderStepInspector(frame);
  renderAgentActionPanel();
  followFocusedAgent();
}

function isGoalSatisfiedInFrame(frame, r, c, goalType) {
  // Box goal: a box of the correct type is at (r,c) in this frame
  for (const b of frame.boxes) {
    if (b.r === r && b.c === c && b.type === goalType) return true;
  }
  // Agent goal: an agent with matching id is at (r,c) in this frame
  for (const a of frame.agents) {
    if (a.r === r && a.c === c && String(a.id) === goalType) return true;
  }
  return false;
}

// ---- Actions panel ---------------------------------------------------------

function renderActions(frame) {
  els.actions.innerHTML = '';
  const actions = frame.actions || [];
  const accepted = frame.accepted || [];
  for (let i = 0; i < actions.length; i++) {
    const row = document.createElement('div');
    row.className = `actionRow ${accepted[i] ? '' : 'rejected'}`;
    const left = document.createElement('span');
    left.textContent = i;
    const right = document.createElement('span');
    right.className = 'act';
    right.textContent = actions[i];
    row.appendChild(left);
    row.appendChild(right);
    els.actions.appendChild(row);
  }
}

function populateAgentFocusMenu() {
  const ids = agentIds();
  const previous = focusedAgentId;
  els.agentMenuList.innerHTML = '';
  focusedAgentId = ids.includes(previous) ? previous : null;

  for (const id of ids) {
    els.agentMenuList.appendChild(agentMenuItem(String(id), `agent${id}`, 'Pin, center, and follow this agent', focusedAgentId === id));
  }
  els.agentMenuBtn.hidden = ids.length === 0;
  closeAgentMenu();
  updateAgentMenuButton();
}

function agentMenuItem(value, title, body, active) {
  const item = document.createElement('li');
  if (active) item.classList.add('active');
  const button = document.createElement('button');
  button.type = 'button';
  button.dataset.agentId = value;
  button.setAttribute('aria-pressed', active ? 'true' : 'false');

  const badge = document.createElement('span');
  badge.className = 'agentMenuBadge';
  badge.textContent = value;
  const copy = document.createElement('span');
  copy.className = 'agentMenuCopy';
  const strong = document.createElement('strong');
  strong.textContent = title;
  const small = document.createElement('small');
  small.textContent = body;
  copy.appendChild(strong);
  copy.appendChild(small);
  button.appendChild(badge);
  button.appendChild(copy);
  item.appendChild(button);
  return item;
}

function updateAgentMenuButton() {
  els.agentMenuBtn.textContent = focusedAgentId == null ? 'Agent' : `agent${focusedAgentId}`;
  els.agentMenuBtn.classList.toggle('active', focusedAgentId != null);
}

function toggleAgentMenu() {
  if (els.agentMenuBtn.hidden) return;
  const shouldOpen = els.agentMenuPanel.hidden;
  els.agentMenuPanel.hidden = !shouldOpen;
  els.boardWrap.classList.toggle('agentMenuOpen', shouldOpen);
  els.agentMenuBtn.setAttribute('aria-expanded', shouldOpen ? 'true' : 'false');
  if (shouldOpen) {
    hideFontSettingsDialog();
  }
}

function closeAgentMenu() {
  els.agentMenuPanel.hidden = true;
  els.boardWrap.classList.remove('agentMenuOpen');
  els.agentMenuBtn.setAttribute('aria-expanded', 'false');
}

function selectAgentFocus(agentId) {
  const nextAgentId = Number.isInteger(agentId) ? agentId : null;
  const agentChanged = focusedAgentId !== nextAgentId;
  focusedAgentId = nextAgentId;
  if (focusedAgentId != null) {
    setLockView(true, false);
  }
  if (agentChanged) {
    resetAgentPanelHeight();
  }
  for (const item of els.agentMenuList.querySelectorAll('li')) {
    const button = item.querySelector('button[data-agent-id]');
    const raw = button?.dataset.agentId;
    const active = focusedAgentId != null && raw === String(focusedAgentId);
    item.classList.toggle('active', active);
    button?.setAttribute('aria-pressed', active ? 'true' : 'false');
  }
  updateAgentMenuButton();
  closeAgentMenu();
  renderAgentActionPanel();
  followFocusedAgent();
}

function setLockView(enabled, followNow = true) {
  lockViewEnabled = Boolean(enabled);
  els.lockViewInput.checked = lockViewEnabled;
  if (lockViewEnabled && followNow) followFocusedAgent();
}

function agentIds() {
  if (!replay || !Array.isArray(replay.frames) || replay.frames.length === 0) return [];
  const ids = new Set();
  for (const frame of replay.frames) {
    for (const agent of frame.agents || []) {
      if (Number.isInteger(agent.id)) ids.add(agent.id);
    }
    if (ids.size > 0) break;
  }
  return [...ids].sort((a, b) => a - b);
}

function clearAgentFocus() {
  selectAgentFocus(null);
}

function renderAgentActionPanel() {
  if (!replay || focusedAgentId == null) {
    els.agentActionPanel.hidden = true;
    els.boardWrap.classList.remove('agentPanelOpen');
    clearAgentPanelStackOffset();
    return;
  }
  const frame = replay.frames[step];
  if (!frame) {
    els.agentActionPanel.hidden = true;
    els.boardWrap.classList.remove('agentPanelOpen');
    clearAgentPanelStackOffset();
    return;
  }

  const events = agentStepEvents(focusedAgentId);
  const intent = events.find(isAgentIntentEvent);
  const actionEvent = events.find(isAgentActionEvent);
  const actionText = frame.actions && frame.actions[focusedAgentId]
    ? frame.actions[focusedAgentId]
    : actionEvent?.action || 'NoOp';
  const accepted = frame.accepted && focusedAgentId < frame.accepted.length
    ? frame.accepted[focusedAgentId]
    : actionEvent?.accepted !== false;
  const pos = (frame.agents || []).find(agent => agent.id === focusedAgentId);

  els.agentActionTitle.textContent = `agent${focusedAgentId}`;
  els.agentActionBody.innerHTML = '';
  const fields = document.createElement('dl');
  fields.className = 'agentActionGrid';
  appendAgentMonitorField(fields, 'Step', String(step));
  appendAgentMonitorField(fields, 'Action', actionText, accepted ? '' : 'rejected');
  appendAgentMonitorField(fields, 'Result', accepted ? 'accepted' : 'rejected', accepted ? '' : 'rejected');
  appendAgentMonitorField(fields, 'Position', pos ? `(${pos.r}, ${pos.c})` : '');
  if (intent) {
    appendAgentMonitorField(fields, 'Phase', intent.phase);
    appendAgentMonitorField(fields, 'Subgoal', intent.subgoal);
    appendAgentMonitorField(fields, 'Intent', intent.reason || intent.message);
    appendAgentMonitorField(fields, 'Planned', intent.action);
    appendAgentMonitorField(fields, 'Server', intent.actualAction);
  }
  if (actionEvent) {
    appendAgentMonitorField(fields, 'Move', movementText(actionEvent));
    appendAgentMonitorField(fields, 'Box', boxMovementText(actionEvent));
  }
  els.agentActionBody.appendChild(fields);
  els.agentActionPanel.hidden = false;
  els.boardWrap.classList.add('agentPanelOpen');
  requestAnimationFrame(updateAgentActionPanelLayout);
}

function updateAgentActionPanelLayout() {
  if (!els.agentActionPanel || els.agentActionPanel.hidden) return;
  applyAgentPanelHeight();
  const overflow = els.agentActionBody.scrollHeight - els.agentActionBody.clientHeight;
  if (overflow > 1) {
    const nextHeight = Math.min(maxAgentPanelHeight(), Math.ceil(agentPanelHeight + overflow));
    if (nextHeight > agentPanelHeight) {
      agentPanelHeight = nextHeight;
      applyAgentPanelHeight();
    }
  }
  updateAgentPanelStackOffset();
}

function resetAgentPanelHeight() {
  agentPanelHeight = DEFAULT_AGENT_PANEL_HEIGHT;
  applyAgentPanelHeight();
}

function applyAgentPanelHeight() {
  if (!els.agentActionPanel) return;
  const height = Math.min(agentPanelHeight, maxAgentPanelHeight());
  els.agentActionPanel.style.setProperty('--agent-panel-height', `${height}px`);
}

function maxAgentPanelHeight() {
  const available = els.boardWrap ? els.boardWrap.clientHeight - 74 : MAX_AGENT_PANEL_HEIGHT;
  return Math.max(180, Math.min(MAX_AGENT_PANEL_HEIGHT, available));
}

function updateAgentPanelStackOffset() {
  if (!els.agentActionPanel || els.agentActionPanel.hidden) {
    clearAgentPanelStackOffset();
    return;
  }
  const panelHeight = Math.ceil(els.agentActionPanel.offsetHeight);
  els.boardWrap.style.setProperty('--agent-panel-stack-bottom', `${panelHeight + 54}px`);
  els.boardWrap.style.setProperty('--agent-panel-stack-clearance', `${panelHeight + 82}px`);
}

function clearAgentPanelStackOffset() {
  els.boardWrap.style.removeProperty('--agent-panel-stack-bottom');
  els.boardWrap.style.removeProperty('--agent-panel-stack-clearance');
  if (els.agentActionPanel) {
    els.agentActionPanel.style.removeProperty('--agent-panel-height');
  }
}

function appendAgentMonitorField(parent, label, value, valueClass = '') {
  const formatted = formatEventValue(value);
  if (!formatted) return;
  const dt = document.createElement('dt');
  dt.textContent = label;
  const dd = document.createElement('dd');
  dd.textContent = formatted;
  if (valueClass) dd.className = valueClass;
  parent.appendChild(dt);
  parent.appendChild(dd);
}

function movementText(event) {
  const from = formatEventValue(event?.from);
  const to = formatEventValue(event?.to);
  if (!from && !to) return '';
  return `${from || '?'} -> ${to || '?'}`;
}

function boxMovementText(event) {
  if (!event || !event.boxType) return '';
  const from = formatEventValue(event.boxFrom);
  const to = formatEventValue(event.boxTo);
  return `${event.boxType} ${from || '?'} -> ${to || '?'}`;
}

function toggleStepInspector() {
  if (!replay) return;
  els.stepInspector.hidden = !els.stepInspector.hidden;
  if (!els.stepInspector.hidden) {
    renderStepInspector(replay.frames[step]);
  }
}

function closeStepInspector() {
  els.stepInspector.hidden = true;
}

function renderStepInspector(frame) {
  if (!replay || !frame) {
    els.stepEventsBtn.hidden = true;
    closeStepInspector();
    return;
  }
  const events = stepEvents(frame, step);
  const highlights = stepHighlights(frame, step);
  const totalEvents = replayEventCount();
  els.stepEventsBtn.hidden = false;
  els.stepEventsBtn.textContent = `Highlights ${highlights.length}`;
  els.stepEventsBtn.classList.toggle('muted', highlights.length === 0);

  if (els.stepInspector.hidden) return;

  els.stepInspectorBody.innerHTML = '';
  const summary = document.createElement('div');
  summary.className = 'stepInspectorSummary';
  summary.textContent = `Step ${step} | ${highlights.length} highlight${highlights.length === 1 ? '' : 's'} here | ${totalEvents} raw events in replay`;
  els.stepInspectorBody.appendChild(summary);

  if (highlights.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'stepInspectorEmpty';
    empty.textContent = totalEvents === 0
      ? 'This replay does not include advanced step events.'
      : 'No high-signal event on this step. Hover an agent for its current intent and action.';
    els.stepInspectorBody.appendChild(empty);
    return;
  }

  for (const event of highlights) {
    els.stepInspectorBody.appendChild(renderEventCard(event));
  }
}

function stepEvents(frame, stepIndex) {
  const events = [];
  collectEvents(events, frame.events);
  collectEvents(events, frame.diagnostics);
  collectEvents(events, frame.debugEvents);
  collectEvents(events, frame.metadata?.events);
  collectEvents(events, indexedReplayEvents.get(stepIndex));
  return events;
}

function stepHighlights(frame, stepIndex) {
  return stepEvents(frame, stepIndex).filter(isStepHighlight);
}

function isStepHighlight(event) {
  if (!event || typeof event !== 'object') return true;
  const kind = String(event.kind || event.type || '').toLowerCase();
  if (kind === 'agent-action' || kind === 'agent-intent') return false;
  if (kind === 'rejected-action') return true;
  if (event.accepted === false) return true;
  const severity = String(event.severity || event.level || '').toLowerCase();
  if (severity.includes('warn') || severity.includes('error') || severity.includes('fail')) return true;
  return kind.includes('subgoal') || kind.includes('rollback') || kind.includes('regress')
      || kind.includes('conflict') || kind.includes('block') || kind.includes('seal');
}

function agentStepEvents(agentId) {
  if (!replay) return [];
  const frame = replay.frames[step];
  if (!frame) return [];
  return stepEvents(frame, step).filter(event => eventAgentId(event) === agentId);
}

function eventAgentId(event) {
  if (!event || typeof event !== 'object') return null;
  const raw = event.agentId ?? event.agent ?? event.agent_id;
  if (Number.isInteger(raw)) return raw;
  if (/^\d+$/.test(String(raw ?? ''))) return Number(raw);
  const match = String(raw ?? '').match(/^agent\s*(\d+)$/i);
  return match ? Number(match[1]) : null;
}

function renderAgentHoverTip(agentId, r, c) {
  const events = agentStepEvents(agentId);
  const intent = events.find(isAgentIntentEvent);
  const action = events.find(isAgentActionEvent);
  const html = [
    `<div class="hoverTipTitle"><strong>agent${agentId}</strong><span>(${r}, ${c})</span></div>`
  ];
  if (!intent && !action) {
    html.push('<div class="hoverTipMuted">No intent event on this step</div>');
    return html.join('');
  }

  if (intent) {
    html.push(renderAgentIntentHoverEvent(intent));
  }
  if (action) {
    html.push(renderAgentActionHoverEvent(action));
  }
  return html.join('');
}

function isAgentIntentEvent(event) {
  return event && typeof event === 'object'
      && String(event.kind || event.type || '').toLowerCase() === 'agent-intent';
}

function isAgentActionEvent(event) {
  if (!event || typeof event !== 'object') return false;
  const kind = String(event.kind || event.type || '').toLowerCase();
  return kind === 'agent-action' || kind === 'rejected-action';
}

function renderAgentIntentHoverEvent(event) {
  const phase = event.phase || 'unknown';
  const subgoal = event.subgoal || event.title || 'unknown';
  const progress = event.stepInSegment && event.segmentSteps
    ? `${event.stepInSegment}/${event.segmentSteps}`
    : '';
  const rows = [
    '<div class="hoverTipEvent success">',
    `<div class="hoverTipSection"><strong>Planner intent</strong><span>${escapeHtml(progress || phase)}</span></div>`,
    hoverField('Phase', phase),
    hoverField('Subgoal', subgoal),
    hoverField('Subgoal type', event.subgoalType),
    hoverField('Goal', event.goal),
    hoverField('Box type', event.boxType),
    hoverField('Progress', progress),
    hoverField('Planned action', event.action),
    hoverField('Server action', event.actualAction),
    hoverField('Reason', event.reason),
    hoverField('Message', event.message)
  ].filter(Boolean);
  rows.push('</div>');
  return rows.join('');
}

function renderAgentActionHoverEvent(event) {
  const normalized = normalizeEvent(event);
  const accepted = event && typeof event === 'object' && event.accepted === false ? 'rejected' : 'accepted';
  const action = event && typeof event === 'object' && event.action ? event.action : normalized.title;
  const from = event && typeof event === 'object' ? formatEventValue(event.from) : '';
  const to = event && typeof event === 'object' ? formatEventValue(event.to) : '';
  const attemptedTo = event && typeof event === 'object' ? formatEventValue(event.attemptedTo) : '';
  const boxType = event && typeof event === 'object' ? event.boxType : null;
  const boxFrom = event && typeof event === 'object' ? formatEventValue(event.boxFrom) : '';
  const boxTo = event && typeof event === 'object' ? formatEventValue(event.boxTo) : '';

  const rows = [
    `<div class="hoverTipEvent ${eventKindClass(accepted)}">`,
    `<div class="hoverTipSection"><strong>Server action</strong><span>${escapeHtml(accepted)}</span></div>`,
    hoverField('Action', action),
    hoverField('Result', accepted === 'rejected' ? 'rejected by server' : 'accepted by server'),
    hoverField('From', from),
    hoverField('To', to),
    hoverField('Attempted to', attemptedTo && attemptedTo !== to ? attemptedTo : ''),
    hoverField('Box type', boxType),
    hoverField('Box from', boxFrom),
    hoverField('Box to', boxTo),
    hoverField('Message', event.message)
  ].filter(Boolean);
  rows.push('</div>');
  return rows.join('');
}

function hoverField(label, value) {
  const formatted = formatEventValue(value);
  if (!formatted) return '';
  return `<p><strong>${escapeHtml(label)}:</strong> ${escapeHtml(formatted)}</p>`;
}

function indexReplayEvents() {
  indexedReplayEvents = new Map();
  indexedReplayEventTotal = 0;
  if (!replay) return;
  for (let i = 0; i < replay.frames.length; i++) {
    indexedReplayEventTotal += frameLocalEventCount(replay.frames[i]);
  }
  if (!Array.isArray(replay.events)) return;
  for (const event of replay.events) {
    const index = eventStep(event);
    if (!Number.isInteger(index) || index < 0 || index >= replay.frames.length) continue;
    const bucket = indexedReplayEvents.get(index) || [];
    bucket.push(event);
    indexedReplayEvents.set(index, bucket);
    indexedReplayEventTotal++;
  }
}

function frameLocalEventCount(frame) {
  let count = 0;
  count += eventSourceCount(frame.events);
  count += eventSourceCount(frame.diagnostics);
  count += eventSourceCount(frame.debugEvents);
  count += eventSourceCount(frame.metadata?.events);
  return count;
}

function eventSourceCount(source) {
  if (!source) return 0;
  return Array.isArray(source) ? source.length : 1;
}

function collectEvents(target, source) {
  if (!source) return;
  if (Array.isArray(source)) {
    for (const item of source) target.push(item);
  } else {
    target.push(source);
  }
}

function replayEventCount() {
  return indexedReplayEventTotal;
}

function eventStep(event) {
  if (!event || typeof event !== 'object') return null;
  for (const key of ['step', 'frame', 'frameIndex', 'tick', 'timestep']) {
    if (Number.isInteger(event[key])) return event[key];
  }
  return null;
}

function renderEventCard(event) {
  const normalized = normalizeEvent(event);
  const card = document.createElement('article');
  card.className = `stepEventCard ${normalized.kindClass}`;

  const header = document.createElement('div');
  header.className = 'stepEventHeader';
  const title = document.createElement('strong');
  title.textContent = normalized.title;
  const kind = document.createElement('span');
  kind.textContent = normalized.kind;
  header.appendChild(title);
  header.appendChild(kind);
  card.appendChild(header);

  if (normalized.message) {
    const message = document.createElement('p');
    message.className = 'stepEventMessage';
    message.textContent = normalized.message;
    card.appendChild(message);
  }

  if (normalized.fields.length > 0) {
    const fields = document.createElement('dl');
    fields.className = 'stepEventFields';
    for (const [key, value] of normalized.fields) {
      const dt = document.createElement('dt');
      dt.textContent = key;
      const dd = document.createElement('dd');
      dd.textContent = formatEventValue(value);
      fields.appendChild(dt);
      fields.appendChild(dd);
    }
    card.appendChild(fields);
  }

  return card;
}

function normalizeEvent(event) {
  if (typeof event === 'string') {
    return {
      kind: 'note',
      kindClass: 'note',
      title: 'Event',
      message: event,
      fields: [],
    };
  }
  if (!event || typeof event !== 'object') {
    return {
      kind: 'note',
      kindClass: 'note',
      title: 'Event',
      message: String(event),
      fields: [],
    };
  }
  const kind = String(event.kind || event.type || event.category || event.severity || 'event');
  const title = String(event.title || event.label || event.name || titleCase(kind));
  const message = event.message || event.reason || event.summary || event.detail || '';
  const priority = event.severity || event.priority || event.level || kind;
  const fields = [];
  const preferred = [
    'agent', 'agentId', 'box', 'boxType', 'phase',
    'subgoal', 'subgoalType', 'verdict', 'action', 'actualAction',
    'stepInSegment', 'segmentSteps', 'from', 'to', 'activeSubgoal',
    'blocker', 'blockerType', 'targetBlocker', 'parking', 'releasedTo',
    'blocked', 'target', 'goal', 'cause', 'dominantReason',
    'selectedBox', 'selectionLayer', 'hungarianStatus',
    'candidateRejectCounts', 'candidateSamples',
    'bspAlgorithm', 'bspReason', 'bspExplored', 'bspBudget',
    'failedRounds', 'blockersBefore', 'blockersAfter',
    'accessDepthBefore', 'accessDepthAfter', 'ordinaryBlockersAfter',
    'protectedCells', 'taskCriticalCells', 'reservedTemps',
  ];
  const seen = new Set(['kind', 'type', 'category', 'severity', 'priority', 'level', 'title', 'label', 'name', 'message', 'reason', 'summary', 'detail', 'step', 'frame', 'frameIndex', 'tick', 'timestep']);
  for (const key of preferred) {
    if (event[key] !== undefined) {
      fields.push([labelForEventKey(key), event[key]]);
      seen.add(key);
    }
  }
  for (const [key, value] of Object.entries(event)) {
    if (seen.has(key) || value === undefined || value === null) continue;
    if (fields.length >= 18) break;
    fields.push([labelForEventKey(key), value]);
  }
  return {
    kind,
    kindClass: eventKindClass(priority),
    title,
    message: String(message || ''),
    fields,
  };
}

function eventKindClass(value) {
  const raw = String(value || '').toLowerCase();
  if (raw.includes('fail') || raw.includes('error') || raw.includes('reject') || raw.includes('block')) return 'danger';
  if (raw.includes('warn') || raw.includes('partial') || raw.includes('retry')) return 'warning';
  if (raw.includes('success') || raw.includes('ok') || raw.includes('solve') || raw.includes('accept')) return 'success';
  return 'note';
}

function labelForEventKey(key) {
  return String(key)
    .replace(/([a-z0-9])([A-Z])/g, '$1 $2')
    .replace(/_/g, ' ')
    .replace(/\b\w/g, ch => ch.toUpperCase());
}

function titleCase(value) {
  return labelForEventKey(String(value).replace(/[-:]/g, ' '));
}

function formatEventValue(value) {
  if (value === null || value === undefined) return '';
  if (typeof value === 'string' || typeof value === 'number' || typeof value === 'boolean') return String(value);
  if (Array.isArray(value)) {
    return value.map(formatEventValue).join(', ');
  }
  if (typeof value === 'object') {
    if (Number.isInteger(value.r) && Number.isInteger(value.c)) return `(${value.r}, ${value.c})`;
    if (Number.isInteger(value.row) && Number.isInteger(value.col)) return `(${value.row}, ${value.col})`;
    return JSON.stringify(value);
  }
  return String(value);
}

// ---- Playback --------------------------------------------------------------

function setStep(next) {
  step = Math.max(0, Math.min(next, replay ? replay.frames.length - 1 : 0));
  render();
}

function togglePlay() {
  if (!replay) return;
  if (timer) {
    stopPlay();
    return;
  }
  startPlay();
}

function stopPlay() {
  if (!timer) return;
  clearInterval(timer);
  timer = null;
  els.playBtn.textContent = 'Play';
  els.playBtn.classList.remove('active');
}

function startPlay() {
  if (!replay || timer) return;
  els.playBtn.textContent = 'Pause';
  els.playBtn.classList.add('active');
  timer = setInterval(() => {
    if (step >= replay.frames.length - 1) { togglePlay(); return; }
    setStep(step + 1);
  }, Math.max(20, Number(els.speedInput.value) || 120));
}

// ---- Helpers ---------------------------------------------------------------

function goalMap() {
  const out = new Map();
  for (const g of replay.level.boxGoals) out.set(`${g.r},${g.c}`, g.type);
  for (const g of replay.level.agentGoals) out.set(`${g.r},${g.c}`, String(g.agent));
  return out;
}

function changedCells(prev, cur) {
  const out = new Set();
  if (!prev || prev === cur) return out;
  const addAgentMoves = () => {
    const before = new Map(prev.agents.map(x => [x.id, x]));
    for (const x of cur.agents) {
      const y = before.get(x.id);
      if (!y || y.r !== x.r || y.c !== x.c) {
        out.add(`${x.r},${x.c}`);
        if (y) out.add(`${y.r},${y.c}`);
      }
    }
  };
  const addBoxMoves = () => {
    const before = new Set(prev.boxes.map(x => `${x.type}@${x.r},${x.c}`));
    const after = new Set(cur.boxes.map(x => `${x.type}@${x.r},${x.c}`));
    for (const x of cur.boxes) if (!before.has(`${x.type}@${x.r},${x.c}`)) out.add(`${x.r},${x.c}`);
    for (const x of prev.boxes) if (!after.has(`${x.type}@${x.r},${x.c}`)) out.add(`${x.r},${x.c}`);
  };
  addAgentMoves();
  addBoxMoves();
  return out;
}

function trackedCells() {
  const q = els.trackInput.value.trim().toLowerCase();
  const out = new Set();
  if (!q || !replay) return out;
  const frame = replay.frames[step];
  if (q.startsWith('agent')) {
    const id = Number(q.replace('agent', ''));
    for (const a of frame.agents) if (a.id === id) out.add(`${a.r},${a.c}`);
  } else if (q.startsWith('box')) {
    const type = q.replace('box', '').toUpperCase();
    for (const b of frame.boxes) if (b.type === type) out.add(`${b.r},${b.c}`);
  }
  return out;
}

function trailCells() {
  const q = els.trackInput.value.trim().toLowerCase();
  const out = new Set();
  if (!q || !replay) return out;
  for (let i = 0; i <= step; i++) {
    const frame = replay.frames[i];
    if (q.startsWith('agent')) {
      const id = Number(q.replace('agent', ''));
      for (const a of frame.agents) if (a.id === id) out.add(`${a.r},${a.c}`);
    } else if (q.startsWith('box')) {
      const type = q.replace('box', '').toUpperCase();
      for (const b of frame.boxes) if (b.type === type) out.add(`${b.r},${b.c}`);
    }
  }
  return out;
}

function parseCoord(text) {
  const m = String(text || '').trim().match(/^\(?\s*(\d+)\s*[, ]\s*(\d+)\s*\)?$/);
  if (!m) return null;
  return { r: Number(m[1]), c: Number(m[2]) };
}

// ---- DOM utilities ---------------------------------------------------------

function token(text, color, extraClass = '') {
  const el = document.createElement('span');
  el.className = extraClass ? `token ${extraClass}` : 'token';
  el.textContent = text;
  el.style.background = color;
  return el;
}
function boxColor(type) { return colors[replay.level.boxColors[type]] || colors.DEFAULT; }
function agentColor(id) { return colors[replay.level.agentColors[String(id)]] || colors.DEFAULT; }
function div(cls, text) { const el = document.createElement('div'); el.className = cls; el.textContent = text; return el; }
function escapeHtml(s) { return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }
function errorMessage(error) { return error && error.message ? error.message : String(error || 'Unknown error.'); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
