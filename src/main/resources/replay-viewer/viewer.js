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
let indexedDerivedActionEventTotal = 0;
let primaryTaskContextCache = null;
let subgoalDecisionContextCache = null;
let focusedAgentId = null;
let lockViewEnabled = true;
let agentPanelHeight = 260;
let timelineAgentId = null;
let timelineMode = 'overview';
let lifecycleSelection = 0;
let timelineSelectionKind = 'current';
let timelineSelectedSegmentIndex = null;
let timelineSelectedChapterIndex = null;
let timelineScrollLeft = 0;
let miniTimelineScrollLeft = 0;
let timelineAutoFollowDisabled = false;
let timelinePanSuppressClick = false;
let timelinePlayTogglePointerHandled = false;
let markers = new Map();
let searchState = { query: '', results: [], index: -1 };
let segmentState = { enabled: false, start: 0, end: 0, digest: '', digestVisible: false, source: '', timelineIndex: null, timelineKind: null };

const TUTORIAL_STORAGE_KEY = 'aims-replay-viewer:onboarding-seen';
const FONT_SIZE_STORAGE_KEY = 'aims-replay-viewer:font-size-px';
const LOG_FONT_SIZE_STORAGE_KEY = 'aims-replay-viewer:log-font-size-px';
const LAST_REPLAY_DB_NAME = 'aims-replay-viewer';
const LAST_REPLAY_DB_VERSION = 1;
const LAST_REPLAY_STORE_NAME = 'viewer-state';
const LAST_REPLAY_KEY = 'last-opened-replay';
const LAST_REPLAY_LOCAL_STORAGE_KEY = 'aims-replay-viewer:last-opened-replay';
const MARKERS_STORAGE_PREFIX = 'aims-replay-viewer:markers:';
const SUPPORTED_REPLAY_SCHEMA = 'mavis-hospital-replay-v1';
const DEFAULT_UI_FONT_SIZE = 12;
const DEFAULT_LOG_FONT_SIZE = 11;
const MIN_UI_FONT_SIZE = 8;
const MAX_UI_FONT_SIZE = 20;
const MIN_LOG_FONT_SIZE = 8;
const MAX_LOG_FONT_SIZE = 24;
const DEFAULT_AGENT_PANEL_HEIGHT = 260;
const MAX_AGENT_PANEL_HEIGHT = 420;
const MAX_TIMELINE_CHILD_EVENTS = 40;
const MAX_EXPORTED_DECISION_EVENTS = 80;
const MAX_STATE_DIFF_ITEMS = 20;
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
    title: 'Search and jump',
    body: 'Use Search / Jump to find rejected actions, agents, boxes, coordinates, event text, or a specific step. Example buttons show the accepted query shapes.'
  },
  {
    title: 'Play a range',
    body: 'Use Playback Range next to the main playback controls. Set start/end, press Play Range, optionally loop it, then export JSON or show a compact digest for that exact range.'
  },
  {
    title: 'Timeline and bookmarks',
    body: 'Use the left-side Lifecycle mini timeline for a compact stage strip, or open Timeline from the lower-right toolbar for the full stage view and metadata export. Clicking a stage selects and jumps; use Playback Range when you want to limit playback.'
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
  jumpSearchInput: document.getElementById('jumpSearchInput'),
  jumpPrevBtn: document.getElementById('jumpPrevBtn'),
  jumpNextBtn: document.getElementById('jumpNextBtn'),
  jumpSearchInfo: document.getElementById('jumpSearchInfo'),
  searchRejectedFilter: document.getElementById('searchRejectedFilter'),
  searchIntentFilter: document.getElementById('searchIntentFilter'),
  searchEventsFilter: document.getElementById('searchEventsFilter'),
  searchAgentFilter: document.getElementById('searchAgentFilter'),
  segmentEnabledInput: document.getElementById('segmentEnabledInput'),
  segmentStartInput: document.getElementById('segmentStartInput'),
  segmentEndInput: document.getElementById('segmentEndInput'),
  segmentSetStartBtn: document.getElementById('segmentSetStartBtn'),
  segmentSetEndBtn: document.getElementById('segmentSetEndBtn'),
  segmentPlayBtn: document.getElementById('segmentPlayBtn'),
  segmentClearBtn: document.getElementById('segmentClearBtn'),
  segmentLoopInput: document.getElementById('segmentLoopInput'),
  segmentExportBtn: document.getElementById('segmentExportBtn'),
  segmentDigestBtn: document.getElementById('segmentDigestBtn'),
  segmentCopyDigestBtn: document.getElementById('segmentCopyDigestBtn'),
  segmentMarkBtn: document.getElementById('segmentMarkBtn'),
  segmentInfo: document.getElementById('segmentInfo'),
  segmentDigestOutput: document.getElementById('segmentDigestOutput'),
  timelineAgentSelect: document.getElementById('timelineAgentSelect'),
  agentTimeline: document.getElementById('agentTimeline'),
  markerToggleBtn: document.getElementById('markerToggleBtn'),
  markerList: document.getElementById('markerList'),
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
  miniTimelineBtn: document.getElementById('miniTimelineBtn'),
  miniTimelinePanel: document.getElementById('miniTimelinePanel'),
  miniTimelineBody: document.getElementById('miniTimelineBody'),
  miniTimelinePlayBtn: document.getElementById('miniTimelinePlayBtn'),
  miniTimelineProgressTrack: document.getElementById('miniTimelineProgressTrack'),
  miniTimelineProgressFill: document.getElementById('miniTimelineProgressFill'),
  miniTimelineProgressText: document.getElementById('miniTimelineProgressText'),
  timelineBtn: document.getElementById('timelineBtn'),
  timelinePanel: document.getElementById('timelinePanel'),
  timelineClose: document.getElementById('timelineClose'),
  timelineKicker: document.getElementById('timelineKicker'),
  timelineTitle: document.getElementById('timelineTitle'),
  timelineOverviewBtn: document.getElementById('timelineOverviewBtn'),
  timelineExportMetaBtn: document.getElementById('timelineExportMetaBtn'),
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
      if (e.key === 'Escape' && els.timelinePanel && !els.timelinePanel.hidden) closeTimelinePanel();
      if (e.key === 'Escape' && els.miniTimelinePanel && !els.miniTimelinePanel.hidden) closeMiniTimelinePanel();
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
  els.jumpSearchInput.addEventListener('input', () => updateSearchResults(false));
  els.jumpSearchInput.addEventListener('keydown', e => {
    if (e.key === 'Enter') {
      e.preventDefault();
      jumpSearch(e.shiftKey ? -1 : 1);
    }
  });
  for (const filter of [els.searchRejectedFilter, els.searchIntentFilter, els.searchEventsFilter, els.searchAgentFilter]) {
    filter.addEventListener('change', () => updateSearchResults(false));
  }
  els.jumpPrevBtn.addEventListener('click', () => jumpSearch(-1));
  els.jumpNextBtn.addEventListener('click', () => jumpSearch(1));
  els.jumpSearchInput.closest('.searchJumpPanel')?.addEventListener('click', event => {
    if (!(event.target instanceof Element)) return;
    const button = event.target.closest('button[data-search-example]');
    if (!button) return;
    els.jumpSearchInput.value = button.dataset.searchExample || '';
    updateSearchResults(true);
    els.jumpSearchInput.focus();
  });
  els.segmentEnabledInput.addEventListener('change', () => {
    segmentState.enabled = els.segmentEnabledInput.checked;
    if (!segmentState.enabled) {
      segmentState.timelineIndex = null;
      segmentState.timelineKind = null;
    }
    renderSegmentPanel();
  });
  els.segmentStartInput.addEventListener('change', () => updateSegmentFromInputs());
  els.segmentEndInput.addEventListener('change', () => updateSegmentFromInputs());
  els.segmentSetStartBtn.addEventListener('click', () => setSegmentBoundary('start', step));
  els.segmentSetEndBtn.addEventListener('click', () => setSegmentBoundary('end', step));
  els.segmentPlayBtn.addEventListener('click', playSegmentRange);
  els.segmentClearBtn.addEventListener('click', clearSegmentRange);
  els.segmentExportBtn.addEventListener('click', exportSegmentJson);
  els.segmentDigestBtn.addEventListener('click', renderSegmentDigest);
  els.segmentCopyDigestBtn.addEventListener('click', copySegmentDigest);
  els.segmentMarkBtn.addEventListener('click', toggleCurrentSegmentMarker);
  els.timelineAgentSelect.addEventListener('change', () => {
    const value = els.timelineAgentSelect.value;
    if (value === '') {
      timelineAgentId = null;
      timelineMode = 'overview';
      if (timer) timelineAutoFollowDisabled = false;
    } else {
      const raw = Number(value);
      timelineAgentId = Number.isInteger(raw) ? raw : null;
      timelineMode = timelineAgentId == null ? 'overview' : 'agent';
    }
    syncTimelineAgentSelect();
    renderAgentTimeline();
  });
  els.timelineOverviewBtn.addEventListener('click', () => {
    timelineMode = 'overview';
    if (timer) timelineAutoFollowDisabled = false;
    renderAgentTimeline();
  });
  els.timelineExportMetaBtn?.addEventListener('click', exportTimelineMetaJson);
  els.agentTimeline.addEventListener('pointerdown', event => {
    if (!(event.target instanceof Element)) return;
    const timelinePlayToggleButton = event.target.closest('button[data-timeline-play-toggle]');
    if (!timelinePlayToggleButton || event.button !== 0) return;
    event.preventDefault();
    event.stopPropagation();
    timelinePlayTogglePointerHandled = true;
    togglePlay();
  }, true);
  els.agentTimeline.addEventListener('click', event => {
    if (!(event.target instanceof Element)) return;
    // A pan-drag just finished on this click target: swallow the click so a drag
    // does not also select/jump.
    if (timelinePanSuppressClick) { timelinePanSuppressClick = false; return; }
    // Clicks on a recognised timeline control must not bubble to the document-level
    // "click outside to close" handler: the control's own handler re-renders the
    // timeline and detaches this node, which would otherwise be misread as an
    // outside click and close the whole panel.
    if (event.target.closest('button[data-lifecycle-index], button[data-lifecycle-jump-index], button[data-lifecycle-focus-index], button[data-lifecycle-play-index], button[data-timeline-play-toggle], button[data-chapter-play-index], button[data-chapter-focus-index], button[data-chapter-jump-index], button[data-chapter-index], button[data-step]')) {
      event.stopPropagation();
    }
    const timelinePlayToggleButton = event.target.closest('button[data-timeline-play-toggle]');
    if (timelinePlayToggleButton) {
      event.preventDefault();
      if (event.detail > 0 && timelinePlayTogglePointerHandled) {
        timelinePlayTogglePointerHandled = false;
        return;
      }
      togglePlay();
      return;
    }
    const lifecyclePlayButton = event.target.closest('button[data-lifecycle-play-index]');
    if (lifecyclePlayButton) {
      playLifecycleSegment(Number(lifecyclePlayButton.dataset.lifecyclePlayIndex));
      return;
    }
    const lifecycleFocusButton = event.target.closest('button[data-lifecycle-focus-index]');
    if (lifecycleFocusButton) {
      focusLifecycleSegment(Number(lifecycleFocusButton.dataset.lifecycleFocusIndex));
      return;
    }
    const lifecycleJumpButton = event.target.closest('button[data-lifecycle-jump-index]');
    if (lifecycleJumpButton) {
      jumpLifecycleSegment(Number(lifecycleJumpButton.dataset.lifecycleJumpIndex));
      return;
    }
    const lifecycleButton = event.target.closest('button[data-lifecycle-index]');
    if (lifecycleButton) {
      selectLifecycleSegment(Number(lifecycleButton.dataset.lifecycleIndex));
      return;
    }
    const chapterPlayButton = event.target.closest('button[data-chapter-play-index]');
    if (chapterPlayButton) {
      playDebugChapter(Number(chapterPlayButton.dataset.chapterPlayIndex));
      return;
    }
    const chapterFocusButton = event.target.closest('button[data-chapter-focus-index]');
    if (chapterFocusButton) {
      focusDebugChapter(Number(chapterFocusButton.dataset.chapterFocusIndex));
      return;
    }
    const chapterJumpButton = event.target.closest('button[data-chapter-jump-index]');
    if (chapterJumpButton) {
      jumpDebugChapter(Number(chapterJumpButton.dataset.chapterJumpIndex));
      return;
    }
    const chapterButton = event.target.closest('button[data-chapter-index]');
    if (chapterButton) {
      selectDebugChapter(Number(chapterButton.dataset.chapterIndex));
      return;
    }
    const button = event.target.closest('button[data-step]');
    if (!button) return;
    const start = Number(button.dataset.segmentStart ?? button.dataset.step);
    const end = Number(button.dataset.segmentEnd ?? button.dataset.step);
    jumpTimelineRange(start, end);
  });
  els.agentTimeline.addEventListener('pointerover', event => {
    if (!(event.target instanceof Element)) return;
    const node = event.target.closest('.timelineEventNode');
    if (!node || (event.relatedTarget instanceof Node && node.contains(event.relatedTarget))) return;
    positionTimelineDropdown(node);
  });
  els.agentTimeline.addEventListener('pointerout', event => {
    if (!(event.target instanceof Element)) return;
    const node = event.target.closest('.timelineEventNode');
    if (!node || (event.relatedTarget instanceof Node && node.contains(event.relatedTarget))) return;
    clearTimelineDropdownPosition(node);
  });
  els.agentTimeline.addEventListener('focusin', event => {
    if (!(event.target instanceof Element)) return;
    const node = event.target.closest('.timelineEventNode');
    if (node) positionTimelineDropdown(node);
  });
  els.agentTimeline.addEventListener('focusout', event => {
    if (!(event.target instanceof Element)) return;
    const node = event.target.closest('.timelineEventNode');
    if (!node) return;
    requestAnimationFrame(() => {
      if (!node.contains(document.activeElement)) clearTimelineDropdownPosition(node);
    });
  });
  els.timelineBtn.addEventListener('click', toggleTimelinePanel);
  els.timelineClose.addEventListener('click', closeTimelinePanel);
  els.timelinePanel.addEventListener('click', event => {
    if (event.target === els.timelinePanel) closeTimelinePanel();
  });
  els.miniTimelineBtn?.addEventListener('click', event => {
    event.stopPropagation();
    toggleMiniTimelinePanel();
  });
  els.miniTimelinePlayBtn?.addEventListener('click', event => {
    event.stopPropagation();
    togglePlay();
    renderMiniTimelinePanel();
  });
  els.miniTimelineBody?.addEventListener('click', event => {
    if (!(event.target instanceof Element)) return;
    if (timelinePanSuppressClick) { timelinePanSuppressClick = false; return; }
    const button = event.target.closest('button[data-lifecycle-index]');
    if (!button) return;
    event.stopPropagation();
    selectLifecycleSegment(Number(button.dataset.lifecycleIndex));
  });
  els.miniTimelineProgressTrack?.addEventListener('click', event => {
    event.stopPropagation();
    if (!replay) return;
    const rect = els.miniTimelineProgressTrack.getBoundingClientRect();
    const ratio = rect.width > 0 ? clamp((event.clientX - rect.left) / rect.width, 0, 1) : 0;
    setStep(Math.round(ratio * (replay.frames.length - 1)));
  });
  els.markerToggleBtn.addEventListener('click', toggleCurrentMarker);
  els.markerList.addEventListener('click', event => {
    if (!(event.target instanceof Element)) return;
    const deleteButton = event.target.closest('button[data-marker-delete]');
    if (deleteButton) {
      deleteMarker(deleteButton.dataset.markerDelete || '');
      return;
    }
    const button = event.target.closest('button[data-step]');
    if (!button) return;
    setStep(Number(button.dataset.step));
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
    if (!(event.target instanceof Element) || !event.target.closest('#timelinePanel, #timelineBtn')) {
      closeTimelinePanel();
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
    positionHoverTip(e);
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

function positionHoverTip(event) {
  if (!els.hoverTip) return;
  const margin = 8;
  const offset = 14;
  const vw = Math.max(1, window.innerWidth || document.documentElement.clientWidth || 1);
  const vh = Math.max(1, window.innerHeight || document.documentElement.clientHeight || 1);
  els.hoverTip.style.maxWidth = `${Math.max(160, vw - margin * 2)}px`;
  els.hoverTip.style.maxHeight = `${Math.max(120, vh - margin * 2)}px`;
  els.hoverTip.style.left = `${margin}px`;
  els.hoverTip.style.top = `${margin}px`;
  els.hoverTip.classList.add('show');

  const rect = els.hoverTip.getBoundingClientRect();
  let left = event.clientX + offset;
  let top = event.clientY + offset;
  if (left + rect.width + margin > vw) left = event.clientX - rect.width - offset;
  if (top + rect.height + margin > vh) top = event.clientY - rect.height - offset;
  left = clamp(left, margin, Math.max(margin, vw - rect.width - margin));
  top = clamp(top, margin, Math.max(margin, vh - rect.height - margin));
  els.hoverTip.style.left = `${Math.round(left)}px`;
  els.hoverTip.style.top = `${Math.round(top)}px`;
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
  const savedUi = storedFontSize(FONT_SIZE_STORAGE_KEY);
  const savedLog = storedFontSize(LOG_FONT_SIZE_STORAGE_KEY);
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

function storedFontSize(key) {
  if (!localStorageAvailable()) return NaN;
  const raw = localStorage.getItem(key);
  if (raw == null || raw === '') return NaN;
  return Number(raw);
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

function replayTextBytes(text) {
  const value = String(text ?? '');
  if (typeof Blob !== 'undefined') return new Blob([value]).size;
  if (typeof TextEncoder !== 'undefined') return new TextEncoder().encode(value).length;
  return value.length;
}

function formatBytes(bytes) {
  const value = Math.max(0, Number(bytes) || 0);
  if (value < 1024) return `${value} B`;
  const units = ['KB', 'MB', 'GB'];
  let size = value / 1024;
  let unitIndex = 0;
  while (size >= 1024 && unitIndex < units.length - 1) {
    size /= 1024;
    unitIndex++;
  }
  return `${size >= 10 ? size.toFixed(1) : size.toFixed(2)} ${units[unitIndex]}`;
}

async function browserStorageEstimate() {
  try {
    if (typeof navigator === 'undefined' || !navigator.storage?.estimate) return null;
    const estimate = await navigator.storage.estimate();
    const quota = Number(estimate?.quota);
    const usage = Number(estimate?.usage);
    if (!Number.isFinite(quota) || quota <= 0 || !Number.isFinite(usage)) return null;
    return { quota, usage };
  } catch (error) {
    console.warn('Unable to estimate browser storage:', error);
    return null;
  }
}

async function rememberedReplayStorageBytes() {
  try {
    const record = await readReplayStorage();
    if (record) return replayTextBytes(JSON.stringify(record));
  } catch (_) {
    // IndexedDB may be unavailable; localStorage fallback is checked below.
  }
  if (!localStorageAvailable()) return 0;
  return replayTextBytes(localStorage.getItem(LAST_REPLAY_LOCAL_STORAGE_KEY) || '');
}

async function replayStorageCanAccept(serializedRecord) {
  const estimate = await browserStorageEstimate();
  if (!estimate) return true;
  const replacementBytes = await rememberedReplayStorageBytes();
  const availableAfterReplace = Math.max(0, estimate.quota - estimate.usage + replacementBytes);
  return replayTextBytes(serializedRecord) <= availableAfterReplace;
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

function setDisabled(control, disabled) {
  if (!control) return;
  control.disabled = Boolean(disabled);
  if (disabled) control.setAttribute('aria-disabled', 'true');
  else control.removeAttribute('aria-disabled');
}

function setControlsDisabled(controls, disabled) {
  for (const control of controls) setDisabled(control, disabled);
}

function syncReplayControlAvailability() {
  const hasReplay = Boolean(replay);
  const hasAgents = agentIds().length > 0;
  const replayControls = [
    els.playBtn,
    els.prevBtn,
    els.nextBtn,
    els.slider,
    els.stepInput,
    els.trackInput,
    els.coordInput,
    els.coordGoBtn,
    els.coordClearBtn,
    els.jumpSearchInput,
    els.jumpPrevBtn,
    els.jumpNextBtn,
    els.searchRejectedFilter,
    els.searchIntentFilter,
    els.searchEventsFilter,
    els.searchAgentFilter,
    els.segmentEnabledInput,
    els.segmentStartInput,
    els.segmentEndInput,
    els.segmentSetStartBtn,
    els.segmentSetEndBtn,
    els.segmentPlayBtn,
    els.segmentClearBtn,
    els.segmentLoopInput,
    els.segmentExportBtn,
    els.segmentDigestBtn,
    els.segmentCopyDigestBtn,
    els.segmentMarkBtn,
    els.markerToggleBtn,
    els.timelineOverviewBtn,
    els.timelineExportMetaBtn,
    els.miniTimelinePlayBtn,
    els.miniTimelineProgressTrack,
  ];
  setControlsDisabled(replayControls, !hasReplay);
  for (const button of document.querySelectorAll('.searchExamples button')) {
    setDisabled(button, !hasReplay);
  }
  for (const button of [els.timelineBtn, els.stepEventsBtn, els.miniTimelineBtn, els.agentMenuBtn]) {
    if (button) button.hidden = false;
  }
  setDisabled(els.timelineBtn, !hasReplay);
  setDisabled(els.stepEventsBtn, !hasReplay);
  setDisabled(els.miniTimelineBtn, !hasReplay);
  setDisabled(els.timelineAgentSelect, !hasReplay || !hasAgents);
  setDisabled(els.agentMenuBtn, !hasAgents);
  setDisabled(els.lockViewInput, !hasReplay || focusedAgentId == null);
  if (!hasReplay) {
    els.playBtn.textContent = 'Play';
    els.playBtn.classList.remove('active');
    els.stepEventsBtn.textContent = 'Highlights 0';
    els.stepEventsBtn.classList.add('muted');
    els.timelineBtn.classList.remove('active');
    els.timelineBtn.setAttribute('aria-expanded', 'false');
    els.miniTimelineBtn?.classList.remove('active');
    els.miniTimelineBtn?.setAttribute('aria-expanded', 'false');
    els.agentMenuBtn.classList.remove('active');
    els.agentMenuBtn.setAttribute('aria-expanded', 'false');
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
    forgetLastOpenedReplay().catch(clearError => {
      console.warn('Unable to clear invalid remembered replay:', clearError);
    });
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
  const serialized = JSON.stringify(record);
  if (!(await replayStorageCanAccept(serialized))) {
    await forgetLastOpenedReplay();
    showError(
      `This replay is ${formatBytes(replayTextBytes(text))}; browser storage does not report enough free space for automatic restore.`,
      'Replay loaded but not remembered'
    );
    return;
  }
  try {
    await writeReplayStorage(record);
  } catch (error) {
    if (!writeReplayLocalStorage(serialized)) {
      await forgetLastOpenedReplay();
      showError(
        `This replay is ${formatBytes(replayTextBytes(text))}; browser storage could not save it for automatic restore.`,
        'Replay loaded but not remembered'
      );
    }
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

function writeReplayLocalStorage(serializedRecord) {
  if (!localStorageAvailable()) return false;
  try {
    localStorage.setItem(LAST_REPLAY_LOCAL_STORAGE_KEY, serializedRecord);
    return true;
  } catch (error) {
    console.warn('Unable to save replay in localStorage:', error);
    return false;
  }
}

async function forgetLastOpenedReplay() {
  if (localStorageAvailable()) {
    localStorage.removeItem(LAST_REPLAY_LOCAL_STORAGE_KEY);
  }
  try {
    await deleteReplayStorage();
  } catch (error) {
    console.warn('Unable to clear IndexedDB replay storage:', error);
  }
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

async function deleteReplayStorage() {
  const db = await openReplayStorage();
  return new Promise((resolve, reject) => {
    const tx = db.transaction(LAST_REPLAY_STORE_NAME, 'readwrite');
    tx.objectStore(LAST_REPLAY_STORE_NAME).delete(LAST_REPLAY_KEY);
    tx.oncomplete = () => {
      db.close();
      resolve();
    };
    tx.onerror = () => {
      db.close();
      reject(tx.error || new Error('Unable to clear replay storage.'));
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
  const previous = captureReplayLoadState();
  stopPlay();
  try {
    replay = data;
    indexReplayEvents();
    primaryTaskContextCache = null;
    subgoalDecisionContextCache = null;
    step = 0;
    timelineMode = 'overview';
    timelineScrollLeft = 0;
    lifecycleSelection = 0;
    timelineSelectionKind = 'current';
    timelineSelectedSegmentIndex = null;
    timelineSelectedChapterIndex = null;
    segmentState = { enabled: false, start: 0, end: replay.frames.length - 1, digest: '', digestVisible: false, source: '', timelineIndex: null, timelineKind: null };
    els.slider.max = replay.frames.length - 1;
    els.stepInput.max = replay.frames.length - 1;
    els.segmentStartInput.max = replay.frames.length - 1;
    els.segmentEndInput.max = replay.frames.length - 1;
    const s = replay.summary || {};
    renderReplayMeta(s);
    populateAgentFocusMenu();
    populateTimelineAgentSelect();
    loadReplayMarkers();
    updateSearchResults(false);
    buildBoard();
    render();
    if (els.autoPlayInput.checked) startPlay();
  } catch (error) {
    restoreReplayLoadState(previous);
    try {
      if (replay) renderReplayMeta(replay.summary || {});
      else renderEmptyReplayMeta();
      render();
    } catch (restoreError) {
      console.warn('Unable to restore previous replay after load failure:', restoreError);
    }
    throw error;
  }
}

function captureReplayLoadState() {
  return {
    replay,
    step,
    highlightCoord,
    indexedReplayEvents: new Map(indexedReplayEvents),
    indexedReplayEventTotal,
    indexedDerivedActionEventTotal,
    primaryTaskContextCache,
    subgoalDecisionContextCache,
    focusedAgentId,
    timelineAgentId,
    timelineMode,
    lifecycleSelection,
    timelineSelectionKind,
    timelineSelectedSegmentIndex,
    timelineSelectedChapterIndex,
    timelineScrollLeft,
    miniTimelineScrollLeft,
    timelineAutoFollowDisabled,
    markers: new Map(markers),
    searchState: { ...searchState, results: [...(searchState.results || [])] },
    segmentState: { ...segmentState },
  };
}

function restoreReplayLoadState(state) {
  replay = state.replay;
  step = state.step;
  highlightCoord = state.highlightCoord;
  indexedReplayEvents = new Map(state.indexedReplayEvents);
  indexedReplayEventTotal = state.indexedReplayEventTotal;
  indexedDerivedActionEventTotal = state.indexedDerivedActionEventTotal;
  primaryTaskContextCache = state.primaryTaskContextCache;
  subgoalDecisionContextCache = state.subgoalDecisionContextCache;
  focusedAgentId = state.focusedAgentId;
  timelineAgentId = state.timelineAgentId;
  timelineMode = state.timelineMode;
  lifecycleSelection = state.lifecycleSelection;
  timelineSelectionKind = state.timelineSelectionKind;
  timelineSelectedSegmentIndex = state.timelineSelectedSegmentIndex;
  timelineSelectedChapterIndex = state.timelineSelectedChapterIndex;
  timelineScrollLeft = state.timelineScrollLeft;
  miniTimelineScrollLeft = state.miniTimelineScrollLeft;
  timelineAutoFollowDisabled = state.timelineAutoFollowDisabled;
  markers = new Map(state.markers);
  searchState = { ...state.searchState, results: [...(state.searchState.results || [])] };
  segmentState = { ...state.segmentState };
}

function renderReplayMeta(summary = {}) {
  const identity = replayRunIdentityDetails();
  els.meta.removeAttribute('title');
  els.meta.innerHTML = `
    <b>${escapeHtml(replay.level.name)}</b><br>
    ${replay.level.rows}x${replay.level.cols}, ${replay.frames.length} frames<br>
    ${escapeHtml(summary.outcome || 'unknown')} | ${summary.executedSteps ?? 0} steps | ${summary.satisfiedBoxGoals ?? 0}/${summary.totalBoxGoals ?? 0} box goals<br>
    ${escapeHtml(replay.generatedAt || '')}
    ${identity.html}
  `;
}

function renderEmptyReplayMeta() {
  els.meta.removeAttribute('title');
  els.meta.textContent = 'No replay loaded';
}

function replayRunIdentityDetails() {
  const runContext = replay?.diagnostics?.runContext || {};
  const codeVersion = runContext.codeVersion || {};
  const fingerprint = runContext.levelFingerprint || {};
  const searchConfig = runContext.searchConfig || {};
  const hasRunContext = Object.keys(runContext).length > 0;
  if (!hasRunContext) return { html: '', title: '' };
  const codeLabel = codeVersion.gitCommitShort
    ? `${codeVersion.gitBranch || 'git'}@${codeVersion.gitCommitShort}${codeVersion.gitDirty ? ' dirty' : ''}`
    : 'not recorded';
  const rows = [
    ['Code', codeLabel],
    ['Artifact SHA', shortHash(codeVersion.runtimeArtifactSha256)],
    ['Level SHA', shortHash(fingerprint.levelStaticSha256)],
    ['Initial SHA', shortHash(fingerprint.initialStateSha256)],
    ['Search Budget', runContext.planningTimeoutMs != null ? `${runContext.planningTimeoutMs} ms` : searchConfig.planningTimeoutMs],
    ['Max Actions', runContext.maxActions ?? searchConfig.maxActions],
    ['Max States', runContext.defaultMaxStates ?? searchConfig.defaultMaxStates],
    ['BSP Budget', runContext.maxBspBudget ?? searchConfig.maxBspBudget],
    ['Log Level', runContext.logLevel ?? searchConfig.logLevel],
  ].filter(([, value]) => value !== undefined && value !== null && value !== '');
  const body = rows.map(([label, value]) => `<dt>${escapeHtml(label)}</dt><dd>${escapeHtml(formatEventValue(value))}</dd>`).join('');
  return {
    html: `
      <span class="metaRunHint">Run context available</span>
      <div class="metaHoverCard" role="tooltip">
        <strong>Run Context</strong>
        <dl>${body}</dl>
      </div>
    `,
  };
}

function shortHash(value, size = 12) {
  const raw = formatEventValue(value);
  if (!raw) return 'not recorded';
  return raw.length > size ? raw.slice(0, size) : raw;
}

function validateReplay(data) {
  if (!data || typeof data !== 'object') {
    throw new Error('The selected file is not a replay object.');
  }
  if (data.schema !== SUPPORTED_REPLAY_SCHEMA) {
    throw new Error(`Unsupported replay schema: ${data.schema || 'missing'}. Expected ${SUPPORTED_REPLAY_SCHEMA}.`);
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
  if (!Number.isInteger(level.rows) || level.rows <= 0 || !Number.isInteger(level.cols) || level.cols <= 0) {
    throw new Error('level.rows and level.cols must be positive integers.');
  }
  for (const field of ['name', 'rows', 'cols', 'walls', 'boxGoals', 'agentGoals', 'agentColors', 'boxColors']) {
    if (!(field in level)) throw new Error(`Missing required field: level.${field}.`);
  }
  if (typeof level.name !== 'string') {
    throw new Error('level.name must be a string.');
  }
  if (!Array.isArray(level.walls) || level.walls.length !== level.rows) {
    throw new Error('level.walls must contain one row string per level row.');
  }
  for (let r = 0; r < level.walls.length; r++) {
    if (typeof level.walls[r] !== 'string' || level.walls[r].length !== level.cols) {
      throw new Error(`level.walls[${r}] must be a string with ${level.cols} columns.`);
    }
  }
  if (!Array.isArray(level.boxGoals) || !Array.isArray(level.agentGoals)) {
    throw new Error('level.boxGoals and level.agentGoals must be arrays.');
  }
  if (!level.agentColors || typeof level.agentColors !== 'object' || Array.isArray(level.agentColors)) {
    throw new Error('level.agentColors must be an object.');
  }
  if (!level.boxColors || typeof level.boxColors !== 'object' || Array.isArray(level.boxColors)) {
    throw new Error('level.boxColors must be an object.');
  }
  level.boxGoals.forEach((goal, index) => validateBoxGoal(goal, index, level));
  level.agentGoals.forEach((goal, index) => validateAgentGoal(goal, index, level));
  for (let i = 0; i < data.frames.length; i++) {
    const frame = data.frames[i];
    if (!frame || typeof frame !== 'object') throw new Error(`Frame ${i} is not an object.`);
    if (!Array.isArray(frame.agents)) throw new Error(`Frame ${i} is missing agents array.`);
    if (!Array.isArray(frame.boxes)) throw new Error(`Frame ${i} is missing boxes array.`);
    if (!Array.isArray(frame.actions)) throw new Error(`Frame ${i} is missing actions array.`);
    if (!Array.isArray(frame.accepted)) throw new Error(`Frame ${i} is missing accepted array.`);
    validateFrameAgents(frame.agents, i, level);
    validateFrameBoxes(frame.boxes, i, level);
    validateFrameActions(frame, i);
  }
}

function validateBoxGoal(goal, index, level) {
  if (!goal || typeof goal !== 'object') throw new Error(`level.boxGoals[${index}] must be an object.`);
  if (!formatEventValue(goal.type)) throw new Error(`level.boxGoals[${index}].type is required.`);
  validateGridCoord(goal, `level.boxGoals[${index}]`, level);
}

function validateAgentGoal(goal, index, level) {
  if (!goal || typeof goal !== 'object') throw new Error(`level.agentGoals[${index}] must be an object.`);
  if (goal.agent === undefined || goal.agent === null || goal.agent === '') {
    throw new Error(`level.agentGoals[${index}].agent is required.`);
  }
  validateGridCoord(goal, `level.agentGoals[${index}]`, level);
}

function validateFrameAgents(agents, frameIndex, level) {
  const ids = new Set();
  for (let i = 0; i < agents.length; i++) {
    const agent = agents[i];
    if (!agent || typeof agent !== 'object') throw new Error(`Frame ${frameIndex} agent ${i} must be an object.`);
    if (!Number.isInteger(agent.id) || agent.id < 0) {
      throw new Error(`Frame ${frameIndex} agent ${i} has invalid id.`);
    }
    if (ids.has(agent.id)) throw new Error(`Frame ${frameIndex} has duplicate agent id ${agent.id}.`);
    ids.add(agent.id);
    validateGridCoord(agent, `Frame ${frameIndex} agent ${agent.id}`, level);
  }
}

function validateFrameBoxes(boxes, frameIndex, level) {
  for (let i = 0; i < boxes.length; i++) {
    const box = boxes[i];
    if (!box || typeof box !== 'object') throw new Error(`Frame ${frameIndex} box ${i} must be an object.`);
    if (!formatEventValue(box.type)) throw new Error(`Frame ${frameIndex} box ${i} is missing type.`);
    validateGridCoord(box, `Frame ${frameIndex} box ${i}`, level);
  }
}

function validateFrameActions(frame, frameIndex) {
  for (let i = 0; i < frame.actions.length; i++) {
    const action = frame.actions[i];
    if (action !== null && action !== undefined && typeof action !== 'string') {
      throw new Error(`Frame ${frameIndex} action ${i} must be a string.`);
    }
  }
  for (let i = 0; i < frame.accepted.length; i++) {
    const accepted = frame.accepted[i];
    if (accepted !== null && accepted !== undefined && typeof accepted !== 'boolean') {
      throw new Error(`Frame ${frameIndex} accepted ${i} must be a boolean.`);
    }
  }
}

function validateGridCoord(value, label, level) {
  if (!Number.isInteger(value.r) || !Number.isInteger(value.c)) {
    throw new Error(`${label} must have integer r and c coordinates.`);
  }
  if (value.r < 0 || value.r >= level.rows || value.c < 0 || value.c >= level.cols) {
    throw new Error(`${label} coordinate (${value.r}, ${value.c}) is outside the ${level.rows}x${level.cols} level.`);
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
    renderEmptyReplayMeta();
    els.statusInfo.textContent = '';
    closeAgentMenu();
    closeTimelinePanel();
    closeMiniTimelinePanel();
    els.agentActionPanel.hidden = true;
    els.boardWrap.classList.remove('agentPanelOpen', 'agentMenuOpen');
    renderSearchInfo();
    renderSegmentPanel();
    renderAgentTimeline();
    renderMarkers();
    syncReplayControlAvailability();
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
  syncSearchIndexToStep();
  renderSearchInfo();
  renderSegmentPanel();
  renderAgentTimeline();
  renderMiniTimelinePanel();
  renderMarkers();
  syncReplayControlAvailability();
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

function updateSearchResults(jumpToNearest) {
  const query = els.jumpSearchInput.value.trim();
  searchState.query = query;
  searchState.results = query || hasActiveSearchFilters() ? computeSearchResults(query) : [];
  searchState.index = nearestSearchIndex(step);
  if (jumpToNearest && searchState.index >= 0) {
    setStep(searchState.results[searchState.index].step);
  } else {
    renderSearchInfo();
  }
}

function computeSearchResults(query) {
  if (!replay) return [];
  const normalized = normalizeSearchText(query);
  const directStep = directStepQuery(normalized);
  if (directStep != null) {
    const bounded = Math.max(0, Math.min(directStep, replay.frames.length - 1));
    return [{ step: bounded, label: `step ${bounded}`, detail: 'direct step jump' }];
  }
  const coord = parseCoord(query);
  const filters = activeSearchFilters();
  const results = [];
  for (let i = 0; i < replay.frames.length; i++) {
    const frame = replay.frames[i];
    if (!frameMatchesFilters(frame, i, filters)) continue;
    if (coord && frameContainsCoord(frame, coord)) {
      results.push(searchResult(i, 'coordinate match', `(${coord.r}, ${coord.c})`));
      continue;
    }
    if (!normalized) {
      results.push(searchResult(i, searchFilterLabel(filters), searchResultDetail(frame)));
      continue;
    }
    const text = frameSearchText(frame, i);
    if (text.includes(normalized)) {
      results.push(searchResult(i, searchResultLabel(frame, normalized), searchResultDetail(frame)));
    }
  }
  return results;
}

function hasActiveSearchFilters() {
  return els.searchRejectedFilter.checked
      || els.searchIntentFilter.checked
      || els.searchEventsFilter.checked
      || els.searchAgentFilter.checked;
}

function activeSearchFilters() {
  return {
    rejected: els.searchRejectedFilter.checked,
    intent: els.searchIntentFilter.checked,
    events: els.searchEventsFilter.checked,
    agent: els.searchAgentFilter.checked ? (focusedAgentId ?? timelineAgentId) : null,
  };
}

function frameMatchesFilters(frame, stepIndex, filters) {
  const events = stepEvents(frame, stepIndex);
  if (filters.rejected && !frameHasRejected(frame, events)) return false;
  if (filters.intent && !events.some(event => isAgentIntentEvent(event) || isAgentExecutionContextEvent(event))) return false;
  if (filters.events && events.length === 0) return false;
  if (filters.agent != null && !frameMentionsAgent(frame, events, filters.agent)) return false;
  return true;
}

function frameHasRejected(frame, events) {
  return (frame.accepted || []).some(value => value === false)
      || events.some(event => isAgentActionEvent(event) && event.accepted === false)
      || events.some(event => String(event?.kind || event?.type || '').toLowerCase() === 'rejected-action');
}

function frameMentionsAgent(frame, events, agentId) {
  if ((frame.agents || []).some(agent => agent.id === agentId)) return true;
  if (frame.actions && agentId < frame.actions.length) return true;
  return events.some(event => eventAgentId(event) === agentId);
}

function searchFilterLabel(filters) {
  const labels = [];
  if (filters.rejected) labels.push('rejected');
  if (filters.intent) labels.push('intent');
  if (filters.events) labels.push('event');
  if (filters.agent != null) labels.push(`agent${filters.agent}`);
  return labels.length ? labels.join(' + ') : 'match';
}

function directStepQuery(query) {
  const match = query.match(/^(?:step\s*)?(\d+)$/);
  return match ? Number(match[1]) : null;
}

function frameContainsCoord(frame, coord) {
  return (frame.agents || []).some(agent => agent.r === coord.r && agent.c === coord.c)
      || (frame.boxes || []).some(box => box.r === coord.r && box.c === coord.c);
}

function frameSearchText(frame, stepIndex) {
  const parts = [`step ${stepIndex}`];
  for (const [agentId, action] of (frame.actions || []).entries()) {
    const accepted = frame.accepted && frame.accepted[agentId] !== false;
    parts.push(`agent${agentId}`, action, accepted ? 'accepted' : 'rejected');
  }
  for (const agent of frame.agents || []) {
    parts.push(`agent${agent.id}`, `${agent.r},${agent.c}`, `(${agent.r}, ${agent.c})`);
  }
  for (const box of frame.boxes || []) {
    parts.push(`box${box.type}`, box.type, `${box.r},${box.c}`, `(${box.r}, ${box.c})`);
  }
  for (const event of stepEvents(frame, stepIndex)) {
    parts.push(eventSearchText(event));
  }
  return normalizeSearchText(parts.join(' | '));
}

function eventSearchText(event) {
  if (event == null) return '';
  if (typeof event !== 'object') return String(event);
  return Object.entries(event)
    .map(([key, value]) => `${key} ${formatEventValue(value)}`)
    .join(' ');
}

function normalizeSearchText(value) {
  return String(value || '').trim().toLowerCase();
}

function searchResult(stepIndex, label, detail) {
  return { step: stepIndex, label, detail };
}

function searchResultLabel(frame, query) {
  if (query === 'rejected' && (frame.accepted || []).some(value => value === false)) return 'rejected action';
  const action = (frame.actions || []).find(item => normalizeSearchText(item).includes(query));
  return action ? action : 'text match';
}

function searchResultDetail(frame) {
  const rejected = [];
  for (const [agentId, accepted] of (frame.accepted || []).entries()) {
    if (accepted === false) rejected.push(`agent${agentId}`);
  }
  if (rejected.length > 0) return `${rejected.join(', ')} rejected`;
  return (frame.actions || []).filter(Boolean).slice(0, 3).join(' | ');
}

function jumpSearch(direction) {
  if (!replay) return;
  if (els.jumpSearchInput.value.trim() !== searchState.query) updateSearchResults(false);
  if (searchState.results.length === 0) {
    renderSearchInfo();
    return;
  }
  const base = searchState.index >= 0 ? searchState.index : nearestSearchIndex(step);
  const next = (base + direction + searchState.results.length) % searchState.results.length;
  searchState.index = next;
  setStep(searchState.results[next].step);
}

function nearestSearchIndex(stepIndex) {
  if (!searchState.results.length) return -1;
  const exact = searchState.results.findIndex(result => result.step === stepIndex);
  if (exact >= 0) return exact;
  const next = searchState.results.findIndex(result => result.step > stepIndex);
  return next >= 0 ? next : 0;
}

function syncSearchIndexToStep() {
  if (!searchState.results.length) return;
  const exact = searchState.results.findIndex(result => result.step === step);
  if (exact >= 0) searchState.index = exact;
}

function renderSearchInfo() {
  if (!els.jumpSearchInfo) return;
  if (!replay) {
    els.jumpSearchInfo.textContent = 'Load a replay to search.';
    return;
  }
  if (!searchState.query && !hasActiveSearchFilters()) {
    els.jumpSearchInfo.textContent = 'Examples: step 120, rejected, agent0, boxA, Push, 28,3, phase or subgoal text.';
    return;
  }
  if (searchState.results.length === 0) {
    els.jumpSearchInfo.textContent = 'No matches.';
    return;
  }
  const index = searchState.index >= 0 ? searchState.index : nearestSearchIndex(step);
  const result = searchState.results[index];
  els.jumpSearchInfo.textContent = `${index + 1}/${searchState.results.length} | step ${result.step} | ${result.label}`;
}

function updateSegmentFromInputs() {
  if (!replay) return;
  const bounds = readSegmentInputs();
  segmentState.start = bounds.start;
  segmentState.end = bounds.end;
  segmentState.enabled = els.segmentEnabledInput.checked;
  segmentState.digest = '';
  segmentState.digestVisible = false;
  segmentState.source = 'Manual';
  segmentState.timelineIndex = null;
  segmentState.timelineKind = null;
  renderSegmentPanel();
}

function readSegmentInputs() {
  if (!replay) return { start: 0, end: 0 };
  const max = replay.frames.length - 1;
  let start = clamp(Number(els.segmentStartInput.value), 0, max);
  let end = clamp(Number(els.segmentEndInput.value), 0, max);
  if (start > end) [start, end] = [end, start];
  return { start: Math.round(start), end: Math.round(end) };
}

function setSegmentBoundary(boundary, value) {
  if (!replay) return;
  const max = replay.frames.length - 1;
  const bounded = Math.round(clamp(value, 0, max));
  if (boundary === 'start') {
    segmentState.start = bounded;
    if (segmentState.start > segmentState.end) segmentState.end = segmentState.start;
  } else {
    segmentState.end = bounded;
    if (segmentState.end < segmentState.start) segmentState.start = segmentState.end;
  }
  segmentState.enabled = true;
  segmentState.digest = '';
  segmentState.digestVisible = false;
  segmentState.source = 'Current Step';
  segmentState.timelineIndex = null;
  segmentState.timelineKind = null;
  renderSegmentPanel();
}

function setSegmentRange(start, end, source = '', options = {}) {
  if (!replay) return;
  const max = replay.frames.length - 1;
  let boundedStart = Math.round(clamp(Number(start), 0, max));
  let boundedEnd = Math.round(clamp(Number(end), 0, max));
  if (boundedStart > boundedEnd) [boundedStart, boundedEnd] = [boundedEnd, boundedStart];
  segmentState.start = boundedStart;
  segmentState.end = boundedEnd;
  segmentState.enabled = true;
  segmentState.digest = '';
  segmentState.digestVisible = false;
  segmentState.source = source;
  segmentState.timelineIndex = Number.isInteger(options.timelineIndex) ? options.timelineIndex : null;
  segmentState.timelineKind = options.timelineKind || null;
  renderSegmentPanel();
}

function playSegmentRange() {
  if (!replay) return;
  const bounds = segmentBounds();
  if (timer) {
    stopPlay();
    renderSegmentPanel();
    return;
  }
  segmentState.enabled = true;
  if (step < bounds.start || step >= bounds.end) setStep(bounds.start);
  renderSegmentPanel();
  startPlay();
}

function clearSegmentRange() {
  if (!replay) return;
  segmentState.enabled = false;
  segmentState.digest = '';
  segmentState.digestVisible = false;
  segmentState.source = '';
  segmentState.timelineIndex = null;
  segmentState.timelineKind = null;
  renderSegmentPanel();
}

function segmentBounds() {
  if (!replay) return { start: 0, end: 0 };
  const max = replay.frames.length - 1;
  const start = Math.round(clamp(segmentState.start, 0, max));
  const end = Math.round(clamp(segmentState.end, 0, max));
  return start <= end ? { start, end } : { start: end, end: start };
}

function activeSegmentBounds() {
  if (!replay || !segmentState.enabled) return null;
  return segmentBounds();
}

function selectedTimelineStageLabel() {
  if (!replay || !Number.isInteger(segmentState.timelineIndex)) return '';
  const segments = plannerLifecycleSpans();
  if (segmentState.timelineKind === 'chapter') {
    const chapters = debugTimelineChapters(annotateDebugSegments(segments));
    const index = segmentState.timelineIndex;
    if (!chapters[index]) return '';
    return ` | chapter ${index + 1}/${chapters.length}`;
  }
  const index = segmentState.timelineIndex;
  if (!segments[index]) return '';
  return ` | stage ${index + 1}/${segments.length}`;
}

function renderSegmentPanel() {
  if (!els.segmentInfo) return;
  if (!replay) {
    els.segmentStartInput.value = '0';
    els.segmentEndInput.value = '0';
    els.segmentInfo.textContent = 'Load a replay to create a segment.';
    els.segmentDigestOutput.hidden = true;
    els.segmentDigestBtn.textContent = 'Show Digest';
    return;
  }
  const bounds = segmentBounds();
  els.segmentEnabledInput.checked = segmentState.enabled;
  els.segmentStartInput.value = String(bounds.start);
  els.segmentEndInput.value = String(bounds.end);
  const length = bounds.end - bounds.start + 1;
  const activeText = segmentState.enabled ? 'playback limited' : 'full playback';
  const sourceText = segmentState.enabled && segmentState.source ? ` | from ${segmentState.source}` : '';
  const stageText = segmentState.enabled ? selectedTimelineStageLabel() : '';
  const marked = Boolean(segmentMarker());
  const markerText = marked ? ' | bookmarked' : '';
  els.segmentMarkBtn.textContent = marked ? 'Remove Bookmark' : 'Bookmark Range';
  els.segmentPlayBtn.textContent = timer && segmentState.enabled ? 'Pause Range' : 'Play Range';
  els.segmentInfo.textContent = `${activeText} | steps ${bounds.start}-${bounds.end} | ${length} frame${length === 1 ? '' : 's'}${sourceText}${stageText}${markerText}`;
  els.segmentDigestBtn.textContent = segmentState.digestVisible ? 'Hide Digest' : 'Show Digest';
  if (segmentState.digest && segmentState.digestVisible) {
    els.segmentDigestOutput.hidden = false;
    els.segmentDigestOutput.value = segmentState.digest;
  } else {
    els.segmentDigestOutput.hidden = true;
    els.segmentDigestOutput.value = '';
  }
}

function segmentMarker() {
  const bounds = segmentBounds();
  return markers.get(markerIdForSegment(bounds.start, bounds.end));
}

function segmentReplayObject() {
  const bounds = segmentBounds();
  const frames = replay.frames.slice(bounds.start, bounds.end + 1);
  const events = Array.isArray(replay.events)
    ? replay.events
        .filter(event => {
          const sourceStep = eventStep(event);
          return Number.isInteger(sourceStep) && sourceStep >= bounds.start && sourceStep <= bounds.end;
        })
        .map(event => ({ ...event, originalStep: eventStep(event), step: eventStep(event) - bounds.start }))
    : undefined;
  const segment = {
    sourceLevel: replay.level?.name || '',
    startStep: bounds.start,
    endStep: bounds.end,
    frameCount: frames.length,
    exportedAt: new Date().toISOString(),
    markers: markersInRange(bounds.start, bounds.end),
  };
  const out = {
    ...replay,
    generatedAt: new Date().toISOString(),
    segment,
    frames,
  };
  if (events) out.events = events;
  return out;
}

function exportSegmentJson() {
  if (!replay) return;
  const bounds = segmentBounds();
  const name = safeFilePart(replay.level?.name || 'replay');
  const text = JSON.stringify(segmentReplayObject(), null, 2);
  downloadText(`${name}__steps-${bounds.start}-${bounds.end}.json`, text, 'application/json');
}

function exportTimelineMetaJson() {
  if (!replay) return;
  const name = safeFilePart(replay.level?.name || 'replay');
  const spans = annotateDebugSegments(plannerLifecycleSpans());
  const chapters = debugTimelineChapters(spans);
  const diagnosis = timelineDiagnosisSummary(spans, chapters);
  const diagnostics = replay.diagnostics || {};
  const portfolio = timelinePortfolioSummary(diagnostics.portfolioAttempts || []);
  const rawFocus = replayDiagnosticFocus();
  const meta = {
    schema: 'aims-replay-lifecycle-meta-v1',
    exportedAt: new Date().toISOString(),
    source: {
      level: replay.level?.name || '',
      generatedAt: replay.generatedAt || '',
      replaySchema: replay.schema || '',
    },
    runContext: diagnostics.runContext || {},
    summary: {
      outcome: replay.summary?.outcome || 'unknown',
      executedSteps: replay.summary?.executedSteps ?? null,
      plannedSteps: replay.summary?.plannedSteps ?? null,
      frames: replay.frames?.length || 0,
      events: replayEventCount(),
      rawEvents: replayEventCount(),
      derivedActionEvents: derivedActionEventCount(),
      satisfiedBoxGoals: replay.summary?.satisfiedBoxGoals ?? null,
      totalBoxGoals: replay.summary?.totalBoxGoals ?? null,
    },
    diagnosticFocus: {
      source: rawFocus ? 'replay' : 'computed',
      headline: diagnosis.headline,
      detail: diagnosis.detail,
      firstFocus: diagnosis.firstFocus,
      finalFocus: diagnosis.finalFocus || '',
      hotspotCount: diagnosis.hotspotCount,
      focusEventCount: numberOrNull(rawFocus?.focusEventCount),
      topEventKinds: normalizeCountEntries(rawFocus?.topEventKinds),
      topReasons: normalizeCountEntries(rawFocus?.topReasons),
      topSubgoals: normalizeCountEntries(rawFocus?.topSubgoals),
      chapterCount: chapters.length,
      chapters: chapters.map(timelineMetaChapter),
      raw: rawFocus || null,
    },
    portfolio,
    lifecycle: {
      spanCount: spans.length,
      primaryTasks: primaryTaskBandsForTimeline(spans).map(timelineMetaPrimaryTaskBand),
      spans: spans.map((span, index) => timelineMetaSpan(span, index, spans)),
    },
  };
  downloadText(`${name}__lifecycle-meta.json`, JSON.stringify(meta, null, 2), 'application/json');
}

function renderSegmentDigest() {
  if (!replay) return;
  if (segmentState.digestVisible) {
    segmentState.digestVisible = false;
  } else {
    if (!segmentState.digest) segmentState.digest = buildSegmentDigest();
    segmentState.digestVisible = true;
  }
  renderSegmentPanel();
}

async function copySegmentDigest() {
  if (!replay) return;
  if (!segmentState.digest) segmentState.digest = buildSegmentDigest();
  segmentState.digestVisible = true;
  renderSegmentPanel();
  try {
    await navigator.clipboard.writeText(segmentState.digest);
    showError('Segment digest copied to clipboard.', 'Copied');
  } catch (_) {
    els.segmentDigestOutput.hidden = false;
    els.segmentDigestOutput.focus();
    els.segmentDigestOutput.select();
  }
}

function buildSegmentDigest() {
  const bounds = segmentBounds();
  const lines = [];
  lines.push(`# AIMS Replay Debug Digest`);
  lines.push('');
  lines.push(`Level: ${replay.level?.name || 'unknown'}`);
  lines.push(`Segment: steps ${bounds.start}-${bounds.end} (${bounds.end - bounds.start + 1} frames)`);
  lines.push(`Replay outcome: ${replay.summary?.outcome || 'unknown'}`);
  lines.push(`Generated at: ${replay.generatedAt || 'unknown'}`);
  const rawFocus = replayDiagnosticFocus();
  if (rawFocus) {
    lines.push(`Diagnostic focus: ${formatEventValue(rawFocus.headline) || 'recorded'} | ${formatEventValue(rawFocus.firstFocus) || 'no first focus'}`);
  }
  const codeVersion = replay.diagnostics?.runContext?.codeVersion || {};
  if (codeVersion.gitCommitShort) {
    lines.push(`Code version: ${codeVersion.gitBranch || 'git'}@${codeVersion.gitCommitShort}${codeVersion.gitDirty ? ' dirty' : ''}`);
  }
  lines.push('');
  lines.push(`## Bookmarks`);
  const selectedMarkers = markersInRange(bounds.start, bounds.end);
  if (selectedMarkers.length === 0) {
    lines.push('- None');
  } else {
    for (const marker of selectedMarkers) {
      lines.push(`- ${markerRangeText(marker)}: ${marker.label}${marker.note ? ` | note: ${marker.note}` : ''}`);
    }
  }
  lines.push('');
  lines.push(`## Rejected Actions`);
  const rejected = rejectedStepsInRange(bounds.start, bounds.end);
  if (rejected.length === 0) {
    lines.push('- None');
  } else {
    for (const item of rejected.slice(0, 80)) {
      lines.push(`- step ${item.step}: ${item.detail}`);
    }
  }
  lines.push('');
  lines.push(`## Agent Timeline`);
  const ids = timelineAgentId == null ? agentIds() : [timelineAgentId];
  for (const id of ids.slice(0, 6)) {
    lines.push(`### agent${id}`);
    const segments = agentTimelineSegments(id).filter(segment => segment.end >= bounds.start && segment.start <= bounds.end);
    for (const segment of segments.slice(0, 80)) {
      lines.push(`- ${Math.max(segment.start, bounds.start)}-${Math.min(segment.end, bounds.end)}: ${segment.label}${segment.detail ? ` | ${segment.detail}` : ''}${segment.rejected ? ' | rejected' : ''}`);
    }
    if (segments.length === 0) lines.push('- No timeline data');
  }
  lines.push('');
  lines.push(`## Highlight Events`);
  const highlights = highlightEventsInRange(bounds.start, bounds.end);
  if (highlights.length === 0) {
    lines.push('- None');
  } else {
    for (const item of highlights.slice(0, 120)) {
      lines.push(`- step ${item.step}: ${item.title}${item.message ? ` | ${item.message}` : ''}`);
    }
  }
  return lines.join('\n');
}

function rejectedStepsInRange(start, end) {
  const out = [];
  for (let i = start; i <= end; i++) {
    const frame = replay.frames[i];
    const rejected = [];
    for (const [agentId, accepted] of (frame.accepted || []).entries()) {
      if (accepted === false) rejected.push(`agent${agentId}:${frame.actions?.[agentId] || '?'}`);
    }
    if (rejected.length > 0) out.push({ step: i, detail: rejected.join(' | ') });
  }
  return out;
}

function highlightEventsInRange(start, end) {
  const out = [];
  for (let i = start; i <= end; i++) {
    for (const event of stepHighlights(replay.frames[i], i)) {
      const normalized = normalizeEvent(event);
      out.push({ step: i, title: normalized.title, message: normalized.message });
    }
  }
  return out;
}

function downloadText(filename, text, mimeType) {
  const blob = new Blob([text], { type: `${mimeType};charset=utf-8` });
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function safeFilePart(value) {
  return String(value || 'replay').replace(/[^\w.-]+/g, '_').replace(/^_+|_+$/g, '') || 'replay';
}

function populateAgentFocusMenu() {
  const ids = agentIds();
  const previous = focusedAgentId;
  els.agentMenuList.innerHTML = '';
  focusedAgentId = ids.includes(previous) ? previous : null;

  for (const id of ids) {
    els.agentMenuList.appendChild(agentMenuItem(String(id), `agent${id}`, 'Pin, center, and follow this agent', focusedAgentId === id));
  }
  els.agentMenuBtn.hidden = false;
  setDisabled(els.agentMenuBtn, ids.length === 0);
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
  if (!replay || els.agentMenuBtn.hidden || els.agentMenuBtn.disabled) return;
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
  if (!replay) return;
  const nextAgentId = Number.isInteger(agentId) ? agentId : null;
  const agentChanged = focusedAgentId !== nextAgentId;
  focusedAgentId = nextAgentId;
  if (focusedAgentId != null) {
    setLockView(true, false);
    timelineAgentId = focusedAgentId;
    syncTimelineAgentSelect();
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
  renderAgentTimeline();
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

function populateTimelineAgentSelect() {
  const ids = agentIds();
  els.timelineAgentSelect.innerHTML = '';
  const overviewOption = document.createElement('option');
  overviewOption.value = '';
  overviewOption.textContent = 'Overview';
  els.timelineAgentSelect.appendChild(overviewOption);
  if (ids.length === 0) {
    timelineAgentId = null;
    timelineMode = 'overview';
    els.timelineAgentSelect.disabled = true;
    renderAgentTimeline();
    renderMiniTimelinePanel();
    syncReplayControlAvailability();
    return;
  }
  els.timelineAgentSelect.disabled = false;
  if (timelineMode === 'overview') {
    timelineAgentId = null;
  } else if (!ids.includes(timelineAgentId)) {
    timelineAgentId = focusedAgentId != null && ids.includes(focusedAgentId) ? focusedAgentId : ids[0];
  }
  for (const id of ids) {
    const option = document.createElement('option');
    option.value = String(id);
    option.textContent = `agent${id}`;
    els.timelineAgentSelect.appendChild(option);
  }
  syncTimelineAgentSelect();
  renderAgentTimeline();
  renderMiniTimelinePanel();
}

function syncTimelineAgentSelect() {
  if (els.timelineAgentSelect) {
    els.timelineAgentSelect.value = timelineMode === 'overview' || timelineAgentId == null ? '' : String(timelineAgentId);
  }
  els.timelineBtn.textContent = 'Timeline';
  els.timelineOverviewBtn.classList.toggle('active', timelineMode === 'overview');
}

function toggleTimelinePanel() {
  if (!replay) return;
  const shouldOpen = els.timelinePanel.hidden;
  if (shouldOpen) {
    timelineMode = 'overview';
    timelineAgentId = null;
    timelineSelectionKind = 'current';
    timelineSelectedSegmentIndex = null;
    timelineSelectedChapterIndex = null;
    timelineScrollLeft = 0;
    timelineAutoFollowDisabled = false;
  }
  els.timelinePanel.hidden = !shouldOpen;
  els.timelineBtn.classList.toggle('active', shouldOpen);
  els.timelineBtn.setAttribute('aria-expanded', shouldOpen ? 'true' : 'false');
  if (shouldOpen) renderAgentTimeline();
}

function closeTimelinePanel() {
  if (!els.timelinePanel) return;
  els.timelinePanel.hidden = true;
  els.timelineBtn.classList.remove('active');
  els.timelineBtn.setAttribute('aria-expanded', 'false');
}

function toggleMiniTimelinePanel() {
  if (!replay || !els.miniTimelinePanel) return;
  const shouldOpen = els.miniTimelinePanel.hidden;
  if (shouldOpen) closeTimelinePanel();
  els.miniTimelinePanel.hidden = !shouldOpen;
  els.miniTimelineBtn?.classList.toggle('active', shouldOpen);
  els.miniTimelineBtn?.setAttribute('aria-expanded', shouldOpen ? 'true' : 'false');
  if (shouldOpen) renderMiniTimelinePanel();
}

function closeMiniTimelinePanel() {
  if (!els.miniTimelinePanel) return;
  els.miniTimelinePanel.hidden = true;
  els.miniTimelineBtn?.classList.remove('active');
  els.miniTimelineBtn?.setAttribute('aria-expanded', 'false');
}

function renderMiniTimelinePanel(options = {}) {
  if (!els.miniTimelinePanel || !els.miniTimelineBody) return;
  syncMiniTimelineProgress();
  if (!replay) {
    els.miniTimelineBody.textContent = 'Load a replay to inspect lifecycle stages.';
    return;
  }
  syncMiniTimelinePlayButton();
  if (els.miniTimelinePanel.hidden) return;
  const segments = annotateDebugSegments(plannerLifecycleSpans());
  const currentScroll = els.miniTimelineBody.querySelector('.timelineScroll');
  if (currentScroll) {
    miniTimelineScrollLeft = currentScroll.__timelineRestorePending
      ? (currentScroll.scrollLeft || currentScroll.__timelineRestoreLeft || 0)
      : currentScroll.scrollLeft;
    currentScroll.__timelineIgnoreScroll = true;
  }
  els.miniTimelineBody.innerHTML = '';
  els.miniTimelineBody.appendChild(renderTimelineMap(segments, {
    compact: true,
    progress: true,
    includeHeader: false,
    followCurrent: true,
    lockCurrent: Boolean(options.lockCurrent),
    alignCurrentEnd: Boolean(options.alignCurrentEnd),
  }));
}

function syncMiniTimelineProgress() {
  if (!els.miniTimelineProgressFill || !els.miniTimelineProgressText) return;
  const maxStep = replay ? Math.max(0, replay.frames.length - 1) : 0;
  const ratio = maxStep > 0 ? clamp(step / maxStep, 0, 1) : 0;
  els.miniTimelineProgressFill.style.width = `${Math.round(ratio * 1000) / 10}%`;
  els.miniTimelineProgressText.textContent = replay ? `step ${step} / ${maxStep}` : 'step 0 / 0';
}

function syncMiniTimelinePlayButton() {
  if (!els.miniTimelinePlayBtn) return;
  const isPlaying = Boolean(timer);
  els.miniTimelinePlayBtn.classList.toggle('playing', isPlaying);
  els.miniTimelinePlayBtn.title = isPlaying ? 'Pause replay' : 'Play replay';
  els.miniTimelinePlayBtn.setAttribute('aria-label', isPlaying ? 'Pause replay' : 'Play replay');
  els.miniTimelinePlayBtn.setAttribute('aria-pressed', isPlaying ? 'true' : 'false');
}

function renderAgentTimeline() {
  if (!els.agentTimeline) return;
  const wasOverview = els.agentTimeline.classList.contains('lifecycleOverview');
  const preservePanelScroll =
    (timelineMode === 'overview' && wasOverview)
    || (timelineMode === 'agent' && !wasOverview && els.agentTimeline.childElementCount > 0);
  const savedPanelScrollTop = preservePanelScroll
    ? (els.agentTimeline.__timelinePanelRestorePending ? els.agentTimeline.__timelinePanelRestoreTop || 0 : els.agentTimeline.scrollTop)
    : 0;
  const savedPanelScrollLeft = preservePanelScroll
    ? (els.agentTimeline.__timelinePanelRestorePending ? els.agentTimeline.__timelinePanelRestoreLeft || 0 : els.agentTimeline.scrollLeft)
    : 0;
  const currentScroll = els.agentTimeline.querySelector('.timelineScroll');
  if (currentScroll) {
    timelineScrollLeft = currentScroll.__timelineRestorePending
      ? (currentScroll.scrollLeft || currentScroll.__timelineRestoreLeft || 0)
      : currentScroll.scrollLeft;
    currentScroll.__timelineIgnoreScroll = true;
  }
  els.agentTimeline.innerHTML = '';
  syncTimelineAgentSelect();
  els.agentTimeline.className = timelineMode === 'overview' ? 'agentTimeline lifecycleOverview' : 'agentTimeline';
  if (!replay) {
    if (els.timelineKicker) els.timelineKicker.textContent = 'Replay Timeline';
    if (els.timelineTitle) els.timelineTitle.textContent = 'Timeline';
    els.agentTimeline.textContent = 'Load a replay to inspect lifecycle metadata.';
    return;
  }
  if (timelineMode === 'overview') {
    renderLifecycleTimeline();
    if (preservePanelScroll) restoreAgentTimelinePanelScroll(savedPanelScrollTop, savedPanelScrollLeft, 'overview');
    else resetAgentTimelinePanelScroll('overview');
    return;
  }
  renderAgentTimelineDetails();
  if (preservePanelScroll) restoreAgentTimelinePanelScroll(savedPanelScrollTop, savedPanelScrollLeft, 'agent');
  else resetAgentTimelinePanelScroll('agent');
}

function restoreAgentTimelinePanelScroll(top, left, expectedMode) {
  els.agentTimeline.__timelinePanelRestorePending = true;
  els.agentTimeline.__timelinePanelRestoreTop = top;
  els.agentTimeline.__timelinePanelRestoreLeft = left;
  if (timelineMode === expectedMode) {
    els.agentTimeline.scrollTop = top;
    els.agentTimeline.scrollLeft = left;
  }
  els.agentTimeline.__timelinePanelRestorePending = false;
}

function resetAgentTimelinePanelScroll(expectedMode) {
  if (!els.agentTimeline || timelineMode !== expectedMode) return;
  els.agentTimeline.__timelinePanelRestorePending = false;
  els.agentTimeline.scrollTop = 0;
  els.agentTimeline.scrollLeft = 0;
}

function selectTimelineAgent(agentId) {
  if (!Number.isInteger(agentId)) return;
  timelineAgentId = agentId;
  timelineMode = 'agent';
  syncTimelineAgentSelect();
  renderAgentTimeline();
}

function renderAgentTimelineDetails() {
  if (timelineAgentId == null) {
    timelineMode = 'overview';
    renderLifecycleTimeline();
    return;
  }
  if (els.timelineKicker) els.timelineKicker.textContent = 'Agent Timeline';
  if (els.timelineTitle) els.timelineTitle.textContent = `agent${timelineAgentId}`;
  const segments = agentTimelineSegments(timelineAgentId);
  if (segments.length === 0) {
    els.agentTimeline.textContent = `No timeline data for agent${timelineAgentId}.`;
    return;
  }
  for (const segment of segments) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'timelineSegment';
    if (step >= segment.start && step <= segment.end) button.classList.add('active');
    if (segment.rejected) button.classList.add('rejected');
    if (segmentHasMarker(segment)) button.classList.add('marked');
    button.dataset.step = String(segment.start);
    button.dataset.segmentStart = String(segment.start);
    button.dataset.segmentEnd = String(segment.end);
    const range = segment.start === segment.end ? `step ${segment.start}` : `${segment.start}-${segment.end}`;
    button.innerHTML = `<span>${escapeHtml(range)}</span><strong>${escapeHtml(segment.label)}</strong><small>${escapeHtml(segment.detail)}</small>`;
    els.agentTimeline.appendChild(button);
  }
}

function renderLifecycleTimeline() {
  if (els.timelineKicker) els.timelineKicker.textContent = 'Replay Timeline';
  const segments = annotateDebugSegments(plannerLifecycleSpans());
  const chapters = debugTimelineChapters(segments);
  if (els.timelineTitle) els.timelineTitle.textContent = 'Timeline';
  const activeIndex = segments.findIndex(item => step >= item.start && step <= item.end);
  const selectedSegmentIndex = validTimelineSegmentIndex(timelineSelectedSegmentIndex, segments)
    ? timelineSelectedSegmentIndex
    : activeIndex;
  lifecycleSelection = selectedSegmentIndex >= 0 ? selectedSegmentIndex : -1;
  const activeChapterIndex = validTimelineChapterIndex(timelineSelectedChapterIndex, chapters) && timelineSelectionKind === 'chapter'
    ? timelineSelectedChapterIndex
    : activeDebugChapterIndex(chapters, lifecycleSelection, step);
  const overview = document.createElement('div');
  overview.className = 'timelineOverviewStack';
  overview.appendChild(renderPortfolioAttemptsCard());
  overview.appendChild(renderTimelineMap(segments, { followCurrent: Boolean(timer) || !timelineAutoFollowDisabled }));
  overview.appendChild(renderTimelineSelectionMessage(segments, chapters, activeChapterIndex, step));
  els.agentTimeline.appendChild(overview);
}

function timelineDebugSummary(segments, diagnosis) {
  const summary = replay.summary || {};
  const transactions = segments.flatMap(segment => timelineNativeTransactions(segment));
  const candidates = segments.flatMap(segment => timelineMetaCandidateSummary(segment));
  const decisionContext = subgoalDecisionContext();
  const candidateGroups = timelineCandidateProblemGroups(candidates);
  const riskyTransactions = transactions
    .filter(timelineIsRiskyTransaction)
    .sort((a, b) => timelineIssueStep(a) - timelineIssueStep(b));
  const primaryTransaction = riskyTransactions[0] || null;
  const primaryCandidate = timelinePrimaryCandidateGroup(primaryTransaction, candidateGroups);
  const primaryDecision = timelinePrimarySubgoalDecision(decisionContext, primaryTransaction, primaryCandidate);
  const route = timelinePlannerRouteSummary(transactions, decisionContext);
  const initialOrdering = route.initialOrdering || decisionContext.orderings[0] || null;
  return {
    headline: timelineDebugHeadline(summary, transactions, candidateGroups),
    startHere: timelineStartHereText(primaryTransaction, primaryCandidate, diagnosis),
    retryPoints: timelineRetryPointsText(candidateGroups),
    route,
    choice: timelineDebugChoiceText(primaryDecision),
    health: timelineDebugHealthText(transactions, candidates, decisionContext.selections, decisionContext.orderings),
    orderInsight: timelineOrderInsightText(initialOrdering, route),
    runtimeInsight: timelineRuntimeInsightText(route),
    attention: timelineAttentionText(primaryTransaction, candidateGroups),
    evidenceHint: timelineEvidenceHintText(initialOrdering, route),
    primaryTransaction,
    primaryCandidate,
    primaryDecision,
    riskyTransactions,
    candidateGroups,
    transactions,
    candidates,
    subgoalSelections: decisionContext.selections,
    subgoalOrderings: decisionContext.orderings,
  };
}

function timelineDebugHeadline(summary, transactions, candidateGroups) {
  const outcome = titleCase(summary.outcome || 'unknown');
  const steps = summary.executedSteps ?? Math.max(0, replay.frames.length - 1);
  const goals = `${summary.satisfiedBoxGoals ?? 0}/${summary.totalBoxGoals ?? 0} box goals`;
  const rollbackCount = transactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('roll')).length;
  const retryPoints = candidateGroups.filter(group => group.retryCount > 0).length;
  const issues = [];
  if (retryPoints) issues.push(`${retryPoints} box-selection retry point${retryPoints === 1 ? '' : 's'}`);
  if (rollbackCount) issues.push(`${rollbackCount} rollback${rollbackCount === 1 ? '' : 's'}`);
  return `${outcome} in ${steps} steps | ${goals}${issues.length ? ` | ${issues.join(', ')}` : ''}`;
}

function timelineStartHereText(transaction, candidateGroup, diagnosis) {
  if (transaction && candidateGroup) {
    const txReason = transaction.rollbackReason || transaction.reason || transaction.commitStatus || 'review transaction';
    return `Step ${timelineIssueStep(candidateGroup)} | ${timelineCandidateTargetText(candidateGroup)} | ${transaction.transactionId || 'transaction'} ${timelineTransactionRiskLabel(transaction)}: ${txReason}`;
  }
  if (transaction) {
    const txReason = transaction.rollbackReason || transaction.reason || transaction.commitStatus || 'review transaction';
    return `Step ${timelineIssueStep(transaction)} | ${transaction.transactionId || 'transaction'} ${timelineTransactionRiskLabel(transaction)}: ${txReason}`;
  }
  if (candidateGroup) {
    return `Step ${timelineIssueStep(candidateGroup)} | ${timelineCandidateTargetText(candidateGroup)} | ${timelineCandidateOutcomeText(candidateGroup)}: ${timelineCandidateReasonsText(candidateGroup)}`;
  }
  return diagnosis.firstFocus && diagnosis.firstFocus !== 'none'
    ? diagnosis.firstFocus
    : 'No high-signal planner issue detected.';
}

function timelineRetryPointsText(candidateGroups) {
  const retryGroups = candidateGroups.filter(group => group.retryCount > 0);
  if (retryGroups.length === 0) return 'No box-selection retry points.';
  const shown = retryGroups.slice(0, 2)
    .map(group => `step ${timelineIssueStep(group)} ${timelineCandidateTargetText(group)}`);
  const overflow = retryGroups.length > shown.length ? ` +${retryGroups.length - shown.length}` : '';
  return `${retryGroups.length} point${retryGroups.length === 1 ? '' : 's'}: ${shown.join('; ')}${overflow}`;
}

function timelineDebugHealthText(transactions, candidates, selections = [], orderings = []) {
  const txTotal = transactions.length;
  const committed = transactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('commit')).length;
  const rolledBack = transactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('roll')).length;
  const selected = candidates.filter(item => String(item.verdict || '').toLowerCase().includes('select')).length;
  const failed = candidates.filter(item => String(item.verdict || '').toLowerCase().includes('fail')).length;
  const retry = candidates.filter(item => String(item.verdict || '').toLowerCase().includes('retry')).length;
  const txPart = txTotal === 0
    ? 'Tx: none recorded'
    : `Tx ${txTotal}: ${committed} committed${rolledBack ? `, ${rolledBack} rolled back` : ''}`;
  const candidatePart = candidates.length === 0
    ? 'Box selection: none recorded'
    : `Box selection ${candidates.length}: ${selected} selected, ${retry} retry, ${failed} failed`;
  const subgoalPart = selections.length === 0 && orderings.length === 0
    ? ''
    : `Subgoal choices ${selections.length}: ${orderings.length} order snapshot${orderings.length === 1 ? '' : 's'}`;
  return [txPart, candidatePart, subgoalPart].filter(Boolean).join(' | ');
}

function timelineDebugDetailsHtml(debugSummary) {
  const blocks = [];
  blocks.push(timelineDebugOrderingHtml(debugSummary.route.initialOrdering));
  blocks.push(timelineDebugRuntimeHtml(debugSummary.route));
  blocks.push(timelineDebugAttentionHtml(debugSummary));
  return `<div class="timelineDebugDetails">${blocks.join('')}</div>`;
}

function timelineDebugChoiceText(decision) {
  const selection = decision?.selection || null;
  const ordering = decision?.ordering || null;
  if (!selection && !ordering) return 'No subgoal choice evidence recorded.';
  const selectionText = selection
    ? `${timelineReadableSubgoalLabel(selection.subgoal)} selected at step ${selection.step}${selection.selectedRank != null ? `, rank ${selection.selectedRank}` : ''}${selection.eligibleCount != null ? ` of ${selection.eligibleCount} eligible` : ''}`
    : '';
  const orderingText = ordering
    ? `${timelineReadableOrderingMode(ordering.orderingMode)} order${ordering.goalDependencyEdges != null ? `, ${ordering.goalDependencyEdges} dependency edges` : ''}`
    : '';
  return [selectionText, orderingText].filter(Boolean).join(' | ');
}

function timelineOrderInsightText(ordering, route) {
  if (!ordering) return 'No ordering snapshot recorded.';
  const pairReasons = parseAdjacentPairRationales(ordering.adjacentPairRationaleSample);
  const pairText = pairReasons.slice(0, 2).map(timelineAdjacentReasonSummary).filter(Boolean).join(' | ');
  if (pairText) return pairText;
  const edges = ordering.goalDependencyEdges ?? extractTopologyCount(ordering.topologyBasis, 'goalDependencyEdges');
  const transit = ordering.transitDeferredToTail ?? extractTopologyCount(ordering.topologyBasis, 'transitProfiles');
  const first = parseSubgoalSampleEntries(ordering.orderSample)[0];
  const leader = first ? timelineReadableSubgoalLabel(first.subgoal) : '';
  const parts = [
    'dependency-first topological order',
    edges != null ? `${edges} hard-block edges` : '',
    transit ? `${transit} transit goals delayed` : '',
    leader ? `${leader} first` : '',
  ].filter(Boolean);
  const txText = route?.transactions?.length ? `; runtime made ${route.transactions.length} tx` : '';
  return `${parts.join(' | ')}${txText}`;
}

function timelineRuntimeInsightText(route) {
  const transactions = route?.transactions || [];
  if (transactions.length === 0) return 'No planner transaction route recorded.';
  const support = transactions.filter(tx => tx.supportKind);
  const rolled = transactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('roll'));
  const repeatedTasks = timelineRepeatedTransactionTasks(transactions);
  const parts = [];
  if (support.length) parts.push(`${support.length} support split${support.length === 1 ? '' : 's'}`);
  if (rolled.length) parts.push(`${rolled.length} rollback${rolled.length === 1 ? '' : 's'} (${timelineReadableSubgoalLabel(rolled[0].subgoal)}: ${timelineReadableReason(rolled[0].rollbackReason || rolled[0].reason)})`);
  if (repeatedTasks.length) parts.push(`${repeatedTasks.length} repeated task${repeatedTasks.length === 1 ? '' : 's'} after support/rollback`);
  if (parts.length === 0) parts.push('route followed the order without rollback');
  return parts.join(' | ');
}

function timelineAttentionText(transaction, candidateGroups) {
  const parts = [];
  if (transaction) {
    parts.push(`${transaction.transactionId || 'tx'} ${timelineReadableTransactionStatus(transaction)} on ${timelineReadableSubgoalLabel(transaction.subgoal)}: ${timelineReadableReason(transaction.rollbackReason || transaction.reason || transaction.commitStatus)}`);
  }
  const retryGroups = (candidateGroups || []).filter(group => group.retryCount > 0);
  if (retryGroups.length) {
    const shown = retryGroups.slice(0, 2).map(group => `step ${timelineIssueStep(group)} ${timelineCandidateTargetText(group)}`);
    parts.push(`${retryGroups.length} retry point${retryGroups.length === 1 ? '' : 's'} (${shown.join('; ')})`);
  }
  return parts.join(' | ') || 'No rollback or retry hotspot recorded.';
}

function timelineEvidenceHintText(ordering, route) {
  if (!ordering) return 'Open stage details for raw events.';
  const rationaleCount = splitNumberedEntries(ordering.adjacentPairRationaleSample).length;
  const dependencyCount = parseDependencyEdgeEntries(ordering.dependencyEdges).length
    || ordering.goalDependencyEdges
    || extractTopologyCount(ordering.topologyBasis, 'explainedDependencyEdges');
  const txCount = route?.transactions?.length || 0;
  return [
    rationaleCount ? `${rationaleCount} adjacent-pair reasons` : '',
    dependencyCount ? `${dependencyCount} dependency edges` : '',
    txCount ? `${txCount} tx outcomes` : '',
  ].filter(Boolean).join(' | ') || 'ordering summary recorded';
}

function timelinePlannerRouteSummary(transactions, decisionContext) {
  const routeTransactions = uniquePlannerTransactions(transactions);
  const initialOrdering = (decisionContext?.orderings || [])[0] || null;
  const initialGoals = parseSubgoalSampleEntries(initialOrdering?.orderSample);
  const committed = routeTransactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('commit')).length;
  const rolledBack = routeTransactions.filter(tx => String(tx.commitStatus || '').toLowerCase().includes('roll')).length;
  const initialGoalCount = initialGoals.length || initialOrdering?.orderedSubgoalCount || 0;
  const text = routeTransactions.length
    ? `Initial order ${initialGoalCount || '?'} goals | runtime produced ${routeTransactions.length} planner transactions: ${committed} committed${rolledBack ? `, ${rolledBack} rolled back` : ''}`
    : initialGoalCount
      ? `Initial order ${initialGoalCount} goals | no planner transactions recorded`
      : 'No dependency-order route recorded.';
  return {
    text,
    transactions: routeTransactions,
    initialOrdering,
    initialGoals,
    selectionCount: (decisionContext?.selections || []).length,
    orderingCount: (decisionContext?.orderings || []).length,
  };
}

function uniquePlannerTransactions(transactions) {
  const byKey = new Map();
  for (const tx of transactions || []) {
    const key = tx.transactionId
      || [tx.step, tx.planStartStep, tx.planEndStep, tx.subgoal, tx.commitStatus].join('|');
    if (!byKey.has(key)) byKey.set(key, tx);
  }
  return [...byKey.values()].sort((a, b) =>
    (a.planStartStep ?? a.step ?? 0) - (b.planStartStep ?? b.step ?? 0)
    || (a.planEndStep ?? a.step ?? 0) - (b.planEndStep ?? b.step ?? 0)
    || String(a.transactionId || '').localeCompare(String(b.transactionId || '')));
}

function timelineDebugRouteHtml(route) {
  if (!route) return '';
  const fields = [];
  if (route.initialOrdering) {
    fields.push([
      'Initial dependency order',
      timelineFriendlySubgoalSample(route.initialOrdering.orderSample, { limit: Math.max(9, route.initialGoals?.length || 0) }),
    ]);
  }
  fields.push([
    'Route shape',
    `${route.selectionCount || 0} subgoal selection checkpoints | ${route.orderingCount || 0} order snapshots | ${route.transactions.length} planner transactions`,
  ]);
  for (const tx of route.transactions) {
    fields.push([tx.transactionId || `tx@${timelineIssueStep(tx)}`, timelinePlannerTransactionRouteText(tx)]);
  }
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>Transaction route</strong>
        <span>${escapeHtml(route.text)}</span>
      </div>
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelineDebugOrderingHtml(ordering) {
  if (!ordering) return '';
  const fields = [];
  const pairReasons = parseAdjacentPairRationales(ordering.adjacentPairRationaleSample).slice(0, 5);
  const pairRows = pairReasons.length ? `
    <div class="timelineReasonList">
      ${pairReasons.map(timelineAdjacentReasonRowHtml).join('')}
    </div>
  ` : '';
  fields.push(['Initial order', timelineCompactOrderSequence(ordering.orderSample, 9)]);
  const edgeText = timelineDependencyEdgeExamplesText(ordering);
  if (edgeText) fields.push(['Hard blocks', edgeText]);
  const transit = timelineTransitTailText(ordering);
  if (transit) fields.push(['Tail moved', transit]);
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>Order reasoning</strong>
        <span>${escapeHtml(timelineOrderInsightText(ordering, null))}</span>
      </div>
      ${pairRows}
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelineDebugRuntimeHtml(route) {
  if (!route) return '';
  const fields = [];
  fields.push(['Shape', route.text]);
  const support = (route.transactions || []).filter(tx => tx.supportKind).slice(0, 3);
  if (support.length) {
    fields.push(['Support', support.map(tx => `${tx.transactionId}: ${timelineReadableSubgoalLabel(tx.subgoal)} needed ${timelineReadableReason(tx.supportKind)}`).join(' | ')]);
  }
  const rolled = (route.transactions || []).filter(tx => String(tx.commitStatus || '').toLowerCase().includes('roll')).slice(0, 3);
  if (rolled.length) {
    fields.push(['Rollback', rolled.map(tx => `${tx.transactionId}: ${timelineReadableSubgoalLabel(tx.subgoal)} -> ${timelineReadableReason(tx.rollbackReason || tx.reason)}`).join(' | ')]);
  }
  const repeated = timelineRepeatedTransactionTasks(route.transactions || []).slice(0, 3);
  if (repeated.length) fields.push(['Repeated tasks', repeated.map(item => `${item.label} (${item.count} tx)`).join(' | ')]);
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>Runtime changes</strong>
        <span>${escapeHtml(timelineRuntimeInsightText(route))}</span>
      </div>
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelineDebugAttentionHtml(debugSummary) {
  const fields = [];
  if (debugSummary.primaryTransaction) {
    fields.push(['Primary risk', timelinePlannerTransactionRouteText(debugSummary.primaryTransaction)]);
  }
  const retryGroups = (debugSummary.candidateGroups || []).filter(group => group.retryCount > 0).slice(0, 3);
  if (retryGroups.length) {
    fields.push(['Retry points', retryGroups.map(group => `${timelineCandidateTargetText(group)} at step ${timelineIssueStep(group)}: ${timelineCandidateReasonsText(group)}`).join(' | ')]);
  }
  if (!fields.length) fields.push(['Status', 'Solved without recorded rollback or retry hotspot.']);
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>Debug focus</strong>
        <span>${escapeHtml(debugSummary.attention)}</span>
      </div>
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelinePlannerTransactionRouteText(tx) {
  if (!tx) return '';
  const task = timelineReadableSubgoalLabel(tx.taskLabel || tx.subgoal || tx.rawEvent?.ownerSubgoal || '');
  const status = timelineReadableTransactionStatus(tx);
  const span = tx.planStartStep != null && tx.planEndStep != null
    ? `steps ${tx.planStartStep}-${tx.planEndStep}`
    : tx.step != null ? `step ${tx.step}` : '';
  const reason = timelineReadableReason(tx.rollbackReason || tx.reason || '');
  const support = timelineReadableReason(tx.supportKind || '');
  return [
    task,
    status,
    span,
    reason ? `reason: ${reason}` : '',
    support ? `support: ${support}` : '',
  ].filter(Boolean).join(' | ');
}

function timelineReadableTransactionStatus(tx) {
  const status = String(tx?.commitStatus || '').toLowerCase();
  if (status.includes('roll')) return 'rolled back';
  if (status.includes('commit')) return 'committed';
  if (status.includes('fail')) return 'failed';
  if (status.includes('reject')) return 'rejected';
  return status || 'transaction';
}

function timelineReadableReason(value) {
  const text = formatEventValue(value).trim();
  if (!text) return '';
  return text.replace(/[-_]+/g, ' ');
}

function timelineRepeatedTransactionTasks(transactions) {
  const counts = new Map();
  for (const tx of transactions || []) {
    const label = timelineReadableSubgoalLabel(tx.subgoal || tx.taskLabel || tx.rawEvent?.ownerSubgoal || '');
    if (!label) continue;
    counts.set(label, (counts.get(label) || 0) + 1);
  }
  return [...counts.entries()]
    .filter(([, count]) => count > 1)
    .map(([label, count]) => ({ label, count }));
}

function extractTopologyCount(text, key) {
  const match = formatEventValue(text).match(new RegExp(`${key}=(-?\\d+)`));
  return match ? Number(match[1]) : null;
}

function timelineCompactOrderSequence(value, limit = 9) {
  const entries = parseSubgoalSampleEntries(value);
  if (entries.length === 0) return formatEventValue(value);
  const shown = entries.slice(0, limit)
    .map(entry => `${entry.rank}. ${timelineReadableSubgoalLabel(entry.subgoal)}`);
  const overflow = entries.length > shown.length ? ` | +${entries.length - shown.length} more` : '';
  return `${shown.join(' | ')}${overflow}`;
}

function splitNumberedEntries(value) {
  const text = formatEventValue(value);
  if (!text) return [];
  const matches = [...text.matchAll(/(?:^|;\s)(\d+):/g)];
  if (matches.length === 0) return [];
  return matches.map((match, index) => {
    const start = match.index + (match[0].startsWith(';') ? 2 : 0);
    const end = matches[index + 1] ? matches[index + 1].index : text.length;
    return text.slice(start, end).trim().replace(/;\s*$/, '');
  }).filter(Boolean);
}

function parseAdjacentPairRationales(value) {
  return splitNumberedEntries(value).map(entry => {
    const match = entry.match(/^(\d+):(.+?) before (.+?) because (.+?)(?:; before=|$)/);
    if (!match) return null;
    return {
      rank: match[1],
      before: match[2],
      after: match[3],
      reason: match[4],
    };
  }).filter(Boolean);
}

function timelineAdjacentReasonSummary(item) {
  if (!item) return '';
  const before = timelineReadableSubgoalLabel(item.before);
  const after = timelineReadableSubgoalLabel(item.after);
  const evidence = timelineAdjacentReasonEvidence(item.reason);
  return `${before} before ${after}${evidence ? `: ${evidence}` : ''}`;
}

function timelineAdjacentReasonRowHtml(item) {
  const before = timelineReadableSubgoalLabel(item.before);
  const after = timelineReadableSubgoalLabel(item.after);
  const explanation = timelineAdjacentReasonExplanation(item.reason);
  const evidence = timelineAdjacentReasonEvidence(item.reason);
  return `
    <div class="timelineReasonRow">
      <strong>${escapeHtml(before)} before ${escapeHtml(after)}</strong>
      <span>${escapeHtml(explanation)}</span>
      ${evidence ? `<small>${escapeHtml(evidence)}</small>` : ''}
    </div>
  `;
}

function timelineAdjacentReasonEvidence(reason) {
  const text = String(reason || '');
  const comparison = timelineComparisonText(text);
  const lower = text.toLowerCase();
  if (lower.includes('path-conflict')) return comparison ? `path-conflict ${comparison}` : 'higher path-conflict';
  if (lower.includes('importance tier')) return comparison ? `importance ${comparison}` : 'higher importance';
  if (lower.includes('lower crossing-count')) return comparison ? `crossings ${comparison.replace('>', '<')}` : 'lower crossing count';
  if (lower.includes('equal crossing-count')) return 'same crossing count';
  const direct = text.match(/direct-dependency:\s*([^\s]+)\s+depends on\s+([^\s(]+)/i);
  if (direct) return `${direct[1]} depends on ${direct[2]}`;
  return timelineReadableReason(text);
}

function timelineAdjacentReasonExplanation(reason) {
  const lower = String(reason || '').toLowerCase();
  if (lower.includes('path-conflict')) return 'The earlier goal blocks more future traffic.';
  if (lower.includes('importance tier')) return 'The earlier goal unblocks more later goals.';
  if (lower.includes('direct-dependency')) return 'The later goal cannot be safely filled before the earlier goal.';
  if (lower.includes('lower crossing-count')) return 'Inside the transit tail, fewer crossings go first.';
  if (lower.includes('equal crossing-count')) return 'Same transit pressure, so the previous order is kept.';
  return timelineReadableReason(reason);
}

function timelineComparisonText(text) {
  const match = String(text || '').match(/(-?\d+(?:\.\d+)?)\s+vs\s+(-?\d+(?:\.\d+)?)/);
  if (!match) return '';
  return `${match[1]} > ${match[2]}`;
}

function timelineTransitTailText(ordering) {
  const profiles = parseTransitProfileDetails(ordering?.transitProfileDetails);
  const chokepoints = profiles.filter(item => item.profile === 'CHOKEPOINT');
  const deferred = ordering?.transitDeferredToTail ?? chokepoints.length;
  if (!deferred && chokepoints.length === 0) return '';
  const shown = chokepoints
    .sort((a, b) => a.crossings - b.crossings || a.goal.localeCompare(b.goal))
    .slice(0, 4)
    .map(item => `${timelineReadableSubgoalLabel(item.goal)} crosses ${item.crossings}`);
  return `${deferred} transit/chokepoint goals delayed${shown.length ? `: ${shown.join('; ')}` : ''}`;
}

function parseTransitProfileDetails(value) {
  const text = formatEventValue(value);
  if (!text) return [];
  return text.split(/\s*;\s*/).map(part => {
    const match = part.match(/^(.+?) profile=([A-Z_]+) crossings=(\d+)/);
    return match ? { goal: match[1], profile: match[2], crossings: Number(match[3]) || 0 } : null;
  }).filter(Boolean);
}

function timelineDependencyEdgeExamplesText(ordering) {
  const edges = parseDependencyEdgeEntries(ordering?.dependencyEdges);
  if (!edges.length) return '';
  const total = ordering.goalDependencyEdges ?? edges.length;
  const examples = edges.slice(0, 3).map(edge =>
    `${timelineReadableSubgoalLabel(edge.dependent)} waits for ${timelineReadableSubgoalLabel(edge.prerequisite)}`
  );
  return `${total} hard-block edges; examples: ${examples.join('; ')}`;
}

function parseDependencyEdgeEntries(value) {
  const text = formatEventValue(value);
  if (!text) return [];
  const parts = text.split(/;\s(?=(?:agent\d+|[A-Z])@[-\d,()]+ depends-on )/);
  return parts.map(part => {
    const match = part.match(/^(.+?) depends-on (.+?) source=([^\s]+)(?: .*?evidence=(.*))?$/);
    return match ? {
      dependent: match[1],
      prerequisite: match[2],
      source: match[3],
      evidence: match[4] || '',
    } : null;
  }).filter(Boolean);
}

function timelineDebugDecisionHtml(decision) {
  const selection = decision?.selection || null;
  const ordering = decision?.ordering || null;
  const title = selection
    ? `step ${selection.step} | ${selection.subgoal || selection.selectionId || 'subgoal choice'}`
    : `step ${ordering?.step ?? 0} | ${ordering?.orderSnapshotId || 'subgoal ordering'}`;
  const fields = [];
  if (selection) {
    fields.push(['Selected', timelineSubgoalSelectionSummaryText(selection)]);
    fields.push(['Why selected', timelineFriendlySelectionBasis(selection)]);
    fields.push(['Gate checks', timelineFriendlySelectionGates(selection)]);
    fields.push(['Candidate pool', timelineFriendlySubgoalSample(selection.candidateStatusSample, { limit: 5, includeStatus: true })]);
  }
  if (ordering) {
    fields.push(['Priority order', timelineSubgoalOrderingSummaryText(ordering)]);
    fields.push(['Order rule', timelineFriendlyOrderingBasis(ordering)]);
    fields.push(['Distance model', timelineFriendlyHeuristic(ordering.heuristicBasis || selection?.heuristicBasis)]);
    fields.push(['Order sample', timelineFriendlySubgoalSample(ordering.orderSample, { limit: 5 })]);
  }
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>Focus choice reasoning</strong>
        <span>${escapeHtml(title)}</span>
      </div>
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelineDebugIssueHtml(label, group, transaction) {
  const title = group
    ? `step ${timelineIssueStep(group)} | ${timelineCandidateTargetText(group)}`
    : `step ${timelineIssueStep(transaction)} | ${transaction?.transactionId || 'transaction'}`;
  const fields = [];
  if (group) {
    fields.push(['Outcome', timelineCandidateOutcomeText(group)]);
    fields.push(['Reason', timelineCandidateReasonsText(group)]);
    fields.push(['Boxes checked', timelineBoxesCheckedText(group)]);
    fields.push(['Rejected boxes', joinSet(group.candidateSamples, 3)]);
    fields.push(['Reject counts', joinSet(group.rejectCounts, 3)]);
    fields.push(['Hungarian', timelineCandidateHungarianText(group)]);
  }
  if (transaction) {
    const txReason = transaction.rollbackReason || transaction.reason || transaction.commitStatus || '';
    const span = transaction.planStartStep != null && transaction.planEndStep != null
      ? `${transaction.planStartStep}-${transaction.planEndStep}`
      : '';
    fields.push(['Transaction', `${transaction.transactionId || 'transaction'} ${timelineTransactionRiskLabel(transaction)}${txReason ? `: ${txReason}` : ''}`]);
    fields.push(['Plan span', span]);
  }
  return `
    <article class="timelineDebugIssue">
      <div class="timelineDebugIssueHeader">
        <strong>${escapeHtml(label)}</strong>
        <span>${escapeHtml(title)}</span>
      </div>
      ${timelineDebugFieldRowsHtml(fields)}
    </article>
  `;
}

function timelineDebugFieldRowsHtml(fields) {
  const rows = fields
    .filter(([, value]) => value !== undefined && value !== null && value !== '')
    .map(([label, value]) => `<dt>${escapeHtml(label)}</dt><dd>${escapeHtml(formatEventValue(value))}</dd>`)
    .join('');
  return rows ? `<dl>${rows}</dl>` : '';
}

function timelineCandidateProblemGroups(candidates) {
  const groups = new Map();
  for (const candidate of candidates) {
    if (!timelineIsProblemCandidate(candidate)) continue;
    const key = [
      candidate.step ?? '-',
      candidate.agentId ?? '-',
      candidate.boxType || '-',
      candidate.goal || candidate.subgoal || '-',
    ].join('|');
    const group = groups.get(key) || {
      step: candidate.step,
      agentId: candidate.agentId,
      boxType: candidate.boxType,
      goal: candidate.goal,
      subgoal: candidate.subgoal,
      subgoalType: candidate.subgoalType,
      retryCount: 0,
      failedCount: 0,
      rejectedCount: 0,
      rawBoxesOfType: null,
      reasons: new Set(),
      verdicts: new Set(),
      candidateSamples: new Set(),
      rejectCounts: new Set(),
      hungarianStatuses: new Set(),
      usedHungarianValues: new Set(),
    };
    const verdict = String(candidate.verdict || '').toLowerCase();
    if (verdict.includes('retry')) group.retryCount++;
    if (verdict.includes('fail')) group.failedCount++;
    if (verdict.includes('reject')) group.rejectedCount++;
    if (candidate.rawBoxesOfType != null) {
      group.rawBoxesOfType = Math.max(group.rawBoxesOfType ?? 0, candidate.rawBoxesOfType);
    }
    if (candidate.reason || candidate.dominantReason) group.reasons.add(candidate.dominantReason || candidate.reason);
    if (candidate.verdict) group.verdicts.add(candidate.verdict);
    if (candidate.candidateSamples) group.candidateSamples.add(candidate.candidateSamples);
    if (candidate.candidateRejectCounts) group.rejectCounts.add(candidate.candidateRejectCounts);
    if (candidate.hungarianStatus) group.hungarianStatuses.add(candidate.hungarianStatus);
    if (candidate.usedHungarian !== '') group.usedHungarianValues.add(String(candidate.usedHungarian));
    groups.set(key, group);
  }
  return [...groups.values()].sort((a, b) => timelineIssueStep(a) - timelineIssueStep(b));
}

function timelinePrimaryCandidateGroup(transaction, groups) {
  if (groups.length === 0) return null;
  if (transaction) {
    const txStep = timelineIssueStep(transaction);
    const exact = groups.find(group => timelineIssueStep(group) === txStep && timelineTransactionMatchesCandidate(transaction, group));
    if (exact) return exact;
    const start = transaction.planStartStep ?? txStep;
    const end = transaction.planEndStep ?? txStep;
    const within = groups.find(group => timelineIssueStep(group) >= start && timelineIssueStep(group) <= end && timelineTransactionMatchesCandidate(transaction, group));
    if (within) return within;
  }
  return groups.find(group => group.retryCount > 0) || groups[0];
}

function timelineTransactionMatchesCandidate(transaction, group) {
  if (!transaction || !group) return false;
  if (Number.isInteger(transaction.agentId) && Number.isInteger(group.agentId) && transaction.agentId !== group.agentId) return false;
  if (transaction.boxType && group.boxType && transaction.boxType !== group.boxType) return false;
  return true;
}

function timelineIsRiskyTransaction(transaction) {
  return /roll|fail|reject/.test(String(transaction?.commitStatus || '').toLowerCase());
}

function timelineIsProblemCandidate(candidate) {
  return /fail|retry|reject/.test(String(candidate?.verdict || '').toLowerCase());
}

function timelineIssueStep(item) {
  return item?.step ?? item?.planEndStep ?? item?.planStartStep ?? 0;
}

function timelineTransactionRiskLabel(transaction) {
  const status = String(transaction?.commitStatus || '').toLowerCase();
  if (status.includes('roll')) return 'rollback';
  if (status.includes('fail')) return 'failed';
  if (status.includes('reject')) return 'rejected';
  return status || 'needs review';
}

function timelineCandidateTargetText(group) {
  if (!group) return 'box selection';
  const agent = Number.isInteger(group.agentId) ? `agent${group.agentId}` : 'agent?';
  const box = group.boxType ? `${group.boxType}` : 'box';
  const goal = timelineGoalText(group.goal);
  if (goal) return `${agent} -> ${box} goal ${goal}`;
  return group.subgoal || `${agent} -> ${box}`;
}

function timelineGoalText(value) {
  const raw = formatEventValue(value);
  const match = raw.match(/^\(?\s*(-?\d+)\s*,\s*(-?\d+)\s*\)?$/);
  return match ? `(${match[1]},${match[2]})` : raw;
}

function timelineCandidateOutcomeText(group) {
  const parts = [];
  if (group.retryCount) parts.push(`${group.retryCount} retry`);
  if (group.failedCount) parts.push(`${group.failedCount} failed`);
  if (group.rejectedCount) parts.push(`${group.rejectedCount} rejected`);
  return parts.join(' + ') || joinSet(group.verdicts, 3) || 'candidate issue';
}

function timelineCandidateReasonsText(group) {
  return joinSet(group.reasons, 3) || joinSet(group.verdicts, 3) || 'unknown reason';
}

function timelineBoxesCheckedText(group) {
  if (group.rawBoxesOfType == null) return '';
  const label = group.boxType ? `${group.boxType} box` : 'box';
  return `${group.rawBoxesOfType} ${label}${group.rawBoxesOfType === 1 ? '' : 'es'}`;
}

function timelineCandidateHungarianText(group) {
  const parts = [];
  const used = joinSet(group.usedHungarianValues, 2);
  const statuses = joinSet(group.hungarianStatuses, 2);
  if (used) parts.push(`used=${used}`);
  if (statuses) parts.push(statuses);
  return parts.join(' | ');
}

function renderPortfolioAttemptsCard() {
  const card = document.createElement('section');
  card.className = 'timelineSummaryCard timelinePortfolioCard';
  const portfolio = timelinePortfolioSummary(replay.diagnostics?.portfolioAttempts || []);
  const successText = portfolio.successfulAttemptOrdinal == null ? 'no successful attempt recorded' : `successful attempt #${portfolio.successfulAttemptOrdinal}`;
  const attempts = portfolio.attempts.slice(0, 6);
  const rows = attempts.map(attempt => {
    const goals = attempt.finalSatisfiedGoals != null && attempt.finalTotalGoals != null
      ? `${attempt.finalSatisfiedGoals}/${attempt.finalTotalGoals}`
      : '-';
    const boxGoals = attempt.finalSatisfiedBoxGoals != null && attempt.finalTotalBoxGoals != null
      ? `${attempt.finalSatisfiedBoxGoals}/${attempt.finalTotalBoxGoals}`
      : '-';
    const stateHash = attempt.finalStateHash ? shortHash(attempt.finalStateHash, 10) : '-';
    return `
      <div class="portfolioRow">
        <span>${escapeHtml(String(attempt.ordinal ?? '-'))}</span>
        <strong>${escapeHtml(attempt.label || attempt.strategy || 'attempt')}</strong>
        <span>${escapeHtml(attempt.success ? 'success' : attempt.failureKind || 'failed')}</span>
        <span>${escapeHtml(String(attempt.planSteps || 0))}</span>
        <span>${escapeHtml(boxGoals)}</span>
        <span>${escapeHtml(stateHash)}</span>
      </div>
    `;
  }).join('');
  const overflow = portfolio.attempts.length > attempts.length
    ? `<p class="timelineFoldedNote">+${portfolio.attempts.length - attempts.length} more attempt${portfolio.attempts.length - attempts.length === 1 ? '' : 's'} in Export Meta.</p>`
    : '';
  card.innerHTML = `
    <div>
      <strong>Portfolio Attempts</strong>
      <small>${portfolio.attemptCount} attempt${portfolio.attemptCount === 1 ? '' : 's'} | ${escapeHtml(successText)} | best ${portfolio.bestPlanSteps || 0} steps</small>
    </div>
    ${portfolio.attemptCount === 0 ? '<p class="timelineFoldedNote">No portfolio attempt data recorded in this replay.</p>' : `
      <div class="portfolioTable" role="table" aria-label="Portfolio attempts">
        <div class="portfolioRow portfolioHeader" role="row">
          <span>#</span>
          <span>Strategy</span>
          <span>Result</span>
          <span>Steps</span>
          <span>Box Goals</span>
          <span>State</span>
        </div>
        ${rows}
      </div>
      ${overflow}
    `}
  `;
  return card;
}

function selectLifecycleSegment(index) {
  if (!replay || !Number.isInteger(index)) return;
  const segments = annotateDebugSegments(plannerLifecycleSpans());
  const segment = segments[index];
  if (!segment) return;
  lifecycleSelection = index;
  timelineSelectionKind = 'stage';
  timelineSelectedSegmentIndex = index;
  timelineSelectedChapterIndex = null;
  renderAgentTimeline();
}

function selectDebugChapter(index) {
  if (!replay || !Number.isInteger(index)) return;
  const chapters = debugTimelineChapters(annotateDebugSegments(plannerLifecycleSpans()));
  const chapter = chapters[index];
  if (!chapter) return;
  lifecycleSelection = chapter.firstSegmentIndex;
  timelineSelectionKind = 'chapter';
  timelineSelectedChapterIndex = index;
  timelineSelectedSegmentIndex = chapter.bestSegmentIndex ?? chapter.firstSegmentIndex;
  renderAgentTimeline();
}

function jumpTimelineRange(start, _end, options = {}) {
  if (!replay) return;
  setStep(Math.round(clamp(Number(start), 0, replay.frames.length - 1)));
  if (options.close) closeTimelinePanel();
}

function lifecycleSegmentAt(index) {
  if (!replay || !Number.isInteger(index)) return null;
  const segments = annotateDebugSegments(plannerLifecycleSpans());
  return segments[index] || null;
}

function focusLifecycleSegment(index) {
  if (timer) return;
  const segment = lifecycleSegmentAt(index);
  if (!segment) return;
  setSegmentRange(segment.start, segment.end, 'Lifecycle Stage', { timelineIndex: index, timelineKind: 'stage' });
  lifecycleSelection = index;
  timelineSelectionKind = 'stage';
  timelineSelectedSegmentIndex = index;
  timelineSelectedChapterIndex = null;
  renderAgentTimeline();
}

function playLifecycleSegment(index) {
  if (timer) return;
  const segment = lifecycleSegmentAt(index);
  if (!segment) return;
  setSegmentRange(segment.start, segment.end, 'Lifecycle Stage', { timelineIndex: index, timelineKind: 'stage' });
  lifecycleSelection = index;
  timelineSelectionKind = 'stage';
  timelineSelectedSegmentIndex = index;
  timelineSelectedChapterIndex = null;
  setStep(segment.start);
  closeTimelinePanel();
  if (timer) stopPlay();
  startPlay();
}

function jumpLifecycleSegment(index) {
  if (timer) return;
  const segment = lifecycleSegmentAt(index);
  if (!segment) return;
  lifecycleSelection = index;
  timelineSelectionKind = 'stage';
  timelineSelectedSegmentIndex = index;
  timelineSelectedChapterIndex = null;
  jumpTimelineRange(segment.start, segment.end);
}

function focusDebugChapter(index) {
  if (timer) return;
  const chapter = timelineChapterAt(index);
  if (!chapter) return;
  setSegmentRange(chapter.start, chapter.end, 'Timeline Chapter', { timelineIndex: index, timelineKind: 'chapter' });
  renderAgentTimeline();
}

function playDebugChapter(index) {
  if (timer) return;
  const chapter = timelineChapterAt(index);
  if (!chapter) return;
  setSegmentRange(chapter.start, chapter.end, 'Timeline Chapter', { timelineIndex: index, timelineKind: 'chapter' });
  setStep(chapter.start);
  closeTimelinePanel();
  if (timer) stopPlay();
  startPlay();
}

function jumpDebugChapter(index) {
  if (timer) return;
  const chapter = timelineChapterAt(index);
  if (!chapter) return;
  jumpTimelineRange(chapter.start, chapter.end, { close: true });
}

function timelineChapterAt(index) {
  if (!replay || !Number.isInteger(index)) return null;
  const chapters = debugTimelineChapters(annotateDebugSegments(plannerLifecycleSpans()));
  return chapters[index] || null;
}

function validTimelineSegmentIndex(index, segments) {
  return Number.isInteger(index) && index >= 0 && index < segments.length;
}

function validTimelineChapterIndex(index, chapters) {
  return Number.isInteger(index) && index >= 0 && index < chapters.length;
}

// Greedy lane packing: place time-positioned items into the fewest horizontal
// lanes so none overlap. Items must carry numeric { left, width }.
function packTimelineLanes(items) {
  const laneRight = [];
  const placed = items
    .map((item, i) => ({ ...item, _order: i }))
    .sort((a, b) => a.left - b.left || a._order - b._order)
    .map(item => {
      const packWidth = Number.isFinite(item.packWidth) ? item.packWidth : item.width;
      let lane = laneRight.findIndex(right => right <= item.left - 2);
      if (lane === -1) { lane = laneRight.length; laneRight.push(0); }
      laneRight[lane] = item.left + packWidth + 4;
      return { ...item, lane };
    });
  return { placed, laneCount: Math.max(1, laneRight.length) };
}

function timelineLayerVisualRanges(items, maxStep) {
  const sorted = items
    .map((item, index) => ({ item, index }))
    .sort((a, b) => a.item.start - b.item.start || a.item.end - b.item.end || a.index - b.index);
  const ranges = new Map();
  for (let i = 0; i < sorted.length; i++) {
    const { item, index } = sorted[i];
    const start = Number.isFinite(item.start) ? item.start : 0;
    const end = Number.isFinite(item.end) ? item.end : start;
    const next = sorted.slice(i + 1).find(entry =>
      Number.isFinite(entry.item.start) && (entry.item.start === end || (end <= start && entry.item.start === start)));
    const nominalRight = end >= maxStep ? maxStep : end + 1;
    const right = next && next.item.start <= nominalRight
      ? next.item.start
      : nominalRight;
    ranges.set(index, {
      start,
      end,
      right: Math.max(start, right),
    });
  }
  return ranges;
}

function timelineRangeContainsStep(range, stepIndex) {
  if (!range) return false;
  if (range.right <= range.start) return stepIndex === range.start;
  return stepIndex >= range.start && stepIndex < range.right;
}

function alignFinalTimelineEdges(taskPack, stagePack, maxStep) {
  const actualWidthOf = item => Number.isFinite(item.durationWidth) ? item.durationWidth : item.width;
  const finalRight = Math.max(
    0,
    ...taskPack.placed
      .filter(item => item.band?.end >= maxStep)
      .map(item => item.left + actualWidthOf(item)),
    ...stagePack.placed
      .filter(item => item.segment?.end >= maxStep)
      .map(item => item.left + actualWidthOf(item))
  );
  if (!finalRight) return;
  for (const item of taskPack.placed) {
    if (item.band?.end >= maxStep) {
      item.durationWidth = Math.max(actualWidthOf(item), finalRight - item.left);
      item.width = Math.max(item.width, item.durationWidth);
    }
  }
  for (const item of stagePack.placed) {
    if (item.segment?.end >= maxStep) {
      item.durationWidth = Math.max(actualWidthOf(item), finalRight - item.left);
      item.width = Math.max(item.width, item.durationWidth);
    }
  }
}

// Pick a "nice" tick interval (1/2/5 x 10^n) so the ruler shows ~one label per 90px.
function niceTimelineTick(maxStep, canvasWidth) {
  const targetTicks = Math.max(2, Math.min(14, Math.floor(canvasWidth / 90)));
  const raw = Math.max(1, maxStep / targetTicks);
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const frac = raw / pow;
  const nice = frac <= 1 ? 1 : frac <= 2 ? 2 : frac <= 5 ? 5 : 10;
  return Math.max(1, Math.round(nice * pow));
}

// Wire up drag-to-pan + native horizontal scroll, and preserve the scroll offset
// across the full re-renders that every player step triggers.
function attachTimelinePan(scroll, options = {}) {
  const isMini = Boolean(options.mini);
  const restoreLeft = Number.isFinite(options.restoreLeft)
    ? options.restoreLeft
    : (isMini ? miniTimelineScrollLeft : timelineScrollLeft);
  scroll.addEventListener('scroll', () => {
    if (scroll.__timelineIgnoreScroll) return;
    if (scroll.__timelineProgrammaticRestore) return;
    if (scroll.__timelineRestorePending) return;
    if (Number.isFinite(scroll.__timelineRestoringUntil) && performance.now() < scroll.__timelineRestoringUntil) return;
    scroll.__timelineUserScrolled = true;
    if (isMini) miniTimelineScrollLeft = scroll.scrollLeft;
    else {
      timelineScrollLeft = scroll.scrollLeft;
      if (options.followCurrent && !timer) timelineAutoFollowDisabled = true;
    }
  });
  let panning = false;
  let startX = 0;
  let startScroll = 0;
  let moved = false;
  scroll.addEventListener('pointerdown', event => {
    if (event.button !== 0) return;
    panning = true;
    moved = false;
    startX = event.clientX;
    startScroll = scroll.scrollLeft;
    timelinePanSuppressClick = false;
  });
  scroll.addEventListener('pointermove', event => {
    if (!panning) return;
    const dx = event.clientX - startX;
    if (!moved && Math.abs(dx) > 4) {
      moved = true;
      scroll.classList.add('panning');
      try { scroll.setPointerCapture(event.pointerId); } catch (_) { /* ignore */ }
    }
    if (moved) scroll.scrollLeft = startScroll - dx;
  });
  const endPan = () => {
    if (!panning) return;
    panning = false;
    scroll.classList.remove('panning');
    if (moved) {
      timelinePanSuppressClick = true;
      if (!isMini && !timer) timelineAutoFollowDisabled = true;
    }
  };
  scroll.addEventListener('pointerup', endPan);
  scroll.addEventListener('pointercancel', endPan);
  scroll.addEventListener('pointerleave', endPan);
  scroll.__timelineRestorePending = true;
  scroll.__timelineRestoreLeft = restoreLeft;
  const applyRestoredLeft = left => {
    scroll.__timelineProgrammaticRestore = true;
    scroll.__timelineRestoringUntil = performance.now() + 180;
    scroll.scrollLeft = left;
    if (isMini) miniTimelineScrollLeft = left;
    else timelineScrollLeft = left;
    window.setTimeout(() => { scroll.__timelineProgrammaticRestore = false; }, 80);
  };
  if (!options.followCurrent && !options.lockCurrent && !options.alignCurrentEnd) {
    applyRestoredLeft(restoreLeft);
    requestAnimationFrame(() => {
      if (!scroll.__timelineUserScrolled) applyRestoredLeft(restoreLeft);
      scroll.__timelineRestorePending = false;
    });
    return;
  }
  const restoreScroll = () => {
    const nextLeft = timelineCurrentScrollLeft(scroll, restoreLeft, {
      centerCurrent: Boolean(options.followCurrent) && (Boolean(timer) || Boolean(options.lockCurrent)),
      alignCurrentEnd: Boolean(options.alignCurrentEnd),
      keepCurrentVisible: Boolean(options.followCurrent),
    });
    scroll.__timelineRestoreLeft = nextLeft;
    scroll.__timelineIgnoreScroll = true;
    applyRestoredLeft(nextLeft);
  };
  requestAnimationFrame(() => {
    restoreScroll();
    requestAnimationFrame(() => {
      restoreScroll();
      scroll.__timelineIgnoreScroll = false;
      scroll.__timelineRestorePending = false;
    });
  });
}

function timelineCurrentScrollLeft(scroll, fallbackLeft, options = {}) {
  const now = scroll.querySelector('.timelineGanttNow');
  const maxLeft = Math.max(0, scroll.scrollWidth - scroll.clientWidth);
  if (!now) return clamp(fallbackLeft, 0, maxLeft);
  const nowLeft = now.offsetLeft;
  if (options.alignCurrentEnd) {
    return Math.round(clamp(nowLeft - scroll.clientWidth + 1, 0, maxLeft));
  }
  if (options.centerCurrent) {
    return Math.round(clamp(nowLeft - scroll.clientWidth / 2, 0, maxLeft));
  }
  if (!options.keepCurrentVisible) return Math.round(clamp(fallbackLeft, 0, maxLeft));
  let nextLeft = clamp(fallbackLeft, 0, maxLeft);
  const margin = Math.min(92, Math.max(42, Math.floor(scroll.clientWidth * 0.22)));
  if (nowLeft < nextLeft + margin) {
    nextLeft = nowLeft - margin;
  } else if (nowLeft > nextLeft + scroll.clientWidth - margin) {
    nextLeft = nowLeft - scroll.clientWidth + margin;
  }
  return Math.round(clamp(nextLeft, 0, maxLeft));
}

function renderTimelineMap(segments, options = {}) {
  const { compact = false, progress = false, includeHeader = true, followCurrent = false, lockCurrent = false, alignCurrentEnd = false } = options;
  const gantt = document.createElement('section');
  gantt.className = 'timelineGantt timelineMap';
  if (compact) gantt.classList.add('compact');
  if (!segments.length) {
    gantt.textContent = 'No lifecycle stages found.';
    return gantt;
  }
  const maxStep = Math.max(1, replay.frames.length - 1);
  const currentTask = primaryTaskForStep(step);

  const header = document.createElement('div');
  header.className = 'timelineMapHeader';
  header.innerHTML = `
    <div class="timelineMapTitle">
      <strong>Primary task: ${escapeHtml(currentTask ? currentTask.displayLabel : 'none')}</strong>
      <span>upper lane: primary tasks | lower lane: lifecycle stages | drag to pan</span>
    </div>
  `;
  if (includeHeader && !compact) {
    let actionIndex = validTimelineSegmentIndex(lifecycleSelection, segments)
      ? lifecycleSelection
      : segments.findIndex(item => step >= item.start && step <= item.end);
    if (!validTimelineSegmentIndex(actionIndex, segments)) actionIndex = 0;
    const actions = document.createElement('div');
    actions.className = 'timelineMapActions';
    if (validTimelineSegmentIndex(actionIndex, segments)) {
      const disabledAttr = timer ? ' disabled aria-disabled="true"' : '';
      const playLabel = timer ? 'Pause replay' : 'Play replay';
      const playingClass = timer ? ' playing' : '';
      actions.innerHTML = `
        <button class="miniButton" type="button" data-lifecycle-jump-index="${actionIndex}"${disabledAttr}>Jump</button>
        <button class="miniButton" type="button" data-lifecycle-focus-index="${actionIndex}"${disabledAttr}>Focus Range</button>
        <button class="miniButton" type="button" data-lifecycle-play-index="${actionIndex}"${disabledAttr}>Play Range</button>
        <button class="miniTimelinePlayBtn timelineMapPlayBtn${playingClass}" type="button" data-timeline-play-toggle title="${playLabel}" aria-label="${playLabel}" aria-pressed="${timer ? 'true' : 'false'}">
          <span class="miniTimelinePlayIcon" aria-hidden="true"></span>
        </button>
      `;
    }
    header.appendChild(actions);
  }

  // Time-proportional horizontal scale with a readable minimum bar width.
  // Bars are sized by real duration; the canvas grows wider than the viewport so
  // it scrolls instead of cramming every stage into one fixed width.
  const MIN_BAR_PX = compact ? 34 : 50;
  const POINT_BAR_PX = compact ? 5 : 7;
  const MIN_TASK_LABEL_PX = compact ? 120 : 180;
  const TASK_LANE_H = compact ? 24 : 32;
  const STAGE_LANE_H = compact ? 16 : 20;
  const RULER_H = compact ? 18 : 22;
  const viewportW = Math.max(compact ? 220 : 280, ((compact ? els.miniTimelineBody : els.agentTimeline)?.clientWidth || 440) - 18);
  const avgDur = Math.max(1, (maxStep + 1) / segments.length);
  const taskBands = primaryTaskBandsForTimeline(segments);
  const taskLabelWidth = (label, txText = '') => Math.ceil(
    compact
      ? 28 + String(label || '').length * 5.8 + String(txText || '').length * 5.2
      : 44 + String(label || '').length * 7.4 + String(txText || '').length * 6.3
  );
  const stageLabelWidth = (label, ordinal) => Math.ceil(
    compact
      ? 30 + String(label || '').length * 4.8 + String(ordinal || '').length * 7
      : 44 + String(label || '').length * 6.8 + String(ordinal || '').length * 7
  );
  const taskRanges = timelineLayerVisualRanges(taskBands, maxStep);
  const stageRanges = timelineLayerVisualRanges(segments, maxStep);
  const labelScalePxPerStep = Math.max(
    ...taskBands.map((band, index) => {
      const range = taskRanges.get(index);
      const duration = Math.max(0, (range?.right ?? band.end) - (range?.start ?? band.start));
      if (duration <= 2) return 0;
      const txText = band.transactions.length ? band.transactions.map(tx => tx.transactionId).join('/') : '';
      return Math.max(taskLabelWidth(band.displayLabel, txText), MIN_TASK_LABEL_PX) / duration;
    }),
    ...segments.map((segment, index) => {
      const range = stageRanges.get(index);
      const duration = Math.max(0, (range?.right ?? segment.end) - (range?.start ?? segment.start));
      if (duration <= 2) return 0;
      return stageLabelWidth(segment.label, index + 1) / duration;
    }),
    0
  );
  const basePxPerStep = Math.max(
    clamp((compact ? MIN_BAR_PX : MIN_BAR_PX * 1.35) / avgDur, compact ? 0.5 : 0.85, 16),
    clamp(labelScalePxPerStep, 0, 12)
  );
  const timeWidth = Math.max(viewportW, Math.round((maxStep + 1) * basePxPerStep));
  const pxPerStep = timeWidth / (maxStep + 1);
  const leftOf = start => start * pxPerStep;
  const widthForRange = range => {
    const strictWidth = Math.max(0, (range.right - range.start) * pxPerStep);
    return strictWidth > 0 ? Math.max(1, strictWidth) : POINT_BAR_PX;
  };
  const taskPack = packTimelineLanes(taskBands.map((band, index) => {
    const range = taskRanges.get(index) || { start: band.start, right: band.end };
    const durationWidth = widthForRange(range);
    const txText = band.transactions.length ? band.transactions.map(tx => tx.transactionId).join('/') : '';
    const labelWidth = Math.max(taskLabelWidth(band.displayLabel, txText), MIN_TASK_LABEL_PX);
    const width = Math.max(durationWidth, labelWidth);
    return {
      band,
      index,
      range,
      left: leftOf(range.start),
      width,
      durationWidth,
      packWidth: width,
    };
  }));
  const stagePack = packTimelineLanes(segments.map((segment, index) => {
    const range = stageRanges.get(index) || { start: segment.start, right: segment.end };
    const durationWidth = widthForRange(range);
    const labelWidth = Math.max(stageLabelWidth(segment.label, index + 1), MIN_BAR_PX);
    const width = Math.max(durationWidth, labelWidth);
    return {
      segment,
      index,
      range,
      left: leftOf(range.start),
      width,
      durationWidth,
      packWidth: width,
    };
  }));
  alignFinalTimelineEdges(taskPack, stagePack, maxStep);

  const taskFieldH = taskBands.length ? taskPack.laneCount * TASK_LANE_H + 5 : 0;
  const fieldH = stagePack.laneCount * STAGE_LANE_H + 6;
  const canvasWidth = Math.ceil(Math.max(
    timeWidth,
    ...taskPack.placed.map(item => item.left + Math.max(item.width, item.packWidth || 0)),
    ...stagePack.placed.map(item => item.left + Math.max(item.width, item.packWidth || 0)),
  ));
  const canvasHeight = RULER_H + taskFieldH + fieldH;

  const scroll = document.createElement('div');
  scroll.className = 'timelineScroll';
  const canvas = document.createElement('div');
  canvas.className = 'timelineCanvas';
  canvas.style.width = `${canvasWidth}px`;
  canvas.style.height = `${canvasHeight}px`;

  const ruler = document.createElement('div');
  ruler.className = 'timelineRuler';
  ruler.style.height = `${RULER_H}px`;
  const tickInterval = niceTimelineTick(maxStep, canvasWidth);
  for (let s = 0; s <= maxStep; s += tickInterval) {
    const tick = document.createElement('span');
    tick.className = 'timelineRulerTick';
    tick.style.left = `${leftOf(s)}px`;
    tick.textContent = String(s);
    ruler.appendChild(tick);
  }

  const taskField = document.createElement('div');
  taskField.className = 'timelinePrimaryTaskField';
  taskField.style.height = `${taskFieldH}px`;
  taskField.setAttribute('aria-label', 'Primary tasks');
  for (const item of taskPack.placed) {
    const band = item.band;
    const bar = document.createElement('button');
    bar.type = 'button';
    bar.className = `timelinePrimaryTaskBar ${band.statusClass || 'note'}`;
    if (timelineRangeContainsStep(item.range, step)) bar.classList.add('active');
    bar.dataset.step = String(band.start);
    bar.dataset.segmentStart = String(band.start);
    bar.dataset.segmentEnd = String(band.end);
    const txText = band.transactions.length
      ? band.transactions.map(tx => tx.transactionId).join('/')
      : 'no tx';
    bar.title = `${band.displayLabel} (${band.start}-${band.end}) | ${txText}`;
    bar.setAttribute('aria-label', bar.title);
    bar.style.left = `${item.left}px`;
    bar.style.width = `${item.width}px`;
    bar.style.setProperty('--duration-width', `${item.durationWidth || item.width}px`);
    bar.style.top = `${item.lane * TASK_LANE_H + 2}px`;
    bar.innerHTML = `
      <span class="taskText">
        <span class="taskLabel">${escapeHtml(band.displayLabel)}</span>
        <span class="taskTxList">${escapeHtml(txText)}</span>
      </span>
    `;
    taskField.appendChild(bar);
  }

  const field = document.createElement('div');
  field.className = 'timelineStageField';
  field.style.height = `${fieldH}px`;
  field.setAttribute('aria-label', 'Lifecycle stages');
  for (const item of stagePack.placed) {
    const segment = item.segment;
    const bar = document.createElement('button');
    bar.type = 'button';
    bar.className = `timelineStageBar ${segment.statusClass || 'note'}`;
    if (segment.highSignal) bar.classList.add('hotspot');
    if (item.index === lifecycleSelection) bar.classList.add('active');
    if (markersInRange(segment.start, segment.end).length > 0) bar.classList.add('marked');
    bar.dataset.lifecycleIndex = String(item.index);
    bar.title = `Stage ${item.index + 1}: ${segment.label} (${segmentRangeText(segment)})${segment.primaryTask ? ` | Primary ${segment.primaryTask.displayLabel}` : ''}`;
    bar.setAttribute('aria-label', bar.title);
    bar.style.left = `${item.left}px`;
    bar.style.width = `${item.width}px`;
    bar.style.setProperty('--duration-width', `${item.durationWidth || item.width}px`);
    bar.style.top = `${item.lane * STAGE_LANE_H + 3}px`;
    if (progress) {
      bar.classList.add('progress');
      const progressWidth = Math.round((item.durationWidth || item.width) * lifecycleStageProgress(segment));
      bar.style.setProperty('--stage-progress-width', `${progressWidth}px`);
    }
    bar.innerHTML = `
      <span class="stageText">
        <span class="barIdx">${item.index + 1}</span>
        <span class="barLabel">${escapeHtml(segment.label)}</span>
      </span>
    `;
    field.appendChild(bar);
  }

  const now = document.createElement('span');
  now.className = 'timelineGanttNow';
  const shouldAlignCurrentEnd = alignCurrentEnd || step >= maxStep;
  let nowLeft = leftOf(clamp(step, 0, maxStep));
  if (shouldAlignCurrentEnd) {
    const currentStage = [...stagePack.placed]
      .reverse()
      .find(item => timelineRangeContainsStep(item.range, step))
      || stagePack.placed[stagePack.placed.length - 1];
    if (currentStage) nowLeft = currentStage.left + (currentStage.durationWidth || currentStage.width);
  }
  now.style.left = `${Math.min(nowLeft, canvasWidth)}px`;
  now.title = `current step ${step}`;

  canvas.append(ruler);
  if (taskBands.length) canvas.appendChild(taskField);
  canvas.append(field, now);
  scroll.appendChild(canvas);

  const footer = document.createElement('div');
  footer.className = 'timelineGanttFooter';
  footer.innerHTML = `
    <span>step 0</span>
    <strong>${escapeHtml(String(taskBands.length))} primary task${taskBands.length === 1 ? '' : 's'} | ${escapeHtml(String(segments.length))} lifecycle stages</strong>
    <span>step ${escapeHtml(String(maxStep))}</span>
  `;

  attachTimelinePan(scroll, {
    mini: compact,
    followCurrent,
    lockCurrent,
    alignCurrentEnd: shouldAlignCurrentEnd,
    restoreLeft: compact ? miniTimelineScrollLeft : timelineScrollLeft,
  });
  if (includeHeader) gantt.appendChild(header);
  gantt.append(scroll, footer);
  return gantt;
}

function lifecycleStageProgress(segment) {
  if (!segment || !Number.isInteger(segment.start) || !Number.isInteger(segment.end)) return 0;
  if (step < segment.start) return 0;
  if (step > segment.end) return 1;
  const duration = Math.max(1, segment.end - segment.start + 1);
  return clamp((step - segment.start + 1) / duration, 0, 1);
}

function renderTimelineSelectionMessage(segments, chapters, activeChapterIndex, currentStep) {
  const section = document.createElement('section');
  const selectedSegment = validTimelineSegmentIndex(lifecycleSelection, segments) ? segments[lifecycleSelection] : null;
  const selectedChapter = chapters[activeChapterIndex] || null;
  const statusClass = selectedSegment?.statusClass || selectedChapter?.statusClass || 'note';
  section.className = `timelineSelectionMessage ${escapeHtml(statusClass)}`;

  if (timelineSelectionKind === 'chapter' && selectedChapter) {
    section.innerHTML = `
      <div class="timelineSelectionHeader">
        <div>
          <strong>Selected Chapter ${selectedChapter.index + 1}: ${escapeHtml(selectedChapter.label)}</strong>
          <small>${escapeHtml(chapterRangeText(selectedChapter))} | ${selectedChapter.stageCount} stage${selectedChapter.stageCount === 1 ? '' : 's'} | ${selectedChapter.hotspotCount} hotspot${selectedChapter.hotspotCount === 1 ? '' : 's'}</small>
        </div>
        <span>${escapeHtml(selectedChapter.statusLabel)}</span>
      </div>
      <p>${escapeHtml(selectedChapter.focusText)}</p>
    `;
    return section;
  }

  if (selectedSegment) {
    const primaryAgentId = lifecyclePrimaryAgentId(selectedSegment);
    const selectionLabel = timelineSelectionKind === 'stage' ? 'Selected Stage' : 'Current Stage';
    const primaryTask = selectedSegment.primaryTask;
    const supportSummary = timelineSupportSummaryText(timelineSupportRollup(selectedSegment));
    const supportRow = supportSummary
      ? `<dt>Support</dt><dd>${escapeHtml(supportSummary)}</dd>`
      : '';
    const decisionRows = timelineDecisionRowsHtml(selectedSegment, { includeSamples: false });
    const primaryTaskRows = primaryTask ? `
        <dt>Primary Task</dt><dd>${escapeHtml(primaryTask.displayLabel)}</dd>
        <dt>Transaction</dt><dd>${escapeHtml(primaryTaskTransactionText(primaryTask))}</dd>
      ` : `
        <dt>Primary Task</dt><dd>none recorded</dd>
        <dt>Transaction</dt><dd>not recorded</dd>
      `;
    section.innerHTML = `
      <div class="timelineSelectionHeader">
        <div>
          <strong>${selectionLabel} ${selectedSegment.ordinal ?? selectedSegment.segmentIndex + 1}: ${escapeHtml(selectedSegment.label)}</strong>
          <small>${escapeHtml(segmentRangeText(selectedSegment))} | ${escapeHtml(primaryTask ? primaryTask.displayLabel : (Number.isInteger(primaryAgentId) ? `agent${primaryAgentId}` : 'mixed agents'))}</small>
        </div>
        <span>${escapeHtml(selectedSegment.statusLabel || 'active')}</span>
      </div>
      <dl>
        ${primaryTaskRows}
        <dt>Current Stage</dt><dd>${escapeHtml(selectedSegment.label)}</dd>
        <dt>Executor</dt><dd>${escapeHtml(timelineSegmentExecutorText(selectedSegment))}</dd>
        <dt>Phase</dt><dd>${escapeHtml(joinSet(selectedSegment.phases) || 'execution')}</dd>
        <dt>Subgoal</dt><dd>${escapeHtml(selectedSegment.subgoal || 'none')}</dd>
        <dt>Actions</dt><dd>${escapeHtml(joinSet(selectedSegment.actionFamilies) || 'NoOp')}</dd>
        <dt>Signals</dt><dd>${escapeHtml(selectedSegment.debugReasons.join(', ') || 'ordinary execution')}</dd>
        ${supportRow}
        ${decisionRows}
      </dl>
      ${timelineSelectionEventsHtml(selectedSegment)}
    `;
    return section;
  }

  section.innerHTML = `
    <div class="timelineSelectionHeader">
      <div>
        <strong>No stage selected</strong>
        <small>Current player step ${currentStep >= 0 ? currentStep : 0}</small>
      </div>
    </div>
  `;
  return section;
}

function timelineSelectionEventsHtml(segment) {
  const events = segment.events || [];
  if (events.length === 0) {
    return `<p class="timelineSelectionEmpty">No decision event captured for this stage. It may be an ordinary intent/action span.</p>`;
  }
  const items = events.slice(0, 5).map(item => {
    const normalized = normalizeEvent(item.event);
    return `<li class="${escapeHtml(normalized.kindClass)}">
      <span>step ${item.step}</span>
      <strong>${escapeHtml(normalized.title)}</strong>
      <small>${escapeHtml(normalized.message || normalized.kind)}</small>
    </li>`;
  }).join('');
  const overflow = events.length > 5
    ? `<p class="timelineSelectionEmpty">+${events.length - 5} more event${events.length - 5 === 1 ? '' : 's'} in this stage.</p>`
    : '';
  return `<ol class="timelineSelectionEvents">${items}</ol>${overflow}`;
}

function annotateDebugSegments(segments) {
  const totalFrames = Math.max(1, replay?.frames?.length || 1);
  return segments.map((segment, index) => {
    const annotated = { ...segment, ordinal: index + 1, segmentIndex: index };
    annotated.primaryTask = primaryTaskForSegment(annotated);
    const score = segmentDebugScore(annotated, index, segments.length, totalFrames);
    annotated.debugScore = score;
    annotated.debugReasons = segmentDebugReasons(annotated, score, totalFrames);
    annotated.highSignal = score >= 18 || annotated.statusClass === 'danger' || annotated.statusClass === 'warning';
    return annotated;
  });
}

function debugTimelineChapters(segments) {
  if (!segments.length) return [];
  const maxChapters = 8;
  if (segments.length <= maxChapters) {
    return segments.map((segment, index) => createDebugChapter([segment], index));
  }

  const chapters = [];
  let cursor = 0;
  while (cursor < segments.length) {
    const remainingSegments = segments.length - cursor;
    const remainingSlots = maxChapters - chapters.length;
    if (remainingSlots <= 1) {
      chapters.push(createDebugChapter(segments.slice(cursor), chapters.length));
      break;
    }

    const current = segments[cursor];
    const next = segments[cursor + 1];
    const shouldIsolateCurrent = current.highSignal && remainingSegments > remainingSlots;
    if (shouldIsolateCurrent) {
      chapters.push(createDebugChapter([current], chapters.length));
      cursor += 1;
      continue;
    }

    const targetSize = Math.max(1, Math.ceil(remainingSegments / remainingSlots));
    let end = cursor;
    while (end + 1 < segments.length && end - cursor + 1 < targetSize) {
      if (segments[end + 1].highSignal && end > cursor) break;
      end += 1;
    }
    if (next && next.highSignal && end === cursor && remainingSegments > remainingSlots) {
      end = cursor;
    }
    chapters.push(createDebugChapter(segments.slice(cursor, end + 1), chapters.length));
    cursor = end + 1;
  }
  return chapters;
}

function createDebugChapter(chapterSegments, index) {
  const start = Math.min(...chapterSegments.map(segment => segment.start));
  const end = Math.max(...chapterSegments.map(segment => segment.end));
  const best = chapterSegments.reduce((winner, item) => (
    !winner || item.debugScore > winner.debugScore ? item : winner
  ), null);
  const statusClass = chapterSegments.reduce((status, item) => strongerStatus(status, item.statusClass), 'note');
  const hotspotCount = chapterSegments.filter(segment => segment.highSignal).length;
  const label = chapterLabel(chapterSegments, index, best, hotspotCount);
  const primaryAgentId = lifecyclePrimaryAgentId(best);
  return {
    index,
    label,
    shortLabel: compactChapterLabel(label),
    start,
    end,
    statusClass,
    statusLabel: chapterStatusLabel(statusClass, hotspotCount),
    segments: chapterSegments,
    stageCount: chapterSegments.length,
    firstSegmentIndex: chapterSegments[0]?.segmentIndex ?? 0,
    bestSegmentIndex: best?.segmentIndex ?? chapterSegments[0]?.segmentIndex ?? 0,
    bestSegment: best,
    debugScore: chapterSegments.reduce((sum, segment) => sum + (segment.debugScore || 0), 0),
    hotspotCount,
    primaryAgentId,
    focusText: chapterFocusText(chapterSegments, best, hotspotCount),
  };
}

function activeDebugChapterIndex(chapters, activeSegmentIndex, currentStep) {
  if (!chapters.length) return -1;
  if (activeSegmentIndex >= 0) {
    const bySegment = chapters.findIndex(chapter =>
      chapter.segments.some(segment => segment.segmentIndex === activeSegmentIndex));
    if (bySegment >= 0) return bySegment;
  }
  const byStep = chapters.findIndex(chapter => currentStep >= chapter.start && currentStep <= chapter.end);
  if (byStep >= 0) return byStep;
  const firstFocus = chapters.findIndex(chapter => chapter.hotspotCount > 0);
  return firstFocus >= 0 ? firstFocus : 0;
}

function replayDiagnosticFocus() {
  const raw = replay?.diagnosticFocus || replay?.diagnostics?.diagnosticFocus;
  return raw && typeof raw === 'object' ? raw : null;
}

function timelineDiagnosisSummary(segments, chapters) {
  const computed = computedTimelineDiagnosisSummary(segments, chapters);
  const raw = replayDiagnosticFocus();
  if (!raw) return computed;
  return {
    ...computed,
    source: 'replay',
    headline: formatEventValue(raw.headline) || computed.headline,
    detail: formatEventValue(raw.detail) || computed.detail,
    hotspotCount: numberOrNull(raw.focusEventCount) ?? computed.hotspotCount,
    firstFocus: formatEventValue(raw.firstFocus) || computed.firstFocus,
    finalFocus: formatEventValue(raw.finalFocus),
    topEventKinds: normalizeCountEntries(raw.topEventKinds),
    topReasons: normalizeCountEntries(raw.topReasons),
    topSubgoals: normalizeCountEntries(raw.topSubgoals),
  };
}

function computedTimelineDiagnosisSummary(segments, chapters) {
  const outcome = String(replay.summary?.outcome || 'unknown').toLowerCase();
  const unsolved = /(unsolved|partial|failed|failure|timeout|no solution|not solved)/.test(outcome);
  const hotspots = segments.filter(segment => segment.highSignal);
  const firstHotspot = hotspots[0];
  const tail = chapters[chapters.length - 1];
  if (unsolved && firstHotspot) {
    return {
      source: 'computed',
      statusClass: 'danger',
      headline: `Unsolved: first suspect near step ${firstHotspot.start}`,
      detail: `${replay.frames.length} frames | ${replayEventCount()} events | ${segments.length} raw stages folded into ${chapters.length} chapters`,
      hotspotCount: hotspots.length,
      firstFocus: `${firstHotspot.label} ${segmentRangeText(firstHotspot)}`,
    };
  }
  if (unsolved && tail) {
    return {
      source: 'computed',
      statusClass: 'danger',
      headline: `Unsolved: inspect the final chapter`,
      detail: `${replay.frames.length} frames | ${replayEventCount()} events | ${segments.length} raw stages folded into ${chapters.length} chapters`,
      hotspotCount: hotspots.length,
      firstFocus: `${tail.label} ${chapterRangeText(tail)}`,
    };
  }
  if (firstHotspot) {
    return {
      source: 'computed',
      statusClass: firstHotspot.statusClass || 'warning',
      headline: `Solved with ${hotspots.length} diagnostic hotspot${hotspots.length === 1 ? '' : 's'}`,
      detail: `${replay.frames.length} frames | ${replayEventCount()} events | ordinary stages are folded by default`,
      hotspotCount: hotspots.length,
      firstFocus: `${firstHotspot.label} ${segmentRangeText(firstHotspot)}`,
    };
  }
  return {
    source: 'computed',
    statusClass: 'success',
    headline: `Solved cleanly: no high-signal hotspot detected`,
    detail: `${replay.frames.length} frames | ${replayEventCount()} events | ${segments.length} raw stages folded into ${chapters.length} chapters`,
    hotspotCount: 0,
    firstFocus: 'none',
  };
}

function renderDebugChapterDetails(chapter, index, totalChapters) {
  const section = document.createElement('section');
  section.className = `timelineChapterDetails ${escapeHtml(chapter.statusClass || 'note')}`;
  const visibleSegments = chapterVisibleSegments(chapter);
  const hiddenCount = Math.max(0, chapter.segments.length - visibleSegments.length);
  const primaryAgentLabel = Number.isInteger(chapter.primaryAgentId) ? `agent${chapter.primaryAgentId}` : 'mixed agents';
  section.innerHTML = `
    <div class="timelineChapterHeader">
      <div>
        <p class="stepInspectorKicker">Selected Chapter ${index + 1} of ${totalChapters}</p>
        <h3>${escapeHtml(chapter.label)}</h3>
        <small>${escapeHtml(chapter.focusText)}</small>
      </div>
      <div class="timelineChapterActions">
        <button class="miniButton" type="button" data-chapter-jump-index="${index}">Jump</button>
        <button class="miniButton" type="button" data-chapter-focus-index="${index}">Focus Range</button>
        <button class="miniButton" type="button" data-chapter-play-index="${index}">Play Range</button>
      </div>
    </div>
    <dl class="timelineChapterStats">
      <dt>Range</dt><dd>${escapeHtml(chapterRangeText(chapter))}</dd>
      <dt>Status</dt><dd>${escapeHtml(chapter.statusLabel)}</dd>
      <dt>Primary</dt><dd>${escapeHtml(primaryAgentLabel)}</dd>
      <dt>Stages</dt><dd>${chapter.stageCount}${hiddenCount ? ` (${hiddenCount} folded)` : ''}</dd>
    </dl>
  `;

  const flow = document.createElement('div');
  flow.className = `timelineFlow chapterStageFlow${visibleSegments.length > 5 ? ' compact' : ''}`;
  for (const segment of visibleSegments) {
    flow.appendChild(renderChapterStageNode(segment));
  }
  section.appendChild(flow);
  if (hiddenCount > 0) {
    const note = document.createElement('p');
    note.className = 'timelineFoldedNote';
    note.textContent = `${hiddenCount} ordinary stage${hiddenCount === 1 ? '' : 's'} folded into this chapter. Export Meta keeps the full lifecycle metadata.`;
    section.appendChild(note);
  }
  return section;
}

function renderChapterStageNode(segment) {
  const ordinal = segment.ordinal ?? segment.segmentIndex + 1;
  const primaryAgentId = lifecyclePrimaryAgentId(segment);
  const primaryAgentLabel = Number.isInteger(primaryAgentId) ? `A${primaryAgentId}` : 'A-';
  const taskText = segment.primaryTask ? `Primary ${segment.primaryTask.displayLabel}` : '';
  const detailText = [taskText, segment.debugReasons.join(' | ') || macroSpanSubtitle(segment)].filter(Boolean).join(' | ');
  const segmentColor = lifecycleSegmentColor(segment);
  const segmentSoftColor = hexToRgba(segmentColor, 0.08);
  const node = document.createElement('div');
  node.className = 'timelineEventNode';
  node.style.setProperty('--event-color', segmentColor);
  node.style.setProperty('--event-soft', segmentSoftColor);
  if (segment.segmentIndex === lifecycleSelection) node.classList.add('active');
  if (!segment.highSignal) node.classList.add('ordinary');
  if (segment.statusClass === 'danger' || segment.rejectedCount > 0) node.classList.add('rejected');
  if (segment.statusClass === 'warning') node.classList.add('warning');
  if (segment.statusClass === 'success') node.classList.add('success');
  if (markersInRange(segment.start, segment.end).length > 0) node.classList.add('marked');

  const button = document.createElement('button');
  button.type = 'button';
  button.className = 'timelineEventBox';
  button.dataset.lifecycleIndex = String(segment.segmentIndex);
  button.title = 'Select and jump to this stage';
  button.style.setProperty('--event-color', segmentColor);
  button.style.setProperty('--event-soft', segmentSoftColor);
  button.innerHTML = `
    <span class="timelineEventMeta">Stage ${ordinal}<br>${escapeHtml(segmentRangeText(segment))}</span>
    <span class="timelineEventMain">
      <strong>${escapeHtml(segment.label)}</strong>
      <small>${escapeHtml(detailText)}</small>
    </span>
    <span class="timelineEventSide">
      <span class="timelineAgentBadge">${escapeHtml(primaryAgentLabel)}</span>
      <span class="timelineStatusPill ${escapeHtml(segment.statusClass)}">${escapeHtml(segment.statusLabel)}</span>
    </span>
  `;
  node.appendChild(button);
  node.appendChild(lifecycleDetailCard(segment, ordinal));
  return node;
}

function chapterVisibleSegments(chapter) {
  const activeIndex = lifecycleSelection;
  const important = chapter.segments.filter(segment =>
    segment.highSignal || segment.segmentIndex === activeIndex || markersInRange(segment.start, segment.end).length > 0);
  if (important.length === 0) {
    const head = chapter.segments.slice(0, 3);
    const tail = chapter.segments.length > 3 ? [chapter.segments[chapter.segments.length - 1]] : [];
    return orderedSegments([...head, ...tail]);
  }
  const withContext = new Set(important);
  for (const segment of important) {
    const localIndex = chapter.segments.indexOf(segment);
    if (localIndex > 0) withContext.add(chapter.segments[localIndex - 1]);
    if (localIndex + 1 < chapter.segments.length) withContext.add(chapter.segments[localIndex + 1]);
  }
  return orderedSegments([...withContext]).slice(0, 7);
}

function orderedSegments(values) {
  return [...new Map(values.map(segment => [segment.segmentIndex, segment])).values()]
    .sort((a, b) => a.segmentIndex - b.segmentIndex);
}

function segmentDebugScore(segment, index, totalSegments, totalFrames) {
  let score = 0;
  if (segment.statusClass === 'danger') score += 28;
  if (segment.statusClass === 'warning') score += 16;
  if ((segment.rejectedCount || 0) > 0) score += 18 + Math.min(segment.rejectedCount, 8) * 3;
  if ((segment.eventCount || 0) > 0) score += Math.min(segment.eventCount, 8) * 2;
  const kinds = joinSet(segment.eventKinds || new Set(), 20).toLowerCase();
  const verdicts = joinSet(segment.verdicts || new Set(), 20).toLowerCase();
  const reasons = joinSet(segment.reasons || new Set(), 20).toLowerCase();
  if (/(rollback|failed|failure|reject|validation|regress|block|conflict|retry|bsp)/.test(`${kinds} ${verdicts} ${reasons}`)) score += 14;
  const duration = segment.end - segment.start + 1;
  if (duration > totalFrames * 0.18) score += 12;
  else if (duration > totalFrames * 0.08) score += 6;
  if (markersInRange(segment.start, segment.end).length > 0) score += 20;
  const outcome = String(replay.summary?.outcome || '').toLowerCase();
  if (!outcome.includes('solved') && index >= Math.max(0, totalSegments - 3)) score += 10;
  return score;
}

function segmentDebugReasons(segment, score, totalFrames) {
  const reasons = [];
  if (segment.statusClass === 'danger') reasons.push('critical signal');
  else if (segment.statusClass === 'warning') reasons.push('warning signal');
  if ((segment.rejectedCount || 0) > 0) reasons.push(`${segment.rejectedCount} rejected`);
  const eventText = joinSet(segment.eventKinds || new Set(), 3);
  if (eventText && eventText !== 'intent') reasons.push(eventText);
  const verdictText = joinSet(segment.verdicts || new Set(), 2);
  if (verdictText) reasons.push(verdictText);
  const duration = segment.end - segment.start + 1;
  if (duration > totalFrames * 0.08) reasons.push(`${duration} frames`);
  if (score < 18 && reasons.length === 0) reasons.push('ordinary execution');
  return reasons.slice(0, 4);
}

function chapterLabel(segments, index, best, hotspotCount) {
  if (hotspotCount > 0 && best) return best.label;
  if (index === 0) return 'Opening Plan';
  const labels = countValues(segments.map(segment => segment.label));
  let bestLabel = 'Execution';
  let bestCount = -1;
  for (const [label, count] of labels.entries()) {
    if (count > bestCount) {
      bestLabel = label;
      bestCount = count;
    }
  }
  return bestLabel;
}

function compactChapterLabel(label) {
  const words = String(label || 'Chapter').split(/\s+/).filter(Boolean);
  return words.slice(0, 2).join(' ') || 'Chapter';
}

function chapterStatusLabel(statusClass, hotspotCount) {
  if (statusClass === 'danger') return 'needs attention';
  if (statusClass === 'warning') return 'watch';
  if (hotspotCount > 0) return 'diagnostic';
  if (statusClass === 'success') return 'committed';
  return 'ordinary';
}

function chapterFocusText(segments, best, hotspotCount) {
  if (hotspotCount > 0 && best) {
    return `${hotspotCount} hotspot${hotspotCount === 1 ? '' : 's'}; first inspect ${best.label} at ${segmentRangeText(best)}.`;
  }
  const duration = segments.reduce((sum, segment) => sum + (segment.end - segment.start + 1), 0);
  return `No strong anomaly; ${segments.length} ordinary stage${segments.length === 1 ? '' : 's'} folded into ${duration} frame${duration === 1 ? '' : 's'}.`;
}

function chapterRangeText(chapter) {
  return chapter.start === chapter.end ? `step ${chapter.start}` : `${chapter.start}-${chapter.end}`;
}

function positionTimelineDropdown(node) {
  const card = node?.querySelector?.('.timelineEventDropdown');
  if (!card) return;
  const trigger = node.querySelector('.timelineEventBox') || node;
  clearTimelineDropdownPosition(node);
  requestAnimationFrame(() => {
    const margin = 12;
    card.style.visibility = 'hidden';
    card.style.display = 'grid';
    card.style.left = '0px';
    card.style.top = '0px';

    const triggerRect = trigger.getBoundingClientRect();
    const cardRect = card.getBoundingClientRect();
    const belowTop = triggerRect.bottom + 8;
    const aboveTop = triggerRect.top - cardRect.height - 8;
    let top = belowTop;
    if (belowTop + cardRect.height > window.innerHeight - margin && aboveTop >= margin) {
      top = aboveTop;
    } else {
      top = Math.min(Math.max(margin, top), window.innerHeight - margin - Math.min(cardRect.height, window.innerHeight - margin * 2));
    }

    let left = triggerRect.left;
    if (left + cardRect.width > window.innerWidth - margin) {
      left = window.innerWidth - margin - cardRect.width;
    }
    left = Math.max(margin, left);

    card.style.left = `${Math.round(left)}px`;
    card.style.top = `${Math.round(top)}px`;
    card.style.visibility = '';
  });
}

function clearTimelineDropdownPosition(node) {
  node?.classList?.remove('dropdownLeft', 'dropdownUp');
  const card = node?.querySelector?.('.timelineEventDropdown');
  if (!card) return;
  card.style.removeProperty('display');
  card.style.removeProperty('left');
  card.style.removeProperty('top');
  card.style.removeProperty('visibility');
}

function lifecycleDetailCard(segment, ordinal) {
  const card = document.createElement('article');
  card.className = 'timelineEventDropdown';
  const primaryAgentId = lifecyclePrimaryAgentId(segment);
  const primaryTask = segment.primaryTask;
  const nativeTransaction = timelineNativeTransaction(segment);
  const candidateSummary = timelineMetaCandidateSummary(segment);
  const supportSummary = timelineSupportSummaryText(timelineSupportRollup(segment));
  const decisionRows = timelineDecisionRowsHtml(segment, { includeSamples: true });
  card.innerHTML = `
    <div class="timelineDetailHeader">
      <strong>Stage ${ordinal}</strong>
      <span>${escapeHtml(segmentRangeText(segment))}</span>
    </div>
    <dl>
      <dt>Macro</dt><dd>${escapeHtml(segment.label)}</dd>
      <dt>Status</dt><dd>${escapeHtml(segment.statusLabel || 'active')}</dd>
      <dt>Primary Task</dt><dd>${escapeHtml(primaryTask ? primaryTask.displayLabel : 'none recorded')}</dd>
      <dt>Task Tx</dt><dd>${escapeHtml(primaryTaskTransactionText(primaryTask))}</dd>
      <dt>Executor</dt><dd>${escapeHtml(timelineSegmentExecutorText(segment))}</dd>
      <dt>Phase</dt><dd>${escapeHtml(joinSet(segment.phases) || 'execution')}</dd>
      <dt>Subgoal</dt><dd>${escapeHtml(segment.subgoal || 'none')}</dd>
      <dt>Subgoal Type</dt><dd>${escapeHtml(joinSet(segment.subgoalTypes) || 'none')}</dd>
      <dt>Primary</dt><dd>${escapeHtml(Number.isInteger(primaryAgentId) ? `agent${primaryAgentId}` : 'mixed/none')}</dd>
      <dt>Agents</dt><dd>${escapeHtml(joinAgents(segment.agentIds))}</dd>
      <dt>Actions</dt><dd>${escapeHtml(joinSet(segment.actionFamilies) || 'NoOp')}</dd>
      <dt>Events</dt><dd>${escapeHtml(joinSet(segment.eventKinds) || 'none')}</dd>
      <dt>Verdicts</dt><dd>${escapeHtml(joinSet(segment.verdicts || new Set()) || 'none')}</dd>
      <dt>Reasons</dt><dd>${escapeHtml(joinSet(segment.reasons || new Set()) || 'none')}</dd>
      <dt>Transaction</dt><dd>${escapeHtml(timelineTransactionSummaryText(nativeTransaction))}</dd>
      <dt>Candidates</dt><dd>${escapeHtml(timelineCandidateSummaryText(candidateSummary))}</dd>
      <dt>Support</dt><dd>${escapeHtml(supportSummary || 'none')}</dd>
      ${decisionRows}
      <dt>Rejected</dt><dd>${segment.rejectedCount}</dd>
      <dt>Children</dt><dd>${segment.eventCount || 0} decision / ${segment.intentCount || 0} intent</dd>
    </dl>
    ${timelineChildEventsHtml(segment)}
  `;
  return card;
}

function timelineDecisionRowsHtml(segment, options = {}) {
  const selection = timelineBestSubgoalSelection(segment);
  const ordering = timelineBestSubgoalOrdering(segment, selection);
  const rows = [];
  if (selection) {
    rows.push(['Selected', timelineSubgoalSelectionSummaryText(selection)]);
    rows.push(['Why selected', timelineFriendlySelectionBasis(selection)]);
    rows.push(['Gate checks', timelineFriendlySelectionGates(selection)]);
    if (options.includeSamples) rows.push(['Candidate pool', timelineFriendlySubgoalSample(selection.candidateStatusSample, { limit: 4, includeStatus: true })]);
  }
  if (ordering) {
    rows.push(['Priority order', timelineSubgoalOrderingSummaryText(ordering)]);
    rows.push(['Order rule', timelineFriendlyOrderingBasis(ordering)]);
    if (options.includeSamples) {
      rows.push(['Distance model', timelineFriendlyHeuristic(ordering.heuristicBasis || selection?.heuristicBasis)]);
      rows.push(['Order sample', timelineFriendlySubgoalSample(ordering.orderSample, { limit: 4 })]);
    }
  } else if (selection?.heuristicBasis && options.includeSamples) {
    rows.push(['Distance model', timelineFriendlyHeuristic(selection.heuristicBasis)]);
  }
  return rows
    .filter(([, value]) => value !== undefined && value !== null && value !== '')
    .map(([label, value]) => `<dt>${escapeHtml(label)}</dt><dd>${escapeHtml(formatEventValue(value))}</dd>`)
    .join('');
}

function timelineBestSubgoalSelection(segment) {
  const selections = timelineMetaSubgoalSelections(segment);
  if (selections.length === 0) return null;
  const primaryLabel = segment.primaryTask?.label || primaryTaskLabelFromSegment(segment);
  if (primaryLabel) {
    const match = selections.find(selection => {
      const subgoal = cleanPrimaryTaskLabel(selection.ownerSubgoal || selection.subgoal);
      return subgoal && subgoal === primaryLabel;
    });
    if (match) return match;
  }
  return selections[0];
}

function timelinePrimarySubgoalDecision(context, transaction = null, candidateGroup = null) {
  const selections = context?.selections || [];
  const orderings = context?.orderings || [];
  if (selections.length === 0 && orderings.length === 0) return { selection: null, ordering: null };
  const anchorStep = timelinePrimaryDecisionAnchorStep(transaction, candidateGroup, selections, orderings);
  const targetLabel = timelinePrimaryDecisionTargetLabel(transaction, candidateGroup);
  let selection = null;
  if (targetLabel) {
    selection = selections
      .filter(item => timelineSubgoalSelectionMatches(item, targetLabel))
      .sort((a, b) => Math.abs((a.step ?? 0) - anchorStep) - Math.abs((b.step ?? 0) - anchorStep))[0] || null;
  }
  if (!selection) {
    selection = selections
      .filter(item => (item.step ?? 0) <= anchorStep)
      .sort((a, b) => b.step - a.step)[0] || selections[0] || null;
  }
  const orderingAnchor = selection?.step ?? anchorStep;
  let ordering = orderings
    .filter(item => (item.step ?? 0) <= orderingAnchor)
    .sort((a, b) => b.step - a.step)[0] || orderings[0] || null;
  if (targetLabel) {
    const mentioning = orderings
      .filter(item => timelineOrderingMentions(item, targetLabel))
      .sort((a, b) => Math.abs((a.step ?? 0) - orderingAnchor) - Math.abs((b.step ?? 0) - orderingAnchor))[0] || null;
    if (mentioning) ordering = mentioning;
  }
  return { selection, ordering };
}

function timelinePrimaryDecisionAnchorStep(transaction, candidateGroup, selections, orderings) {
  if (candidateGroup) return timelineIssueStep(candidateGroup);
  if (transaction) return transaction.planStartStep ?? timelineIssueStep(transaction);
  return selections?.[0]?.step ?? orderings?.[0]?.step ?? 0;
}

function timelinePrimaryDecisionTargetLabel(transaction, candidateGroup) {
  const txLabel = cleanPrimaryTaskLabel(transaction?.taskLabel || transaction?.subgoal || '');
  if (txLabel) return txLabel;
  const candidateSubgoal = cleanPrimaryTaskLabel(candidateGroup?.subgoal || '');
  if (candidateSubgoal) return candidateSubgoal;
  if (candidateGroup?.agentId != null && candidateGroup?.boxType && candidateGroup?.goal) {
    const goal = timelineGoalText(candidateGroup.goal);
    if (goal) return cleanPrimaryTaskLabel(`agent${candidateGroup.agentId}->${candidateGroup.boxType}@${goal}`);
  }
  return '';
}

function timelineSubgoalSelectionMatches(selection, targetLabel) {
  if (!selection || !targetLabel) return false;
  const candidates = [
    selection.ownerSubgoal,
    selection.subgoal,
    selection.boxType && selection.goal ? `agent${selection.agentId}->${selection.boxType}@${timelineGoalText(selection.goal)}` : '',
  ].map(cleanPrimaryTaskLabel).filter(Boolean);
  return candidates.includes(targetLabel);
}

function timelineOrderingMentions(ordering, targetLabel) {
  if (!ordering || !targetLabel) return false;
  return formatEventValue(ordering.orderSample || '').replace(/\s+/g, '').includes(targetLabel);
}

function timelineBestSubgoalOrdering(segment, selection = null) {
  const localOrderings = timelineMetaSubgoalOrderings(segment);
  if (localOrderings.length > 0) {
    return nearestSubgoalOrdering(localOrderings, selection?.step ?? segment.start);
  }
  if (!selection) return null;
  const context = subgoalDecisionContext();
  const anchor = selection?.step ?? segment.start;
  const before = context.orderings.filter(ordering => ordering.step <= anchor);
  if (before.length > 0) return before[before.length - 1];
  return context.orderings[0] || null;
}

function nearestSubgoalOrdering(orderings, anchorStep) {
  let best = orderings[0] || null;
  let bestDistance = Infinity;
  for (const ordering of orderings) {
    const distance = Math.abs((ordering.step ?? 0) - (anchorStep ?? 0));
    if (distance < bestDistance) {
      best = ordering;
      bestDistance = distance;
    }
  }
  return best;
}

function timelineSubgoalSelectionSummaryText(selection) {
  if (!selection) return 'none';
  const pool = selection.candidateCount != null || selection.eligibleCount != null
    ? `${selection.eligibleCount ?? '?'} eligible of ${selection.candidateCount ?? '?'} candidates`
    : '';
  const parts = [
    `${timelineReadableSubgoalLabel(selection.subgoal)} was selected`,
    selection.selectionId ? `id ${selection.selectionId}` : '',
    selection.step != null ? `step ${selection.step}` : '',
    selection.selectedRank != null ? `rank ${selection.selectedRank}` : '',
    pool,
    selection.selectedEstimatedDifficulty ? `diff ${selection.selectedEstimatedDifficulty}` : '',
    selection.selectedUnmetDependencies != null ? `${selection.selectedUnmetDependencies} unmet dependencies` : '',
  ].filter(Boolean);
  return parts.join(' | ') || 'subgoal selection recorded';
}

function timelineSubgoalOrderingSummaryText(ordering) {
  if (!ordering) return 'none';
  const ordered = ordering.orderedSubgoalCount != null || ordering.rawSubgoalCount != null
    ? `${ordering.orderedSubgoalCount ?? '?'} of ${ordering.rawSubgoalCount ?? '?'} goals ordered`
    : '';
  const parts = [
    ordering.orderSnapshotId ? `id ${ordering.orderSnapshotId}` : '',
    timelineReadableOrderingMode(ordering.orderingMode),
    ordered,
    ordering.goalDependencyEdges != null ? `${ordering.goalDependencyEdges} dependency edges` : '',
    ordering.completedBoxGoals != null ? `${ordering.completedBoxGoals} completed goals` : '',
    ordering.transitDeferredToTail != null ? `${ordering.transitDeferredToTail} transit deferred` : '',
    ordering.physicalBlockingPromotions != null ? `${ordering.physicalBlockingPromotions} barrier promotions` : '',
  ].filter(Boolean);
  return parts.join(' | ') || 'subgoal ordering recorded';
}

function timelineFriendlySelectionBasis(selection) {
  const basis = String(selection?.selectionBasis || selection?.reason || '').toLowerCase();
  if (basis.includes('first-eligible')) {
    return 'The planner scans the current priority order and takes the first subgoal that passes all runtime gates.';
  }
  if (basis.includes('priority')) {
    return 'The selected subgoal is the highest-priority candidate that is currently allowed to run.';
  }
  return formatEventValue(selection?.selectionBasis || selection?.reason) || 'Selection basis was not recorded.';
}

function timelineFriendlySelectionGates(selection) {
  if (!selection) return '';
  const parts = [];
  if (selection.selectedUnmetDependencies != null) {
    parts.push(`${selection.selectedUnmetDependencies} unmet dependencies for the chosen subgoal`);
  }
  if (selection.selectionStatusCounts) parts.push(`pool status: ${selection.selectionStatusCounts}`);
  if (selection.selectedCompletionCount != null && selection.selectedCompletionCount > 0) {
    parts.push(`${selection.selectedCompletionCount} prerequisites already completed for this candidate`);
  }
  const raw = String(selection.filterBasis || '').toLowerCase();
  const gates = [];
  if (raw.includes('topology')) gates.push('topology dependencies');
  if (raw.includes('completed')) gates.push('already-completed goals');
  if (raw.includes('cycle')) gates.push('cycle limit');
  if (raw.includes('trap')) gates.push('trap blacklist');
  if (raw.includes('synthetic')) gates.push('synthetic-relief blacklist');
  if (raw.includes('relief certificate')) gates.push('relief certificate state');
  if (gates.length) parts.push(`checked ${gates.join(', ')}`);
  return parts.join(' | ') || formatEventValue(selection.filterBasis);
}

function timelineFriendlyOrderingBasis(ordering) {
  if (!ordering) return '';
  const mode = timelineReadableOrderingMode(ordering.orderingMode);
  const basis = String(ordering.orderingBasis || '').toLowerCase();
  if (basis.includes('levelanalyzer') || basis.includes('dependency-first')) {
    return `${mode} ordering from the LevelAnalyzer dependency graph; dependency-first goals stay ahead, while transit-dependent goals are pushed later.`;
  }
  if (basis.includes('topology')) {
    return `${mode} ordering based on topology dependencies.`;
  }
  return formatEventValue(ordering.orderingBasis || ordering.orderingMode);
}

function timelineFriendlyHeuristic(value) {
  const text = formatEventValue(value);
  if (!text) return '';
  const lower = text.toLowerCase();
  if (lower.includes('truedistanceheuristic') || lower.includes('true-distance')) {
    return 'Difficulty uses true distance when available, otherwise Manhattan. The score combines agent-to-box operation-side work and box-to-goal immovable-aware distance.';
  }
  return text;
}

function timelineFriendlySubgoalSample(value, options = {}) {
  const entries = parseSubgoalSampleEntries(value);
  if (entries.length === 0) return formatEventValue(value);
  const limit = Math.max(1, options.limit || 5);
  const shown = entries.slice(0, limit).map(entry => {
    const details = [];
    const status = options.includeStatus ? timelineFriendlySampleStatus(entry) : '';
    if (status) details.push(status);
    if (entry.diff) details.push(`diff ${entry.diff}`);
    if (entry.unmetDeps !== '') details.push(`deps ${entry.unmetDeps}`);
    if (entry.completions !== '' && entry.completions !== '0') details.push(`completed ${entry.completions}`);
    return `${entry.rank}. ${timelineReadableSubgoalLabel(entry.subgoal)}${details.length ? ` (${details.join(', ')})` : ''}`;
  });
  const overflow = entries.length > limit ? `; +${entries.length - limit} more` : '';
  return `${shown.join('; ')}${overflow}`;
}

function parseSubgoalSampleEntries(value) {
  const text = formatEventValue(value);
  if (!text) return [];
  return text.split(/\s*;\s*/).map(part => {
    const match = part.match(/^(\d+):([^\s]+)\s*(.*)$/);
    if (!match) return null;
    const fields = {};
    const rest = match[3] || '';
    for (const field of rest.matchAll(/([A-Za-z][A-Za-z0-9]*)=([^ ]+)/g)) {
      fields[field[1]] = field[2];
    }
    return {
      rank: match[1],
      subgoal: match[2],
      type: fields.type || '',
      diff: fields.diff || '',
      unmetDeps: fields.unmetDeps ?? '',
      completions: fields.completions ?? '',
      tags: fields.tags || '',
    };
  }).filter(Boolean);
}

function timelineFriendlySampleStatus(entry) {
  const tags = String(entry?.tags || '').toLowerCase();
  if (tags.includes('selected')) return 'selected';
  if (tags.includes('eligible-lower-priority')) return 'eligible, lower priority';
  if (tags.includes('transit')) return 'transit';
  if (tags.includes('failed')) return 'failed';
  return '';
}

function timelineReadableSubgoalLabel(value) {
  const label = cleanPrimaryTaskLabel(value);
  return label ? formatPrimaryTaskLabel(label) : formatEventValue(value || 'subgoal');
}

function timelineReadableOrderingMode(value) {
  const raw = formatEventValue(value);
  if (!raw) return 'priority';
  return raw.toLowerCase() === 'topological' ? 'topological' : raw.toLowerCase();
}

function timelineTransactionSummaryText(transaction) {
  if (!transaction) return 'none';
  const range = transaction.planStartStep != null && transaction.planEndStep != null
    ? `${transaction.planStartStep}-${transaction.planEndStep}`
    : '';
  const parts = [
    transaction.transactionId,
    transaction.commitStatus,
    range ? `plan ${range}` : '',
    transaction.planDelta != null ? `delta ${transaction.planDelta}` : '',
    transaction.supportKind,
  ].filter(Boolean);
  return parts.join(' | ') || 'native transaction';
}

function timelineCandidateSummaryText(candidates) {
  if (!Array.isArray(candidates) || candidates.length === 0) return 'none';
  const verdicts = countValues(candidates.map(item => item.verdict || 'unknown'));
  const verdictText = [...verdicts.entries()].map(([verdict, count]) => `${verdict}:${count}`).join(', ');
  const selected = candidates.find(item => item.selectedBox || item.hungarianAssignedBox);
  const selectedText = selected ? ` | box ${selected.selectedBox || selected.hungarianAssignedBox}` : '';
  return `${candidates.length} summary${candidates.length === 1 ? '' : 'ies'} | ${verdictText}${selectedText}`;
}

function timelineSupportSummaryText(rollup) {
  if (!rollup) return '';
  const range = rollup.firstStep != null && rollup.lastStep != null
    ? `steps ${rollup.firstStep}-${rollup.lastStep}`
    : '';
  const supportKinds = (rollup.supportKinds || [])
    .map(item => `${item.key}${item.count > 1 ? `:${item.count}` : ''}`)
    .join(', ');
  const owners = (rollup.ownerAgents || []).join(', ');
  const executors = (rollup.executorAgents || []).join(', ');
  const parts = [
    `${rollup.stepCount} support step${rollup.stepCount === 1 ? '' : 's'}`,
    range,
    owners ? `owner ${owners}` : '',
    executors ? `executor ${executors}` : '',
    supportKinds,
    rollup.movedBoxSteps ? `${rollup.movedBoxSteps} box-moving step${rollup.movedBoxSteps === 1 ? '' : 's'}` : '',
  ].filter(Boolean);
  return parts.join(' | ');
}

function timelineSegmentExecutorText(segment) {
  const rollup = timelineSupportRollup(segment);
  if (rollup?.executorAgents?.length) return rollup.executorAgents.join(', ');
  const primaryAgentId = lifecyclePrimaryAgentId(segment);
  if (Number.isInteger(primaryAgentId)) return `agent${primaryAgentId}`;
  return joinAgents(segment?.agentIds || []) || 'mixed/none';
}

function primaryTaskContext() {
  if (primaryTaskContextCache) return primaryTaskContextCache;
  const transactions = replayPlannerTransactions();
  const byLabel = new Map();
  for (const transaction of transactions) {
    if (!transaction.taskLabel) continue;
    const bucket = byLabel.get(transaction.taskLabel) || [];
    bucket.push(transaction);
    byLabel.set(transaction.taskLabel, bucket);
  }
  primaryTaskContextCache = { transactions, byLabel };
  return primaryTaskContextCache;
}

function replayPlannerTransactions() {
  if (!replay?.frames) return [];
  const transactions = [];
  for (let i = 0; i < replay.frames.length; i++) {
    for (const event of stepEvents(replay.frames[i], i)) {
      const kind = String(event?.kind || event?.type || '').toLowerCase();
      if (!kind.includes('planner-transaction')) continue;
      const taskLabel = primaryTaskLabelFromEvent(event, { allowSubgoalFallback: true });
      const start = numberOrNull(event.planStart ?? event.planStartStep) ?? i;
      const end = numberOrNull(event.planEnd ?? event.planEndStep) ?? i;
      const rawStatus = formatEventValue(event.commitStatus || event.verdict || '');
      const status = rawStatus ? rawStatus.toLowerCase() : 'transaction';
      transactions.push({
        step: i,
        transactionId: formatEventValue(event.transactionId) || `tx-${transactions.length + 1}`,
        taskLabel,
        agentId: eventAgentId(event),
        ownerAgentId: numberOrNull(event.ownerAgentId),
        ownerAgent: formatEventValue(event.ownerAgent),
        boxType: formatEventValue(event.boxType),
        goal: event.goal ?? null,
        phase: formatEventValue(event.phase),
        subgoal: formatEventValue(event.subgoal),
        planStartStep: Math.min(start, end),
        planEndStep: Math.max(start, end),
        planDelta: numberOrNull(event.planDelta),
        plannedPathSteps: numberOrNull(event.plannedPathSteps),
        supportKind: formatEventValue(event.supportKind),
        supportPlanStart: numberOrNull(event.supportPlanStart),
        supportPlanEnd: numberOrNull(event.supportPlanEnd),
        satisfiedBoxGoalsAfter: numberOrNull(event.satisfiedBoxGoalsAfter),
        commitStatus: status,
        reason: formatEventValue(event.reason || event.rollbackReason),
        statusClass: lifecycleStatusClass(event),
        rawEvent: compactMetaEvent(event),
      });
    }
  }
  return transactions.sort((a, b) =>
    a.planStartStep - b.planStartStep || a.planEndStep - b.planEndStep || a.transactionId.localeCompare(b.transactionId));
}

function primaryTaskLabelFromEvent(event, options = {}) {
  if (!event || typeof event !== 'object') return '';
  const owner = cleanPrimaryTaskLabel(event.ownerSubgoal || event.primaryTask || event.parentSubgoal);
  if (owner) return owner;
  const subgoal = formatEventValue(event.subgoal);
  const kind = String(event.kind || event.type || '').toLowerCase();
  const phase = String(event.phase || '').toUpperCase();
  const reason = String(event.reason || '').toLowerCase();
  const staged = cleanPrimaryTaskLabel(subgoal.replace(/^stage-for\s+/i, ''));
  if (/^stage-for\s+/i.test(subgoal) && staged) return staged;
  if (kind === 'support-blocker-step' && cleanPrimaryTaskLabel(subgoal)) return cleanPrimaryTaskLabel(subgoal);
  if (kind.includes('planner-transaction') && cleanPrimaryTaskLabel(subgoal)) return cleanPrimaryTaskLabel(subgoal);
  if ((phase === 'NORMAL' || phase === 'CLEARED' || reason === 'primary-subgoal' || reason === 'after-support-clearing')
      && cleanPrimaryTaskLabel(subgoal)) return cleanPrimaryTaskLabel(subgoal);
  if (options.allowSubgoalFallback && cleanPrimaryTaskLabel(subgoal)) return cleanPrimaryTaskLabel(subgoal);
  return '';
}

function primaryTaskLabelFromSegment(segment) {
  if (!segment) return '';
  for (const item of segment.children || []) {
    if (!isPrimaryTaskExecutionEvent(item.event)) continue;
    const label = primaryTaskLabelFromEvent(item.event);
    if (label) return label;
  }
  const subgoal = formatEventValue(segment.subgoal);
  if (/^stage-for\s+/i.test(subgoal)) return cleanPrimaryTaskLabel(subgoal.replace(/^stage-for\s+/i, ''));
  if (cleanPrimaryTaskLabel(subgoal) && !/^relief\s+agent\d+$/i.test(subgoal) && !/^park\s+agent\d+$/i.test(subgoal)) {
    return cleanPrimaryTaskLabel(subgoal);
  }
  return '';
}

function isPrimaryTaskExecutionEvent(event) {
  if (!event || typeof event !== 'object') return false;
  const kind = String(event.kind || event.type || '').toLowerCase();
  if (kind.includes('candidate-summary') || kind.includes('bsp') || kind.includes('scan')) return false;
  if (kind.includes('planner-transaction') || kind === 'support-blocker-step') return true;
  if (kind.includes('commit') || kind.includes('validation') || kind.includes('support')) return true;
  return false;
}

function cleanPrimaryTaskLabel(value) {
  const text = formatEventValue(value).trim();
  if (!text) return '';
  if (/^relief\s+agent\d+$/i.test(text) || /^park\s+agent\d+$/i.test(text)) return '';
  const match = text.match(/agent\d+\s*->\s*[^|\s]+/i);
  return match ? match[0].replace(/\s+/g, '') : '';
}

function formatPrimaryTaskLabel(label) {
  const raw = cleanPrimaryTaskLabel(label) || formatEventValue(label);
  const match = raw.match(/^agent(\d+)->(.+)$/i);
  if (!match) return raw || 'none';
  return `agent${match[1]} -> ${match[2]}`;
}

function primaryTaskOwnerAgentLabel(label, transaction = null) {
  if (transaction?.ownerAgent) return transaction.ownerAgent;
  if (Number.isInteger(transaction?.ownerAgentId)) return `agent${transaction.ownerAgentId}`;
  if (Number.isInteger(transaction?.agentId)) return `agent${transaction.agentId}`;
  const match = String(label || '').match(/^agent(\d+)->/i);
  return match ? `agent${match[1]}` : '';
}

function primaryTaskGoalLabel(label, transaction = null) {
  const goal = formatEventValue(transaction?.goal);
  if (goal) return goal;
  const match = String(label || '').match(/@\(([^)]+)\)/);
  return match ? `(${match[1]})` : '';
}

function transactionStatusClass(transaction) {
  if (!transaction) return 'note';
  if (transaction.statusClass) return transaction.statusClass;
  const status = String(transaction.commitStatus || '').toLowerCase();
  if (/roll|fail|reject/.test(status)) return 'danger';
  if (/retry|warn/.test(status)) return 'warning';
  if (/commit|success|accept/.test(status)) return 'success';
  return 'note';
}

function primaryTaskBandStatusClass(band) {
  const transactions = band?.transactions || [];
  if (transactions.length > 0) {
    const committed = transactions.some(tx => /commit|success|accept/.test(String(tx.commitStatus || '').toLowerCase()));
    const rolledBack = transactions.some(tx => /roll|fail|reject/.test(String(tx.commitStatus || '').toLowerCase()));
    if (rolledBack && committed) return 'warning';
    if (rolledBack) return 'danger';
    if (committed) return 'success';
  }
  return band?.statusClass || 'note';
}

function bestTransactionForRange(start, end, label = '') {
  const { transactions, byLabel } = primaryTaskContext();
  const candidates = label && byLabel.has(label) ? byLabel.get(label) : transactions;
  let best = null;
  let bestScore = -Infinity;
  const duration = Math.max(1, end - start + 1);
  for (const tx of candidates) {
    const overlap = Math.max(0, Math.min(end, tx.planEndStep) - Math.max(start, tx.planStartStep) + 1);
    const gap = overlap > 0 ? 0 : Math.min(Math.abs(tx.planStartStep - end), Math.abs(start - tx.planEndStep));
    if (overlap === 0 && (!label || gap > 40)) continue;
    const coverage = overlap / duration;
    if (!label && coverage < 0.5 && duration > 2) continue;
    let score = overlap * 100 - gap + (label && tx.taskLabel === label ? 50 : 0);
    if (!label && start === end) {
      if (tx.planStartStep === start && tx.planEndStep > start) score += 30;
      if (tx.planEndStep === start && tx.planStartStep < start) score -= 20;
    }
    if (score > bestScore) {
      best = tx;
      bestScore = score;
    }
  }
  return best;
}

function primaryTaskForSegment(segment) {
  const directLabel = primaryTaskLabelFromSegment(segment);
  const tx = bestTransactionForRange(segment.start, segment.end, directLabel);
  const label = directLabel || tx?.taskLabel || '';
  if (!label) return null;
  return {
    label,
    displayLabel: formatPrimaryTaskLabel(label),
    ownerAgent: primaryTaskOwnerAgentLabel(label, tx),
    goal: primaryTaskGoalLabel(label, tx),
    transaction: tx || null,
    inferred: !directLabel && Boolean(tx),
  };
}

function primaryTaskForStep(stepIndex, event = null) {
  const directLabel = primaryTaskLabelFromEvent(event);
  const tx = bestTransactionForRange(stepIndex, stepIndex, directLabel);
  const label = directLabel || tx?.taskLabel || '';
  if (!label) return null;
  return {
    label,
    displayLabel: formatPrimaryTaskLabel(label),
    ownerAgent: primaryTaskOwnerAgentLabel(label, tx),
    goal: primaryTaskGoalLabel(label, tx),
    transaction: tx || null,
    inferred: !directLabel && Boolean(tx),
  };
}

function primaryTaskTransactionText(task) {
  const tx = task?.transaction;
  if (!tx) return task?.inferred ? 'inferred from lifecycle span' : 'not recorded';
  const span = tx.planStartStep != null && tx.planEndStep != null
    ? `${tx.planStartStep}-${tx.planEndStep}`
    : '';
  return [
    tx.transactionId,
    tx.commitStatus,
    span ? `span ${span}` : '',
    tx.reason,
  ].filter(Boolean).join(' | ');
}

function primaryTaskBandsForTimeline(segments) {
  const sources = [];
  for (const tx of primaryTaskContext().transactions) {
    if (!tx.taskLabel) continue;
    sources.push({
      label: tx.taskLabel,
      start: tx.planStartStep,
      end: tx.planEndStep,
      statusClass: transactionStatusClass(tx),
      transactions: [tx],
      stageCount: 0,
    });
  }
  for (const segment of segments) {
    const task = segment.primaryTask || primaryTaskForSegment(segment);
    if (!task?.label) continue;
    sources.push({
      label: task.label,
      start: segment.start,
      end: segment.end,
      statusClass: segment.statusClass || 'note',
      transactions: task.transaction ? [task.transaction] : [],
      stageCount: 1,
    });
  }
  sources.sort((a, b) => a.start - b.start || a.end - b.end || a.label.localeCompare(b.label));
  const bands = [];
  for (const source of sources) {
    const existing = bands.find(band =>
      band.label === source.label && source.start <= band.end + 1 && source.end >= band.start - 1);
    if (existing) {
      existing.start = Math.min(existing.start, source.start);
      existing.end = Math.max(existing.end, source.end);
      existing.statusClass = strongerStatus(existing.statusClass, source.statusClass);
      existing.stageCount += source.stageCount || 0;
      for (const tx of source.transactions || []) {
        if (!existing.transactions.some(item => item.transactionId === tx.transactionId)) existing.transactions.push(tx);
      }
    } else {
      bands.push({
        ...source,
        displayLabel: formatPrimaryTaskLabel(source.label),
        ownerAgent: primaryTaskOwnerAgentLabel(source.label, source.transactions?.[0]),
      });
    }
  }
  return bands
    .sort((a, b) => a.start - b.start || a.end - b.end || a.label.localeCompare(b.label))
    .map((band, index) => ({
      ...band,
      index,
      statusClass: primaryTaskBandStatusClass(band),
      transactions: [...(band.transactions || [])].sort((a, b) =>
        a.planStartStep - b.planStartStep || a.planEndStep - b.planEndStep || a.transactionId.localeCompare(b.transactionId)),
    }));
}

function lifecycleSegmentColor(segment) {
  const primaryAgentId = lifecyclePrimaryAgentId(segment);
  if (Number.isInteger(primaryAgentId)) return agentColor(primaryAgentId);
  return '#b59a4a';
}

function lifecyclePrimaryAgentId(segment) {
  if (segment?.agentCounts instanceof Map && segment.agentCounts.size > 0) {
    let bestId = null;
    let bestCount = -1;
    for (const [id, count] of segment.agentCounts.entries()) {
      if (count > bestCount || (count === bestCount && (bestId == null || id < bestId))) {
        bestId = id;
        bestCount = count;
      }
    }
    return bestId;
  }
  const ids = [...(segment?.agentIds || [])].sort((a, b) => a - b);
  return ids.length > 0 ? ids[0] : null;
}

function hexToRgba(hex, alpha) {
  const match = String(hex || '').match(/^#?([0-9a-f]{6})$/i);
  if (!match) return `rgba(181, 154, 74, ${alpha})`;
  const value = match[1];
  const r = parseInt(value.slice(0, 2), 16);
  const g = parseInt(value.slice(2, 4), 16);
  const b = parseInt(value.slice(4, 6), 16);
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function plannerLifecycleSpans() {
  const segments = [];
  const open = new Map();
  const looseEvents = [];
  for (let i = 0; i < replay.frames.length; i++) {
    const frame = replay.frames[i];
    const events = stepEvents(frame, i);
    for (const event of events) {
      if (isAgentIntentEvent(event)) {
        const descriptor = lifecycleDescriptorFromIntent(event);
        const current = open.get(descriptor.key);
        if (current && current.end >= i - 1) {
          current.end = i;
          mergeLifecycleStats(current, descriptor);
          addSpanEvent(current, event, i);
        } else {
          const next = createLifecycleSpan(descriptor, i, i);
          addSpanEvent(next, event, i);
          segments.push(next);
          open.set(descriptor.key, next);
        }
      } else if (!isAgentActionEvent(event) && isPlannerLifecycleEvent(event)) {
        looseEvents.push({ event, step: i });
      }
    }
  }

  for (const item of looseEvents) {
    const target = bestLifecycleSpanForEvent(segments, item.event, item.step);
    if (target) {
      mergeLifecycleStats(target, lifecycleDescriptorFromEvent(item.event));
      target.start = Math.min(target.start, item.step);
      target.end = Math.max(target.end, item.step);
      addSpanEvent(target, item.event, item.step);
    } else if (isStandaloneLifecycleEvent(item.event)) {
      const span = createLifecycleSpan(lifecycleDescriptorFromEvent(item.event), item.step, item.step);
      addSpanEvent(span, item.event, item.step);
      segments.push(span);
    }
  }

  if (segments.length === 0 && replay.frames.length > 0) {
    segments.push(createFallbackExecutionSpan());
  }

  for (const segment of segments) finalizeLifecycleSpan(segment);
  segments.sort((a, b) => a.start - b.start || a.end - b.end || a.label.localeCompare(b.label));
  return segments;
}

function createLifecycleSpan(descriptor, start, end) {
  return {
    ...descriptor,
    start,
    end,
    phases: new Set(descriptor.phases),
    subgoalTypes: new Set(descriptor.subgoalTypes),
    agentIds: new Set(descriptor.agentIds),
    agentCounts: new Map(descriptor.agentCounts),
    actionFamilies: new Set(descriptor.actionFamilies),
    eventKinds: new Set(descriptor.eventKinds),
    verdicts: new Set(descriptor.verdicts),
    reasons: new Set(descriptor.reasons),
    samples: [...descriptor.samples],
    events: [],
    children: [],
    eventCount: 0,
    intentCount: 0,
    rejectedCount: descriptor.rejectedCount || 0,
    statusClass: descriptor.statusClass || 'note',
    statusLabel: descriptor.statusLabel || 'active',
  };
}

function lifecycleDescriptorFromIntent(event) {
  const phase = formatEventValue(event.phase) || 'execution';
  const subgoalType = formatEventValue(event.subgoalType || event.activeSubgoal);
  const subgoal = formatEventValue(event.subgoal || event.goal || event.title || subgoalType);
  const agentId = eventAgentId(event);
  const action = formatEventValue(event.actualAction || event.action);
  const reason = formatEventValue(event.reason);
  const label = lifecycleMacroLabel(event);
  const key = ['intent', label, phase, agentId ?? '-', subgoalType, subgoal].join('|');
  const agentIdSamples = Number.isInteger(agentId) ? [agentId] : [];
  const agentIdsForStep = orderedUnique(agentIdSamples);
  return {
    key,
    label,
    subgoal,
    rejectedCount: 0,
    statusClass: 'note',
    statusLabel: 'active',
    phases: [phase],
    subgoalTypes: subgoalType ? [subgoalType] : [],
    agentIds: agentIdsForStep,
    agentCounts: countValues(agentIdSamples),
    actionFamilies: action ? [actionFamily(action)] : [],
    eventKinds: ['intent'],
    verdicts: [],
    reasons: reason ? [reason] : [],
    samples: orderedUnique([subgoal, reason, formatEventValue(event.message)].filter(Boolean)),
  };
}

function lifecycleDescriptorFromEvent(event) {
  const kind = compactEventKind(String(event.kind || event.type || event.category || '').toLowerCase());
  const phase = formatEventValue(event.phase);
  const subgoalType = formatEventValue(event.subgoalType || event.activeSubgoal);
  const subgoal = formatEventValue(event.subgoal || event.goal || event.title || event.message);
  const agentId = eventAgentId(event);
  const reason = formatEventValue(event.reason);
  const verdict = formatEventValue(event.verdict);
  const action = formatEventValue(event.actualAction || event.action);
  const statusClass = lifecycleStatusClass(event);
  const label = lifecycleMacroLabel(event);
  const key = ['event', label, phase, agentId ?? '-', subgoalType, subgoal, kind].join('|');
  const agentIdSamples = Number.isInteger(agentId) ? [agentId] : [];
  return {
    key,
    label,
    subgoal,
    rejectedCount: event.accepted === false || kind === 'rejected' ? 1 : 0,
    statusClass,
    statusLabel: lifecycleStatusLabel(statusClass, event),
    phases: phase ? [phase] : [],
    subgoalTypes: subgoalType ? [subgoalType] : [],
    agentIds: orderedUnique(agentIdSamples),
    agentCounts: countValues(agentIdSamples),
    actionFamilies: action ? [actionFamily(action)] : [],
    eventKinds: kind ? [kind] : [],
    verdicts: verdict ? [verdict] : [],
    reasons: reason ? [reason] : [],
    samples: orderedUnique([event.title, event.message, reason, verdict, subgoal].map(formatEventValue).filter(Boolean)),
  };
}

function createFallbackExecutionSpan() {
  const descriptor = {
    key: 'fallback|execution',
    label: 'Execution Replay',
    subgoal: '',
    rejectedCount: 0,
    statusClass: 'note',
    statusLabel: 'replay',
    phases: ['execution'],
    subgoalTypes: [],
    agentIds: [],
    agentCounts: new Map(),
    actionFamilies: [],
    eventKinds: [],
    verdicts: [],
    reasons: [],
    samples: ['No planner lifecycle events were found.'],
  };
  return createLifecycleSpan(descriptor, 0, Math.max(0, replay.frames.length - 1));
}

function mergeLifecycleStats(target, source) {
  target.rejectedCount += source.rejectedCount || 0;
  addValues(target.phases, source.phases);
  addValues(target.subgoalTypes, source.subgoalTypes);
  addValues(target.agentIds, source.agentIds);
  mergeCounts(target.agentCounts, source.agentCounts);
  addValues(target.actionFamilies, source.actionFamilies);
  addValues(target.eventKinds, source.eventKinds);
  addValues(target.verdicts, source.verdicts || []);
  addValues(target.reasons, source.reasons || []);
  target.statusClass = strongerStatus(target.statusClass, source.statusClass);
  target.statusLabel = lifecycleStatusLabel(target.statusClass, source);
  if (!target.subgoal && source.subgoal) target.subgoal = source.subgoal;
  for (const sample of source.samples || []) {
    if (target.samples.length >= 8) break;
    if (!target.samples.includes(sample)) target.samples.push(sample);
  }
}

function addSpanEvent(segment, event, stepIndex) {
  if (isAgentIntentEvent(event)) {
    segment.intentCount++;
    return;
  }
  segment.eventCount++;
  segment.children.push({ step: stepIndex, event });
  if (segment.events.length < MAX_TIMELINE_CHILD_EVENTS) {
    segment.events.push({ step: stepIndex, event });
  }
}

function finalizeLifecycleSpan(segment) {
  for (let i = segment.start; i <= segment.end; i++) {
    const frame = replay.frames[i];
    if (!frame) continue;
    for (const action of frame.actions || []) {
      if (action && action !== 'NoOp') segment.actionFamilies.add(actionFamily(action));
    }
    if ((frame.accepted || []).some(value => value === false)) {
      segment.rejectedCount++;
      segment.statusClass = strongerStatus(segment.statusClass, 'danger');
    }
  }
  if (segment.rejectedCount > 0) {
    segment.statusClass = strongerStatus(segment.statusClass, 'danger');
  }
  segment.statusLabel = lifecycleStatusLabel(segment.statusClass, segment);
}

function bestLifecycleSpanForEvent(segments, event, stepIndex) {
  let best = null;
  let bestScore = 0;
  for (const segment of segments) {
    if (stepIndex < segment.start - 1 || stepIndex > segment.end + 1) continue;
    let score = 1;
    const eventSubgoal = formatEventValue(event.subgoal || event.goal || event.title);
    const eventPhase = formatEventValue(event.phase);
    const agentId = eventAgentId(event);
    if (eventSubgoal && segment.subgoal && eventSubgoal === segment.subgoal) score += 5;
    if (Number.isInteger(agentId) && segment.agentIds.has(agentId)) score += 3;
    if (eventPhase && segment.phases.has(eventPhase)) score += 2;
    if (score > bestScore) {
      best = segment;
      bestScore = score;
    }
  }
  return bestScore >= 3 ? best : null;
}

function isPlannerLifecycleEvent(event) {
  if (!event || typeof event !== 'object') return false;
  const kind = String(event.kind || event.type || event.category || '').toLowerCase();
  if (!kind) return false;
  return kind !== 'agent-action' && kind !== 'rejected-action' && kind !== 'agent-intent';
}

function isStandaloneLifecycleEvent(event) {
  if (!isPlannerLifecycleEvent(event)) return false;
  const kind = String(event.kind || event.type || event.category || '').toLowerCase();
  if (kind === 'support-blocker-step' || kind === 'task-relief-bsp-failed') return false;
  const severity = String(event.severity || event.level || '').toLowerCase();
  return severity.includes('warn') || severity.includes('error') || severity.includes('fail')
      || kind.includes('relief') || kind.includes('bsp') || kind.includes('rollback')
      || kind.includes('validation') || kind.includes('selection') || kind.includes('subgoal')
      || kind.includes('regress') || kind.includes('seal') || kind.includes('support')
      || kind.includes('transaction') || kind.includes('candidate');
}

function lifecycleMacroLabel(event) {
  const phase = String(event.phase || '').toUpperCase();
  const kind = String(event.kind || event.type || event.category || '').toLowerCase();
  const subgoalType = String(event.subgoalType || '').toLowerCase();
  if (phase.includes('TASK-RELIEF') || phase.includes('SCOPED-RELIEF') || phase.includes('HELPER-ACCESS')
      || kind.includes('task-relief') || kind.includes('relief')) return 'Task Relief';
  if (phase.includes('PARKING') || kind.includes('parking')) return 'Parking';
  if (phase.includes('SEAL') || kind.includes('seal')) return 'Seal Stage';
  if (phase.includes('CYCLE') || subgoalType.includes('escape')) return 'Cycle Break';
  if (phase.includes('CLEARED') || kind.includes('support-blocker')) return 'Path Clearing';
  if (phase.includes('PARALLEL')) return 'Parallel Execution';
  if (kind.includes('planner-transaction')) return 'Planner Transaction';
  if (kind.includes('subgoal-selection')) return 'Subgoal Selection';
  if (kind.includes('subgoal-ordering')) return 'Subgoal Ordering';
  if (kind.includes('candidate-summary')) return 'Candidate Summary';
  if (kind.includes('box-selection')) return 'Box Selection';
  if (kind.includes('bsp')) return 'BSP Search';
  if (kind.includes('rollback')) return 'Rollback';
  if (kind.includes('validation')) return 'Validation';
  if (kind.includes('subgoal-eval')) return 'Subgoal Evaluation';
  if (subgoalType.includes('synthetic-relief')) return 'Synthetic Relief';
  if (phase && phase !== 'NORMAL') return titleCase(phase);
  return 'Subgoal Execution';
}

function lifecycleStatusClass(event) {
  const kind = String(event.kind || event.type || event.category || '').toLowerCase();
  const severity = String(event.severity || event.level || '').toLowerCase();
  const verdict = String(event.verdict || '').toLowerCase();
  const commitStatus = String(event.commitStatus || '').toLowerCase();
  if (event.accepted === false || severity.includes('error') || severity.includes('fail')
      || kind.includes('failed') || kind.includes('rollback') || kind.includes('reject')
      || verdict.includes('fail') || verdict.includes('roll') || verdict.includes('cycle') || verdict.includes('no_')
      || commitStatus.includes('roll') || commitStatus.includes('fail')) return 'danger';
  if (severity.includes('warn') || kind.includes('retry') || kind.includes('validation')
      || kind.includes('scan') || verdict.includes('warning') || verdict.includes('retry')
      || commitStatus.includes('retry')) return 'warning';
  if (severity.includes('info') || kind.includes('commit') || verdict.includes('accepted')
      || verdict.includes('resolved') || verdict.includes('opened') || verdict.includes('committed')
      || commitStatus.includes('committed')) return 'success';
  return 'note';
}

function strongerStatus(a, b) {
  const rank = { danger: 4, warning: 3, success: 2, note: 1 };
  return (rank[b] || 1) > (rank[a] || 1) ? b : a;
}

function lifecycleStatusLabel(statusClass, source) {
  const verdict = source?.commitStatus || source?.verdict || (source?.verdicts instanceof Set ? joinSet(source.verdicts, 1) : '');
  if (verdict) return String(verdict).toLowerCase();
  if (statusClass === 'danger') return 'needs attention';
  if (statusClass === 'warning') return 'warning';
  if (statusClass === 'success') return 'committed';
  return 'active';
}

function macroSpanSubtitle(segment) {
  const parts = [];
  if (segment.subgoal) parts.push(segment.subgoal);
  const phase = joinSet(segment.phases, 1);
  const agents = joinAgents(segment.agentIds, 3);
  const reasons = joinSet(segment.reasons, 1);
  if (phase) parts.push(phase);
  if (agents) parts.push(agents);
  if (reasons) parts.push(reasons);
  if (segment.eventCount > 0) parts.push(`${segment.eventCount} child event${segment.eventCount === 1 ? '' : 's'}`);
  if (segment.intentCount > 0 && segment.eventCount === 0) parts.push(`${segment.intentCount} intent frame${segment.intentCount === 1 ? '' : 's'}`);
  return parts.join(' | ') || 'planner macro span';
}

function timelineChildEventsHtml(segment) {
  if (!segment.events || segment.events.length === 0) {
    const intentText = segment.intentCount ? ` The span was inferred from ${segment.intentCount} intent frame${segment.intentCount === 1 ? '' : 's'}.` : '';
    return `<p class="timelineChildEmpty">No decision child events captured for this span.${intentText}</p>`;
  }
  const items = segment.events.map(item => {
    const normalized = normalizeEvent(item.event);
    return `<li class="${escapeHtml(normalized.kindClass)}">
      <span>step ${item.step}</span>
      <strong>${escapeHtml(normalized.title)}</strong>
      <small>${escapeHtml(normalized.message || normalized.kind)}</small>
    </li>`;
  }).join('');
  const overflow = segment.eventCount > segment.events.length
    ? `<p class="timelineChildEmpty">+${segment.eventCount - segment.events.length} more child events in this span.</p>`
    : '';
  return `<ol class="timelineChildEvents">${items}</ol>${overflow}`;
}

function timelineMetaSpan(segment, index, allSegments = []) {
  const primaryAgentId = lifecyclePrimaryAgentId(segment);
  const decisionChildren = segment.children || [];
  const stateDiff = timelineSpanStateDiff(segment);
  const primaryTask = segment.primaryTask || primaryTaskForSegment(segment);
  return {
    ordinal: index + 1,
    label: segment.label,
    status: segment.statusLabel || 'active',
    statusClass: segment.statusClass || 'note',
    startStep: segment.start,
    endStep: segment.end,
    frameCount: segment.end - segment.start + 1,
    primaryAgentId: Number.isInteger(primaryAgentId) ? primaryAgentId : null,
    primaryTask: primaryTask ? {
      label: primaryTask.label,
      displayLabel: primaryTask.displayLabel,
      ownerAgent: primaryTask.ownerAgent,
      goal: primaryTask.goal,
      inferred: Boolean(primaryTask.inferred),
      transaction: primaryTask.transaction ? {
        transactionId: primaryTask.transaction.transactionId,
        commitStatus: primaryTask.transaction.commitStatus,
        reason: primaryTask.transaction.reason,
        planStartStep: primaryTask.transaction.planStartStep,
        planEndStep: primaryTask.transaction.planEndStep,
      } : null,
    } : null,
    agentIds: sortedArray(segment.agentIds),
    phases: sortedArray(segment.phases),
    subgoal: segment.subgoal || '',
    subgoalTypes: sortedArray(segment.subgoalTypes),
    actionFamilies: sortedArray(segment.actionFamilies),
    eventKinds: sortedArray(segment.eventKinds),
    verdicts: sortedArray(segment.verdicts),
    reasons: sortedArray(segment.reasons),
    rejectedCount: segment.rejectedCount || 0,
    intentFrameCount: segment.intentCount || 0,
    decisionEventCount: segment.eventCount || 0,
    decisionEventOverflow: Math.max(0, decisionChildren.length - MAX_EXPORTED_DECISION_EVENTS),
    samples: [...(segment.samples || [])],
    stateDiff,
    transaction: timelineMetaTransaction(segment, index, allSegments, stateDiff),
    transactions: timelineNativeTransactions(segment),
    candidateSummary: timelineMetaCandidateSummary(segment),
    subgoalSelections: timelineMetaSubgoalSelections(segment),
    subgoalOrderings: timelineMetaSubgoalOrderings(segment),
    decisionEvents: decisionChildren.slice(0, MAX_EXPORTED_DECISION_EVENTS).map(timelineMetaEvent),
  };
}

function timelineMetaChapter(chapter) {
  return {
    ordinal: chapter.index + 1,
    label: chapter.label,
    status: chapter.statusLabel,
    statusClass: chapter.statusClass,
    startStep: chapter.start,
    endStep: chapter.end,
    frameCount: chapter.end - chapter.start + 1,
    hotspotCount: chapter.hotspotCount,
    stageCount: chapter.stageCount,
    primaryAgentId: Number.isInteger(chapter.primaryAgentId) ? chapter.primaryAgentId : null,
    focus: chapter.focusText,
    stageOrdinals: chapter.segments.map(segment => segment.ordinal ?? segment.segmentIndex + 1),
  };
}

function timelinePortfolioSummary(attempts) {
  const rows = Array.isArray(attempts) ? attempts : [];
  const phases = new Map();
  let totalDurationMs = 0;
  let bestPlanSteps = 0;
  let successfulAttemptOrdinal = null;
  for (const attempt of rows) {
    const phase = formatEventValue(attempt.phase) || 'unknown';
    const entry = phases.get(phase) || { phase, attempts: 0, durationMs: 0, bestPlanSteps: 0, successes: 0 };
    entry.attempts++;
    entry.durationMs += Number(attempt.durationMs) || 0;
    entry.bestPlanSteps = Math.max(entry.bestPlanSteps, Number(attempt.planSteps) || 0);
    if (attempt.success === true) entry.successes++;
    phases.set(phase, entry);
    totalDurationMs += Number(attempt.durationMs) || 0;
    bestPlanSteps = Math.max(bestPlanSteps, Number(attempt.planSteps) || 0);
    if (attempt.success === true && successfulAttemptOrdinal == null) {
      successfulAttemptOrdinal = Number(attempt.ordinal) || null;
    }
  }
  return {
    attemptCount: rows.length,
    totalDurationMs,
    bestPlanSteps,
    successfulAttemptOrdinal,
    phases: [...phases.values()],
    attempts: rows.map(compactPortfolioAttempt),
  };
}

function compactPortfolioAttempt(attempt) {
  return {
    ordinal: Number(attempt.ordinal) || null,
    phase: formatEventValue(attempt.phase),
    label: formatEventValue(attempt.label),
    strategy: formatEventValue(attempt.strategy),
    orderingMode: formatEventValue(attempt.orderingMode),
    randomSeed: Number(attempt.randomSeed) || 0,
    durationMs: Number(attempt.durationMs) || 0,
    success: attempt.success === true,
    planSteps: Number(attempt.planSteps) || 0,
    unsatCount: Number.isFinite(Number(attempt.unsatCount)) ? Number(attempt.unsatCount) : null,
    failedSubgoal: formatEventValue(attempt.failedSubgoal),
    failureKind: formatEventValue(attempt.failureKind),
    reliefCount: Number(attempt.reliefCount) || 0,
    suspendedCount: Number(attempt.suspendedCount) || 0,
    finalSatisfiedGoals: Number.isFinite(Number(attempt.finalSatisfiedGoals)) ? Number(attempt.finalSatisfiedGoals) : null,
    finalTotalGoals: Number.isFinite(Number(attempt.finalTotalGoals)) ? Number(attempt.finalTotalGoals) : null,
    finalSatisfiedBoxGoals: Number.isFinite(Number(attempt.finalSatisfiedBoxGoals)) ? Number(attempt.finalSatisfiedBoxGoals) : null,
    finalTotalBoxGoals: Number.isFinite(Number(attempt.finalTotalBoxGoals)) ? Number(attempt.finalTotalBoxGoals) : null,
    finalUnsatisfiedGoalsSample: formatEventValue(attempt.finalUnsatisfiedGoalsSample),
    finalStateHash: formatEventValue(attempt.finalStateHash),
  };
}

function timelineMetaPrimaryTaskBand(band) {
  return {
    ordinal: band.index + 1,
    label: band.label,
    displayLabel: band.displayLabel,
    ownerAgent: band.ownerAgent,
    statusClass: band.statusClass || 'note',
    startStep: band.start,
    endStep: band.end,
    frameCount: band.end - band.start + 1,
    stageCount: band.stageCount || 0,
    transactions: (band.transactions || []).map(tx => ({
      transactionId: tx.transactionId,
      commitStatus: tx.commitStatus,
      reason: tx.reason,
      planStartStep: tx.planStartStep,
      planEndStep: tx.planEndStep,
      supportKind: tx.supportKind,
      satisfiedBoxGoalsAfter: tx.satisfiedBoxGoalsAfter,
    })),
  };
}

function timelineSpanStateDiff(segment) {
  const beforeIndex = Math.max(0, (segment.start || 0) - 1);
  const afterIndex = Math.max(0, segment.end || 0);
  const before = replay.frames[beforeIndex] || replay.frames[0];
  const after = replay.frames[afterIndex] || before;
  const beforeAgents = frameAgentMap(before);
  const afterAgents = frameAgentMap(after);
  const beforeBoxes = frameBoxMap(before);
  const afterBoxes = frameBoxMap(after);
  const changedAgents = [];
  for (const id of orderedUnique([...beforeAgents.keys(), ...afterAgents.keys()]).sort((a, b) => Number(a) - Number(b))) {
    const from = beforeAgents.get(id) || null;
    const to = afterAgents.get(id) || null;
    if (from !== to) changedAgents.push({ agentId: Number(id), from, to });
  }

  const removed = [];
  const added = [];
  for (const [pos, type] of beforeBoxes.entries()) {
    if (afterBoxes.get(pos) !== type) removed.push({ type, pos });
  }
  for (const [pos, type] of afterBoxes.entries()) {
    if (beforeBoxes.get(pos) !== type) added.push({ type, pos });
  }
  const paired = pairMovedBoxes(removed, added);
  const beforeGoals = satisfiedBoxGoalSet(before);
  const afterGoals = satisfiedBoxGoalSet(after);
  const gainedGoals = [...afterGoals].filter(key => !beforeGoals.has(key));
  const lostGoals = [...beforeGoals].filter(key => !afterGoals.has(key));
  return {
    beforeStep: beforeIndex,
    afterStep: afterIndex,
    changedAgentCount: changedAgents.length,
    changedAgents: changedAgents.slice(0, MAX_STATE_DIFF_ITEMS),
    changedAgentOverflow: Math.max(0, changedAgents.length - MAX_STATE_DIFF_ITEMS),
    movedBoxCount: paired.moved.length,
    movedBoxes: paired.moved.slice(0, MAX_STATE_DIFF_ITEMS),
    movedBoxOverflow: Math.max(0, paired.moved.length - MAX_STATE_DIFF_ITEMS),
    addedBoxes: paired.added.slice(0, MAX_STATE_DIFF_ITEMS),
    addedBoxOverflow: Math.max(0, paired.added.length - MAX_STATE_DIFF_ITEMS),
    removedBoxes: paired.removed.slice(0, MAX_STATE_DIFF_ITEMS),
    removedBoxOverflow: Math.max(0, paired.removed.length - MAX_STATE_DIFF_ITEMS),
    satisfiedBoxGoalsBefore: beforeGoals.size,
    satisfiedBoxGoalsAfter: afterGoals.size,
    gainedBoxGoals: gainedGoals.slice(0, MAX_STATE_DIFF_ITEMS),
    gainedBoxGoalOverflow: Math.max(0, gainedGoals.length - MAX_STATE_DIFF_ITEMS),
    lostBoxGoals: lostGoals.slice(0, MAX_STATE_DIFF_ITEMS),
    lostBoxGoalOverflow: Math.max(0, lostGoals.length - MAX_STATE_DIFF_ITEMS),
  };
}

function timelineMetaTransaction(segment, index, allSegments, stateDiff) {
  const native = timelineNativeTransaction(segment);
  const parentIndex = inferParentSegmentIndex(segment, index, allSegments);
  const supportRollup = timelineSupportRollup(segment);
  const bspFailureSummary = timelineBspFailureSummary(segment);
  const inferred = {
    transactionId: `tx-${index + 1}`,
    parentId: parentIndex == null ? null : `tx-${parentIndex + 1}`,
    phase: joinSet(segment.phases, 2) || 'execution',
    subgoal: segment.subgoal || '',
    subgoalTypes: sortedArray(segment.subgoalTypes),
    planStartStep: segment.start,
    planEndStep: segment.end,
    commitStatus: transactionCommitStatus(segment),
    rollbackReason: transactionRollbackReason(segment),
    touchedAgents: sortedArray(segment.agentIds),
    touchedBoxTypes: sortedArray(touchedBoxTypes(segment, stateDiff)),
    beforeSatisfiedBoxGoals: stateDiff.satisfiedBoxGoalsBefore,
    afterSatisfiedBoxGoals: stateDiff.satisfiedBoxGoalsAfter,
    supportRollup,
    bspFailureSummary,
  };
  if (!native) return { source: 'inferred', ...inferred };
  return {
    source: 'planner-trace',
    ...inferred,
    ...native,
    beforeSatisfiedBoxGoals: stateDiff.satisfiedBoxGoalsBefore,
    afterSatisfiedBoxGoals: stateDiff.satisfiedBoxGoalsAfter,
  };
}

function timelineNativeTransaction(segment) {
  const transactions = timelineNativeTransactions(segment);
  if (transactions.length === 0) return null;
  return transactions.find(tx => /roll|fail|reject/.test(String(tx.commitStatus || '').toLowerCase()))
    || transactions.find(tx => String(tx.commitStatus || '').toLowerCase().includes('committed'))
    || transactions[transactions.length - 1];
}

function timelineNativeTransactions(segment) {
  return timelineNativeTransactionEvents(segment)
    .map(({ item, event }) => timelineNativeTransactionFromEvent(item, event, segment));
}

function timelineNativeTransactionEvents(segment) {
  const children = Array.isArray(segment.children) ? segment.children : [];
  return children
    .map(item => ({ item, event: item.event || {} }))
    .filter(({ event }) => String(event.kind || '').toLowerCase().includes('planner-transaction'));
}

function timelineNativeTransactionFromEvent(item, event, segment) {
  const status = formatEventValue(event.commitStatus || event.verdict || transactionCommitStatus(segment));
  return {
    source: 'planner-trace',
    step: item.step,
    transactionId: formatEventValue(event.transactionId) || `tx-${item.step}`,
    parentId: formatEventValue(event.parentTransactionId) || null,
    agentId: eventAgentId(event),
    ownerAgentId: numberOrNull(event.ownerAgentId),
    ownerAgent: formatEventValue(event.ownerAgent),
    executorAgentId: numberOrNull(event.executorAgentId),
    executorAgent: formatEventValue(event.executorAgent),
    boxType: formatEventValue(event.boxType),
    goal: event.goal ?? null,
    phase: formatEventValue(event.phase) || joinSet(segment.phases, 2) || 'execution',
    subgoal: formatEventValue(event.subgoal) || segment.subgoal || '',
    subgoalTypes: event.subgoalType ? [formatEventValue(event.subgoalType)] : sortedArray(segment.subgoalTypes),
    planStartStep: numberOrNull(event.planStart ?? event.planStartStep) ?? segment.start,
    planEndStep: numberOrNull(event.planEnd ?? event.planEndStep) ?? item.step,
    planDelta: numberOrNull(event.planDelta),
    plannedPathSteps: numberOrNull(event.plannedPathSteps),
    commitStatus: status ? status.toLowerCase() : transactionCommitStatus(segment),
    rollbackReason: String(status).toUpperCase().includes('ROLL')
      ? formatEventValue(event.reason || event.rollbackReason)
      : '',
    reason: formatEventValue(event.reason),
    supportKind: formatEventValue(event.supportKind),
    supportPlanStart: numberOrNull(event.supportPlanStart),
    supportPlanEnd: numberOrNull(event.supportPlanEnd),
    satisfiedBoxGoalsAfter: numberOrNull(event.satisfiedBoxGoalsAfter),
    futureSubgoal: formatEventValue(event.futureSubgoal),
    rawEvent: compactMetaEvent(event),
  };
}

function timelineSupportRollup(segment) {
  const steps = (segment.children || [])
    .filter(item => String(item.event?.kind || '').toLowerCase() === 'support-blocker-step');
  if (steps.length === 0) return null;
  const supportKinds = countValues(steps.map(item => formatEventValue(item.event?.supportKind)));
  const executors = new Set();
  const owners = new Set();
  let movedBoxSteps = 0;
  for (const item of steps) {
    const event = item.event || {};
    const executor = formatEventValue(event.executorAgent || event.helperAgent || event.executorAgentId || event.agentId);
    const owner = formatEventValue(event.ownerAgent || event.ownerAgentId);
    if (executor) executors.add(executor);
    if (owner) owners.add(owner);
    if (event.boxFrom || event.boxTo || event.movedBox) movedBoxSteps++;
  }
  return {
    stepCount: steps.length,
    firstStep: steps[0].step,
    lastStep: steps[steps.length - 1].step,
    supportKinds: countMapEntries(supportKinds),
    ownerAgents: sortedArray(owners),
    executorAgents: sortedArray(executors),
    ownerExecutorMismatch: steps.some(item => String(item.event?.ownerExecutorMismatch) === 'true'),
    movedBoxSteps,
    samples: steps.slice(0, 3).map(item => ({
      step: item.step,
      executor: formatEventValue(item.event?.executorAgent || item.event?.helperAgent || item.event?.agentId),
      action: formatEventValue(item.event?.actualAction || item.event?.supportAction || item.event?.action),
      blocker: formatEventValue(item.event?.blocker),
      target: formatEventValue(item.event?.target),
    })),
  };
}

function timelineBspFailureSummary(segment) {
  const failures = (segment.children || [])
    .filter(item => {
      const kind = String(item.event?.kind || '').toLowerCase();
      return kind === 'task-relief-bsp-failed' || kind === 'bsp-exhausted';
    });
  if (failures.length === 0) return null;
  return {
    count: failures.length,
    reasons: countMapEntries(countValues(failures.map(item =>
      formatEventValue(item.event?.bspReason || item.event?.reason || item.event?.verdict)
    ))),
    modes: countMapEntries(countValues(failures.map(item => formatEventValue(item.event?.mode || item.event?.kind)))),
    executorAgents: sortedArray(new Set(failures
      .map(item => formatEventValue(item.event?.executorAgent || item.event?.helperAgent || item.event?.agentId))
      .filter(Boolean))),
    samples: failures.slice(0, 5).map(item => ({
      step: item.step,
      kind: formatEventValue(item.event?.kind),
      reason: formatEventValue(item.event?.bspReason || item.event?.reason),
      executor: formatEventValue(item.event?.executorAgent || item.event?.helperAgent || item.event?.agentId),
      blocker: formatEventValue(item.event?.blocker),
      target: formatEventValue(item.event?.parking || item.event?.releasedTo || item.event?.goal),
    })),
  };
}

function countMapEntries(map, limit = 8) {
  return [...(map || new Map()).entries()]
    .filter(([key]) => key)
    .sort((a, b) => b[1] - a[1] || String(a[0]).localeCompare(String(b[0])))
    .slice(0, limit)
    .map(([key, count]) => ({ key, count }));
}

function timelineMetaCandidateSummary(segment) {
  const children = Array.isArray(segment.children) ? segment.children : [];
  return children
    .filter(item => String(item.event?.kind || '').toLowerCase().includes('candidate-summary'))
    .map(item => {
      const event = item.event || {};
      return {
        step: item.step,
        verdict: formatEventValue(event.verdict),
        reason: formatEventValue(event.reason),
        dominantReason: formatEventValue(event.dominantReason),
        boxType: formatEventValue(event.boxType),
        goal: formatEventValue(event.goal),
        subgoal: formatEventValue(event.subgoal),
        subgoalType: formatEventValue(event.subgoalType),
        agentId: eventAgentId(event),
        ownerAgentId: numberOrNull(event.ownerAgentId),
        ownerAgent: formatEventValue(event.ownerAgent),
        executorAgentId: numberOrNull(event.executorAgentId),
        executorAgent: formatEventValue(event.executorAgent),
        agentPosition: formatEventValue(event.agentPosition),
        selectedBox: formatEventValue(event.selectedBox),
        hungarianAssignedBox: formatEventValue(event.hungarianAssignedBox),
        hungarianStatus: formatEventValue(event.hungarianStatus),
        attempt: numberOrNull(event.attempt),
        rawBoxesOfType: numberOrNull(event.rawBoxesOfType),
        completedBoxGoals: numberOrNull(event.completedBoxGoals),
        suspendedTransitGoals: numberOrNull(event.suspendedTransitGoals),
        frozenGoals: numberOrNull(event.frozenGoals),
        usableBoxCandidates: numberOrNull(event.usableBoxCandidates),
        operationSideFallbackCandidates: numberOrNull(event.operationSideFallbackCandidates),
        allocationFailedCandidates: numberOrNull(event.allocationFailedCandidates),
        usedHungarian: booleanOrString(event.usedHungarian),
        fixedReliefBox: booleanOrString(event.fixedReliefBox),
        candidateRejectCounts: formatEventValue(event.candidateRejectCounts),
        candidateSamples: formatEventValue(event.candidateSamples),
        rawEvent: compactMetaEvent(event),
      };
    });
}

function subgoalDecisionContext() {
  if (subgoalDecisionContextCache) return subgoalDecisionContextCache;
  const selections = [];
  const orderings = [];
  if (replay?.frames) {
    for (let i = 0; i < replay.frames.length; i++) {
      for (const event of stepEvents(replay.frames[i], i)) {
        const kind = String(event?.kind || event?.type || event?.category || '').toLowerCase();
        if (kind.includes('subgoal-selection-summary')) {
          selections.push(compactSubgoalSelection({ step: i, event }));
        } else if (kind.includes('subgoal-ordering-snapshot')) {
          orderings.push(compactSubgoalOrdering({ step: i, event }));
        }
      }
    }
  }
  subgoalDecisionContextCache = {
    selections: selections.sort((a, b) => a.step - b.step || String(a.selectionId).localeCompare(String(b.selectionId))),
    orderings: orderings.sort((a, b) => a.step - b.step || String(a.orderSnapshotId).localeCompare(String(b.orderSnapshotId))),
  };
  return subgoalDecisionContextCache;
}

function timelineMetaSubgoalSelections(segment) {
  const children = Array.isArray(segment?.children) ? segment.children : [];
  return children
    .filter(item => String(item.event?.kind || '').toLowerCase().includes('subgoal-selection-summary'))
    .map(compactSubgoalSelection);
}

function timelineMetaSubgoalOrderings(segment) {
  const children = Array.isArray(segment?.children) ? segment.children : [];
  return children
    .filter(item => String(item.event?.kind || '').toLowerCase().includes('subgoal-ordering-snapshot'))
    .map(compactSubgoalOrdering);
}

function compactSubgoalSelection(item) {
  const event = item?.event || {};
  return {
    step: item?.step,
    selectionId: formatEventValue(event.selectionId),
    decisionLayer: formatEventValue(event.decisionLayer),
    subgoal: formatEventValue(event.subgoal),
    subgoalType: formatEventValue(event.subgoalType),
    goal: event.goal ?? null,
    boxType: formatEventValue(event.boxType),
    agentId: eventAgentId(event),
    ownerAgentId: numberOrNull(event.ownerAgentId),
    ownerAgent: formatEventValue(event.ownerAgent),
    ownerSubgoal: formatEventValue(event.ownerSubgoal),
    verdict: formatEventValue(event.verdict),
    reason: formatEventValue(event.reason),
    selectionBasis: formatEventValue(event.selectionBasis),
    orderingBasis: formatEventValue(event.orderingBasis),
    filterBasis: formatEventValue(event.filterBasis),
    topologyBasis: formatEventValue(event.topologyBasis),
    heuristicBasis: formatEventValue(event.heuristicBasis),
    candidateCount: numberOrNull(event.candidateCount),
    eligibleCount: numberOrNull(event.eligibleCount),
    selectedRank: numberOrNull(event.selectedRank),
    selectedEstimatedDifficulty: formatEventValue(event.selectedEstimatedDifficulty),
    selectedUnmetDependencies: numberOrNull(event.selectedUnmetDependencies),
    selectedCompletionCount: numberOrNull(event.selectedCompletionCount),
    selectionStatusCounts: formatEventValue(event.selectionStatusCounts),
    candidateStatusSample: formatEventValue(event.candidateStatusSample),
    rawEvent: compactMetaEvent(event),
  };
}

function compactSubgoalOrdering(item) {
  const event = item?.event || {};
  return {
    step: item?.step,
    orderSnapshotId: formatEventValue(event.orderSnapshotId),
    decisionLayer: formatEventValue(event.decisionLayer),
    phase: formatEventValue(event.phase),
    verdict: formatEventValue(event.verdict),
    reason: formatEventValue(event.reason),
    orderingMode: formatEventValue(event.orderingMode),
    orderingBasis: formatEventValue(event.orderingBasis),
    topologyBasis: formatEventValue(event.topologyBasis),
    heuristicBasis: formatEventValue(event.heuristicBasis),
    rawSubgoalCount: numberOrNull(event.rawSubgoalCount),
    orderedSubgoalCount: numberOrNull(event.orderedSubgoalCount),
    precomputedGoalOrderSize: numberOrNull(event.precomputedGoalOrderSize),
    goalDependencyEdges: numberOrNull(event.goalDependencyEdges),
    completedBoxGoals: numberOrNull(event.completedBoxGoals),
    suspendedBoxGoals: numberOrNull(event.suspendedBoxGoals),
    suspendedTransitGoals: numberOrNull(event.suspendedTransitGoals),
    displacedGoals: numberOrNull(event.displacedGoals),
    deprioritizedGoalHints: numberOrNull(event.deprioritizedGoalHints),
    prioritizedGoalHints: numberOrNull(event.prioritizedGoalHints),
    escapeSubgoalHints: numberOrNull(event.escapeSubgoalHints),
    transitProfiles: numberOrNull(event.transitProfiles),
    suspendedRemoved: numberOrNull(event.suspendedRemoved),
    physicalBlockingPromotions: numberOrNull(event.physicalBlockingPromotions),
    deprioritizedMovedToTail: numberOrNull(event.deprioritizedMovedToTail),
    prioritizedMovedToFront: numberOrNull(event.prioritizedMovedToFront),
    escapePrepended: numberOrNull(event.escapePrepended),
    transitDeferredToTail: numberOrNull(event.transitDeferredToTail),
    dependencyEdges: formatEventValue(event.dependencyEdges),
    orderingMetricSample: formatEventValue(event.orderingMetricSample),
    adjacentPairRationaleSample: formatEventValue(event.adjacentPairRationaleSample),
    transitProfileDetails: formatEventValue(event.transitProfileDetails),
    orderStageSample: formatEventValue(event.orderStageSample),
    physicalBlockingPromotionDetails: formatEventValue(event.physicalBlockingPromotionDetails),
    orderSample: formatEventValue(event.orderSample),
    rawEvent: compactMetaEvent(event),
  };
}

function booleanOrString(value) {
  if (value === true || value === false) return value;
  const raw = String(value ?? '').toLowerCase();
  if (raw === 'true') return true;
  if (raw === 'false') return false;
  return formatEventValue(value);
}

function compactMetaEvent(event) {
  const out = {};
  if (!event || typeof event !== 'object') return out;
  for (const [key, value] of Object.entries(event)) {
    if (value === undefined || value === null || value === '') continue;
    out[key] = value;
  }
  return out;
}

function numberOrNull(value) {
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function normalizeCountEntries(value) {
  if (!value) return [];
  if (Array.isArray(value)) {
    return value
      .map(item => {
        if (Array.isArray(item)) return { key: formatEventValue(item[0]), count: Number(item[1]) || 0 };
        if (item && typeof item === 'object') {
          return { key: formatEventValue(item.key ?? item.name ?? item.label), count: Number(item.count) || 0 };
        }
        return null;
      })
      .filter(item => item && item.key);
  }
  const text = String(value).trim();
  if (!text) return [];
  return text
    .replace(/^\[|\]$/g, '')
    .split(/\s*,\s*/)
    .map(part => {
      const match = part.match(/^(.+?)=(\d+)$/);
      return match ? { key: match[1], count: Number(match[2]) || 0 } : null;
    })
    .filter(Boolean);
}

function formatCountEntries(value) {
  const entries = normalizeCountEntries(value);
  return entries.length ? entries.map(item => `${item.key}=${item.count}`).join(', ') : formatEventValue(value);
}

function inferParentSegmentIndex(segment, index, allSegments) {
  let bestIndex = null;
  let bestSpan = Infinity;
  for (let i = 0; i < allSegments.length; i++) {
    if (i === index) continue;
    const candidate = allSegments[i];
    if (candidate.start > segment.start || candidate.end < segment.end) continue;
    const span = candidate.end - candidate.start;
    if (span <= segment.end - segment.start) continue;
    if (span < bestSpan) {
      bestSpan = span;
      bestIndex = i;
    }
  }
  return bestIndex;
}

function transactionCommitStatus(segment) {
  const verdicts = [...(segment.verdicts || [])].map(value => String(value).toUpperCase());
  const kinds = [...(segment.eventKinds || [])].map(value => String(value).toLowerCase());
  if (kinds.some(kind => kind.includes('rollback')) || verdicts.some(v => v.includes('REJECT') || v.includes('FAILED') || v.includes('ROLL'))) {
    return 'failed-or-rolled-back';
  }
  if (verdicts.some(v => v.includes('ACCEPTED') || v.includes('RESOLVED') || v.includes('OPENED') || v.includes('COMMITTED'))) {
    return 'committed';
  }
  if (segment.statusClass === 'danger') return 'failed-or-rolled-back';
  if (segment.statusClass === 'warning') return 'needs-review';
  return 'observed';
}

function transactionRollbackReason(segment) {
  if (transactionCommitStatus(segment) !== 'failed-or-rolled-back') return '';
  return joinSet(segment.reasons || new Set(), 3) || joinSet(segment.verdicts || new Set(), 3) || segment.statusLabel || '';
}

function touchedBoxTypes(segment, stateDiff) {
  const out = new Set();
  for (const item of [...(stateDiff.movedBoxes || []), ...(stateDiff.addedBoxes || []), ...(stateDiff.removedBoxes || [])]) {
    if (item.type) out.add(item.type);
  }
  for (const item of segment.children || []) {
    const boxType = item?.event?.boxType;
    if (boxType) out.add(formatEventValue(boxType));
  }
  return out;
}

function frameAgentMap(frame) {
  const map = new Map();
  for (const agent of frame?.agents || []) {
    if (agent && agent.id !== undefined) map.set(String(agent.id), `${agent.r},${agent.c}`);
  }
  return map;
}

function frameBoxMap(frame) {
  const map = new Map();
  for (const box of frame?.boxes || []) {
    if (!box) continue;
    map.set(`${box.r},${box.c}`, formatEventValue(box.type));
  }
  return map;
}

function pairMovedBoxes(removed, added) {
  const remainingAdded = [...added];
  const moved = [];
  const stillRemoved = [];
  for (const source of removed) {
    let bestIndex = -1;
    let bestDistance = Infinity;
    for (let i = 0; i < remainingAdded.length; i++) {
      const target = remainingAdded[i];
      if (target.type !== source.type) continue;
      const distance = manhattanTextPos(source.pos, target.pos);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = i;
      }
    }
    if (bestIndex >= 0) {
      const target = remainingAdded.splice(bestIndex, 1)[0];
      moved.push({ type: source.type, from: source.pos, to: target.pos, distance: bestDistance });
    } else {
      stillRemoved.push(source);
    }
  }
  return { moved, removed: stillRemoved, added: remainingAdded };
}

function manhattanTextPos(a, b) {
  const [ar, ac] = String(a || '').split(',').map(Number);
  const [br, bc] = String(b || '').split(',').map(Number);
  if (![ar, ac, br, bc].every(Number.isFinite)) return Infinity;
  return Math.abs(ar - br) + Math.abs(ac - bc);
}

function satisfiedBoxGoalSet(frame) {
  const boxes = frameBoxMap(frame);
  const set = new Set();
  for (const goal of replay.level?.boxGoals || []) {
    if (!goal) continue;
    const key = `${goal.r},${goal.c}`;
    const want = formatEventValue(goal.type);
    if (boxes.get(key) === want) set.add(`${want}@${key}`);
  }
  return set;
}

function timelineMetaEvent(item) {
  const event = item.event;
  const normalized = normalizeEvent(event);
  const out = {
    step: item.step,
    kind: normalized.kind,
    statusClass: normalized.kindClass,
    title: normalized.title,
  };
  if (normalized.message) out.message = normalized.message;
  const agentId = eventAgentId(event);
  if (Number.isInteger(agentId)) out.agentId = agentId;
  for (const key of ['phase', 'subgoal', 'subgoalType', 'goal', 'boxType', 'reason', 'verdict']) {
    if (event && typeof event === 'object' && event[key] !== undefined) {
      out[key] = formatEventValue(event[key]);
    }
  }
  if (normalized.fields.length > 0) {
    out.fields = normalized.fields.map(([key, value]) => ({ key, value: formatEventValue(value) }));
  }
  return out;
}

function segmentRangeText(segment) {
  return segment.start === segment.end ? `step ${segment.start}` : `${segment.start}-${segment.end}`;
}

function compactEventKind(kind) {
  if (kind.includes('planner-transaction')) return 'transaction';
  if (kind.includes('candidate-summary')) return 'candidate-summary';
  if (kind.includes('reject')) return 'rejected';
  if (kind.includes('fail')) return 'failed';
  if (kind.includes('block')) return 'blocker';
  if (kind.includes('conflict')) return 'conflict';
  if (kind.includes('intent')) return 'intent';
  return kind || 'event';
}

function orderedUnique(values) {
  return [...new Set(values.filter(value => value !== undefined && value !== null && value !== ''))];
}

function addValues(target, values) {
  for (const value of values) target.add(value);
}

function countValues(values) {
  const counts = new Map();
  for (const value of values) {
    if (value === undefined || value === null || value === '') continue;
    counts.set(value, (counts.get(value) || 0) + 1);
  }
  return counts;
}

function mergeCounts(target, source) {
  if (!(source instanceof Map)) return;
  for (const [value, count] of source.entries()) {
    target.set(value, (target.get(value) || 0) + count);
  }
}

function joinSet(values, limit = 4) {
  const list = [...values].filter(value => value !== undefined && value !== null && value !== '');
  if (list.length <= limit) return list.join(', ');
  return `${list.slice(0, limit).join(', ')} +${list.length - limit}`;
}

function joinAgents(values, limit = 6) {
  const list = [...values].sort((a, b) => a - b).map(id => `agent${id}`);
  if (list.length <= limit) return list.join(', ');
  return `${list.slice(0, limit).join(', ')} +${list.length - limit}`;
}

function sortedArray(values) {
  return [...(values || [])].sort((a, b) => {
    if (typeof a === 'number' && typeof b === 'number') return a - b;
    return String(a).localeCompare(String(b));
  });
}

function agentTimelineSegments(agentId) {
  const segments = [];
  let current = null;
  for (let i = 0; i < replay.frames.length; i++) {
    const descriptor = timelineDescriptor(agentId, i);
    if (current && current.key === descriptor.key) {
      current.end = i;
      current.rejected = current.rejected || descriptor.rejected;
      continue;
    }
    current = { ...descriptor, start: i, end: i };
    segments.push(current);
  }
  return segments;
}

function timelineDescriptor(agentId, stepIndex) {
  const frame = replay.frames[stepIndex];
  const events = stepEvents(frame, stepIndex).filter(event => eventAgentId(event) === agentId);
  const intent = events.find(isAgentIntentEvent) || events.find(isAgentExecutionContextEvent);
  const actionEvent = events.find(isAgentActionEvent);
  const actionText = frame.actions && frame.actions[agentId] ? frame.actions[agentId] : actionEvent?.action || 'NoOp';
  const accepted = frame.accepted && agentId < frame.accepted.length ? frame.accepted[agentId] !== false : actionEvent?.accepted !== false;
  if (intent) {
    const phase = formatEventValue(intent.phase) || 'intent';
    const subgoalType = formatEventValue(intent.subgoalType);
    const subgoal = formatEventValue(intent.subgoal || intent.goal || subgoalType) || actionText;
    const detail = formatEventValue(intent.reason || intent.message || intent.action || actionText);
    return {
      key: `intent|${phase}|${subgoalType}|${subgoal}|${accepted}`,
      label: `${phase}: ${subgoal}`,
      detail,
      rejected: !accepted,
    };
  }
  const family = actionFamily(actionText);
  return {
    key: `action|${family}|${accepted}`,
    label: family,
    detail: accepted ? actionText : `${actionText} rejected by server`,
    rejected: !accepted,
  };
}

function actionFamily(actionText) {
  const match = String(actionText || 'NoOp').match(/^([A-Za-z]+)/);
  return match ? match[1] : String(actionText || 'NoOp');
}

function segmentHasMarker(segment) {
  for (const marker of markers.values()) {
    if (markerOverlapsRange(marker, segment.start, segment.end)) return true;
  }
  return false;
}

function toggleCurrentMarker() {
  if (!replay) return;
  const id = markerIdForStep(step);
  if (markers.has(id)) {
    markers.delete(id);
  } else {
    const label = markerLabelForStep();
    const note = window.prompt('Bookmark note:', label) || '';
    markers.set(id, {
      id,
      type: 'step',
      step,
      label,
      note,
      createdAt: new Date().toISOString(),
    });
  }
  saveReplayMarkers();
  renderMarkers();
  renderAgentTimeline();
}

function toggleCurrentSegmentMarker() {
  if (!replay) return;
  const bounds = segmentBounds();
  const id = markerIdForSegment(bounds.start, bounds.end);
  if (markers.has(id)) {
    markers.delete(id);
  } else {
    const label = `Segment ${bounds.start}-${bounds.end}`;
    const note = window.prompt('Range bookmark note:', label) || '';
    markers.set(id, {
      id,
      type: 'segment',
      start: bounds.start,
      end: bounds.end,
      label,
      note,
      createdAt: new Date().toISOString(),
    });
    segmentState.enabled = true;
  }
  saveReplayMarkers();
  renderSegmentPanel();
  renderMarkers();
  renderAgentTimeline();
}

function markerLabelForStep() {
  const agentId = focusedAgentId ?? timelineAgentId;
  if (agentId != null) {
    const descriptor = timelineDescriptor(agentId, step);
    return `agent${agentId}: ${descriptor.label}`;
  }
  const frame = replay.frames[step];
  return searchResultDetail(frame) || `step ${step}`;
}

function renderMarkers() {
  if (!els.markerList || !els.markerToggleBtn) return;
  els.markerToggleBtn.textContent = markers.has(markerIdForStep(step)) ? 'Remove Bookmark' : 'Bookmark Step';
  els.markerList.innerHTML = '';
  if (!replay) {
    els.markerList.textContent = 'Load a replay to add bookmarks.';
    return;
  }
  if (markers.size === 0) {
    els.markerList.textContent = 'No bookmarks yet.';
    return;
  }
  for (const marker of [...markers.values()].sort((a, b) => markerStart(a) - markerStart(b))) {
    const row = document.createElement('div');
    row.className = markerOverlapsRange(marker, step, step) ? 'bookmarkRow active' : 'bookmarkRow';
    const button = document.createElement('button');
    button.type = 'button';
    button.dataset.step = String(markerStart(marker));
    button.className = 'markerItem';
    button.innerHTML = `<span>${escapeHtml(markerRangeText(marker))}</span><strong>${escapeHtml(marker.label)}</strong>${marker.note ? `<small>${escapeHtml(marker.note)}</small>` : ''}`;
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.className = 'markerDelete';
    remove.dataset.markerDelete = marker.id;
    remove.title = 'Delete bookmark';
    remove.setAttribute('aria-label', `Delete ${markerRangeText(marker)} bookmark`);
    remove.textContent = 'x';
    row.appendChild(button);
    row.appendChild(remove);
    els.markerList.appendChild(row);
  }
}

function deleteMarker(id) {
  if (!id || !markers.has(id)) return;
  markers.delete(id);
  saveReplayMarkers();
  renderSegmentPanel();
  renderMarkers();
  renderAgentTimeline();
}

function markerIdForStep(stepIndex) {
  return `step:${stepIndex}`;
}

function markerIdForSegment(start, end) {
  return `segment:${start}-${end}`;
}

function markerStart(marker) {
  return marker.type === 'segment' ? marker.start : marker.step;
}

function markerEnd(marker) {
  return marker.type === 'segment' ? marker.end : marker.step;
}

function markerRangeText(marker) {
  return marker.type === 'segment' ? `${marker.start}-${marker.end}` : `step ${marker.step}`;
}

function markerOverlapsRange(marker, start, end) {
  return markerStart(marker) <= end && markerEnd(marker) >= start;
}

function markersInRange(start, end) {
  return [...markers.values()]
    .filter(marker => markerOverlapsRange(marker, start, end))
    .sort((a, b) => markerStart(a) - markerStart(b));
}

function markerStorageKey() {
  if (!replay) return '';
  return `${MARKERS_STORAGE_PREFIX}${replay.level.name || 'level'}:${replay.frames.length}:${replay.generatedAt || ''}`;
}

function loadReplayMarkers() {
  markers = new Map();
  if (!replay || !localStorageAvailable()) return;
  try {
    const raw = localStorage.getItem(markerStorageKey());
    const parsed = raw ? JSON.parse(raw) : [];
    for (const item of parsed) {
      if (item.type === 'segment' && Number.isInteger(item.start) && Number.isInteger(item.end)) {
        const id = item.id || markerIdForSegment(item.start, item.end);
        markers.set(id, { ...item, id, type: 'segment' });
      } else if (Number.isInteger(item.step)) {
        const id = item.id || markerIdForStep(item.step);
        markers.set(id, { ...item, id, type: 'step' });
      }
    }
  } catch (error) {
    console.warn('Unable to load replay markers:', error);
  }
}

function saveReplayMarkers() {
  if (!replay || !localStorageAvailable()) return;
  localStorage.setItem(markerStorageKey(), JSON.stringify([...markers.values()]));
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
  const intent = events.find(isAgentIntentEvent) || events.find(isAgentExecutionContextEvent);
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
    appendAgentMonitorField(fields, 'Owner', intent.ownerAgent || intent.ownerAgentId);
    appendAgentMonitorField(fields, 'Executor', intent.executorAgent || intent.executorAgentId || intent.helperAgent);
    appendAgentMonitorField(fields, 'Intent', intent.reason || intent.message);
    appendAgentMonitorField(fields, 'Planned', intent.action || intent.supportAction);
    appendAgentMonitorField(fields, 'Server', intent.actualAction || intent.serverAction);
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
    els.stepEventsBtn.hidden = false;
    els.stepEventsBtn.textContent = 'Highlights 0';
    els.stepEventsBtn.classList.add('muted');
    setDisabled(els.stepEventsBtn, true);
    closeStepInspector();
    return;
  }
  const events = stepEvents(frame, step);
  const highlights = stepHighlights(frame, step);
  const totalEvents = replayEventCount();
  els.stepEventsBtn.hidden = false;
  setDisabled(els.stepEventsBtn, false);
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
  const raw = rawStepEvents(frame, stepIndex);
  return raw.concat(derivedAgentActionEvents(frame, stepIndex, raw));
}

function rawStepEvents(frame, stepIndex) {
  const events = [];
  if (!frame) return events;
  collectEvents(events, frame.events);
  collectEvents(events, frame.diagnostics);
  collectEvents(events, frame.debugEvents);
  collectEvents(events, frame.metadata?.events);
  collectEvents(events, indexedReplayEvents.get(stepIndex));
  return events;
}

function derivedAgentActionEvents(frame, stepIndex, rawEvents = []) {
  if (!replay || !frame || stepIndex <= 0 || !Array.isArray(frame.actions)) return [];
  const previous = replay.frames[stepIndex - 1];
  if (!previous) return [];
  const rawActionAgents = new Set(
    rawEvents
      .filter(isAgentActionEvent)
      .map(eventAgentId)
      .filter(Number.isInteger)
  );
  const events = [];
  for (let agentId = 0; agentId < frame.actions.length; agentId++) {
    if (rawActionAgents.has(agentId)) continue;
    const action = frame.actions[agentId] || 'NoOp';
    const accepted = frame.accepted && agentId < frame.accepted.length
      ? frame.accepted[agentId] !== false
      : true;
    if (accepted && action === 'NoOp') continue;
    events.push(deriveAgentActionEvent(previous, frame, agentId, action, accepted));
  }
  return events.filter(Boolean);
}

function deriveAgentActionEvent(previous, frame, agentId, action, accepted) {
  const from = frameAgentPosition(previous, agentId);
  const to = frameAgentPosition(frame, agentId) || from;
  const intent = actionIntentFromFrame(previous, action, from);
  return {
    kind: accepted ? 'agent-action' : 'rejected-action',
    severity: accepted ? 'info' : 'warning',
    title: `agent${agentId} ${action}`,
    message: accepted
      ? 'Derived from replay frame actions and adjacent states.'
      : 'Derived rejection from replay frame accepted flags.',
    agentId,
    action,
    accepted,
    from,
    to,
    attemptedTo: intent.agentTo,
    boxType: intent.boxType,
    boxFrom: intent.boxFrom,
    boxTo: intent.boxTo,
    derived: true,
    source: 'derived-frame',
  };
}

function actionIntentFromFrame(frame, actionText, from) {
  const parsed = parseActionText(actionText);
  if (!parsed || !from) return {};
  if (parsed.type === 'Move') {
    return { agentTo: movePos(from, parsed.agentDir) };
  }
  if (parsed.type === 'Push') {
    const boxFrom = movePos(from, parsed.agentDir);
    const boxTo = movePos(boxFrom, parsed.boxDir);
    return {
      agentTo: boxFrom,
      boxType: frameBoxTypeAt(frame, boxFrom),
      boxFrom,
      boxTo,
    };
  }
  if (parsed.type === 'Pull') {
    const agentTo = movePos(from, parsed.agentDir);
    const boxFrom = movePos(from, oppositeDir(parsed.boxDir));
    return {
      agentTo,
      boxType: frameBoxTypeAt(frame, boxFrom),
      boxFrom,
      boxTo: from,
    };
  }
  return { agentTo: from };
}

function parseActionText(actionText) {
  const text = String(actionText || 'NoOp');
  if (text === 'NoOp') return { type: 'NoOp' };
  const match = text.match(/^([A-Za-z]+)\(([^,\)]?)(?:,([^,\)]?))?\)$/);
  if (!match) return null;
  return {
    type: match[1],
    agentDir: match[2] || '',
    boxDir: match[3] || '',
  };
}

function frameAgentPosition(frame, agentId) {
  const agent = (frame?.agents || []).find(item => Number(item.id) === agentId);
  return agent ? { r: Number(agent.r), c: Number(agent.c) } : null;
}

function frameBoxTypeAt(frame, pos) {
  if (!pos) return null;
  const box = (frame?.boxes || []).find(item => Number(item.r) === pos.r && Number(item.c) === pos.c);
  return box ? formatEventValue(box.type) : null;
}

function movePos(pos, dir) {
  if (!pos) return null;
  const delta = directionDelta(dir);
  return delta ? { r: pos.r + delta.r, c: pos.c + delta.c } : pos;
}

function directionDelta(dir) {
  return {
    N: { r: -1, c: 0 },
    S: { r: 1, c: 0 },
    E: { r: 0, c: 1 },
    W: { r: 0, c: -1 },
  }[String(dir || '').toUpperCase()] || null;
}

function oppositeDir(dir) {
  return { N: 'S', S: 'N', E: 'W', W: 'E' }[String(dir || '').toUpperCase()] || '';
}

function stepHighlights(frame, stepIndex) {
  return stepEvents(frame, stepIndex).filter(isStepHighlight);
}

function isStepHighlight(event) {
  if (!event || typeof event !== 'object') return true;
  const kind = String(event.kind || event.type || '').toLowerCase();
  if (kind === 'agent-action' || kind === 'agent-intent') return false;
  if (kind === 'support-blocker-step' || kind === 'task-relief-bsp-failed') return false;
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
  const intent = events.find(isAgentIntentEvent) || events.find(isAgentExecutionContextEvent);
  const action = events.find(isAgentActionEvent);
  const activeTask = primaryTaskForStep(step, intent || action);
  const html = [
    `<div class="hoverTipTitle"><strong>agent${agentId}</strong><span>(${r}, ${c})</span></div>`
  ];
  if (!intent && !action) {
    html.push('<div class="hoverTipMuted">No intent event on this step</div>');
    if (activeTask) html.push(renderPrimaryTaskHoverEvent(activeTask));
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

function isAgentExecutionContextEvent(event) {
  if (!event || typeof event !== 'object') return false;
  const kind = String(event.kind || event.type || '').toLowerCase();
  return kind === 'support-blocker-step';
}

function isAgentActionEvent(event) {
  if (!event || typeof event !== 'object') return false;
  const kind = String(event.kind || event.type || '').toLowerCase();
  return kind === 'agent-action' || kind === 'rejected-action';
}

function renderAgentIntentHoverEvent(event) {
  const phase = event.phase || 'unknown';
  const subgoal = event.subgoal || event.title || 'unknown';
  const planned = event.action || event.supportAction;
  const actual = event.actualAction || event.serverAction;
  const heading = isAgentExecutionContextEvent(event) ? 'Planner context' : 'Planner intent';
  const progress = event.stepInSegment && event.segmentSteps
    ? `${event.stepInSegment}/${event.segmentSteps}`
    : '';
  const task = primaryTaskForStep(step, event);
  const rows = [
    '<div class="hoverTipEvent success">',
    `<div class="hoverTipSection"><strong>${heading}</strong><span>${escapeHtml(progress || phase)}</span></div>`,
    hoverField('Primary task', task?.displayLabel),
    hoverField('Transaction', task ? primaryTaskTransactionText(task) : ''),
    hoverField('Current stage', lifecycleMacroLabel(event)),
    hoverField('Phase', phase),
    hoverField('Subgoal', subgoal),
    hoverField('Subgoal type', event.subgoalType),
    hoverField('Owner', event.ownerAgent || event.ownerAgentId),
    hoverField('Executor', event.executorAgent || event.executorAgentId || event.helperAgent),
    hoverField('Goal', event.goal),
    hoverField('Box type', event.boxType),
    hoverField('Progress', progress),
    hoverField('Planned action', planned),
    hoverField('Server action', actual),
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
  const task = primaryTaskForStep(step, event);

  const rows = [
    `<div class="hoverTipEvent ${eventKindClass(accepted)}">`,
    `<div class="hoverTipSection"><strong>Server action</strong><span>${escapeHtml(accepted)}</span></div>`,
    hoverField('Primary task', task?.displayLabel),
    hoverField('Transaction', task ? primaryTaskTransactionText(task) : ''),
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

function renderPrimaryTaskHoverEvent(task) {
  const rows = [
    '<div class="hoverTipEvent note">',
    '<div class="hoverTipSection"><strong>Primary task</strong><span>context</span></div>',
    hoverField('Primary task', task?.displayLabel),
    hoverField('Transaction', primaryTaskTransactionText(task)),
    hoverField('Owner', task?.ownerAgent),
    hoverField('Goal', task?.goal),
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
  indexedDerivedActionEventTotal = 0;
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
  for (let i = 0; i < replay.frames.length; i++) {
    const raw = rawStepEvents(replay.frames[i], i);
    indexedDerivedActionEventTotal += derivedAgentActionEvents(replay.frames[i], i, raw).length;
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

function derivedActionEventCount() {
  return indexedDerivedActionEventTotal;
}

function replayEventKindCount(kind) {
  if (!replay) return 0;
  const needle = String(kind || '').toLowerCase();
  let count = 0;
  for (let i = 0; i < replay.frames.length; i++) {
    for (const event of stepEvents(replay.frames[i], i)) {
      const current = String(event?.kind || event?.type || event?.category || '').toLowerCase();
      if (current === needle) count++;
    }
  }
  return count;
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
    'ownerAgentId', 'ownerAgent', 'executorAgentId', 'executorAgent',
    'helperAgent', 'ownerExecutorMismatch',
    'subgoal', 'subgoalType', 'verdict', 'action', 'actualAction',
    'stepInSegment', 'segmentSteps', 'from', 'to', 'activeSubgoal',
    'transactionId', 'commitStatus', 'planStart', 'planEnd',
    'planStartStep', 'planEndStep', 'planDelta', 'validationBasis',
    'plannedPathSteps', 'supportKind', 'supportPlanStart', 'supportPlanEnd',
    'satisfiedBoxGoalsAfter', 'futureSubgoal',
    'blocker', 'blockerType', 'targetBlocker', 'parking', 'releasedTo',
    'blocked', 'target', 'goal', 'cause', 'dominantReason',
    'selectedBox', 'selectionLayer', 'usedHungarian', 'hungarianAssignedBox',
    'hungarianStatus', 'rawBoxesOfType', 'completedBoxGoals',
    'suspendedTransitGoals', 'usableBoxCandidates',
    'operationSideFallbackCandidates', 'allocationFailedCandidates',
    'agentPosition', 'frozenGoals', 'candidateRejectCounts', 'candidateSamples',
    'selectionId', 'orderSnapshotId', 'decisionLayer', 'selectionBasis',
    'orderingMode', 'orderingBasis', 'filterBasis', 'topologyBasis',
    'heuristicBasis', 'candidateCount', 'eligibleCount', 'selectedRank',
    'selectedEstimatedDifficulty', 'selectedUnmetDependencies',
    'selectedCompletionCount', 'selectionStatusCounts', 'candidateStatusSample',
    'rawSubgoalCount', 'orderedSubgoalCount', 'precomputedGoalOrderSize',
    'goalDependencyEdges', 'suspendedBoxGoals', 'displacedGoals',
    'deprioritizedGoalHints', 'prioritizedGoalHints', 'escapeSubgoalHints',
    'transitProfiles', 'suspendedRemoved', 'physicalBlockingPromotions',
    'deprioritizedMovedToTail', 'prioritizedMovedToFront', 'escapePrepended',
    'transitDeferredToTail', 'dependencyEdges', 'orderingMetricSample',
    'adjacentPairRationaleSample', 'transitProfileDetails', 'orderStageSample',
    'physicalBlockingPromotionDetails', 'orderSample',
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
  if (raw.includes('fail') || raw.includes('error') || raw.includes('reject') || raw.includes('block') || raw.includes('roll')) return 'danger';
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

function stopPlay(options = {}) {
  if (!timer) return;
  clearInterval(timer);
  timer = null;
  els.playBtn.textContent = 'Play';
  els.playBtn.classList.remove('active');
  if (replay) renderSegmentPanel();
  renderMiniTimelinePanel({
    lockCurrent: Boolean(options.lockCurrent),
    alignCurrentEnd: Boolean(options.alignCurrentEnd),
  });
  if (replay && els.timelinePanel && !els.timelinePanel.hidden) renderAgentTimeline();
}

function startPlay() {
  if (!replay || timer) return;
  const bounds = activeSegmentBounds();
  if (bounds && (step < bounds.start || step > bounds.end)) {
    setStep(bounds.start);
  }
  if (els.timelinePanel && !els.timelinePanel.hidden && timelineMode === 'overview') {
    timelineAutoFollowDisabled = false;
  }
  els.playBtn.textContent = 'Pause';
  els.playBtn.classList.add('active');
  timer = setInterval(() => {
    const currentBounds = activeSegmentBounds();
    const finalStep = currentBounds ? currentBounds.end : replay.frames.length - 1;
    if (step >= finalStep) {
      if (currentBounds && els.segmentLoopInput.checked) {
        setStep(currentBounds.start);
      } else {
        stopPlay({ lockCurrent: true, alignCurrentEnd: true });
      }
      return;
    }
    setStep(step + 1);
  }, Math.max(20, Number(els.speedInput.value) || 120));
  renderMiniTimelinePanel();
  if (els.timelinePanel && !els.timelinePanel.hidden) renderAgentTimeline();
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
