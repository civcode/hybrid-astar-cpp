'use strict';
// Manual-only Node regression. Executes the REAL Planning Lab app.js against
// a minimal mock canvas/DOM (no browser, server or planning request required).
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const base = path.resolve(__dirname, '../apps/demo/web');
const source = fs.readFileSync(path.join(base, 'app.js'), 'utf8');
const MapGeometry = require(path.join(base, 'map_geometry.js'));
const MapEditor = require(path.join(base, 'map_editor.js'));

const polygons = [], strokes = [];
let points = [];
const ctx = {
  setTransform() {}, clearRect() {}, fillRect() {},
  beginPath() { points = []; },
  moveTo(x, y) { points = [[x, y]]; },
  lineTo(x, y) { points.push([x, y]); },
  closePath() {
    if (points.length === 4) polygons.push(points.map(p => p.slice()));
  },
  arc() {}, fill() {},
  stroke() { strokes.push({ lineWidth: this.lineWidth,
                            strokeStyle: this.strokeStyle }); },
  save() {}, restore() {}, setLineDash() {},
};
const canvas = {
  clientWidth: 900, clientHeight: 640, width: 0, height: 0,
  getContext() { return ctx; },
  addEventListener() {},
  getBoundingClientRect() { return {left: 0, top: 0}; },
};
const elements = new Map();
function element(id) {
  if (id === 'map') return canvas;
  if (!elements.has(id)) elements.set(id, {
    value: '', checked: false, disabled: false, textContent: '',
    className: '',
    addEventListener() {},
    classList: { add() {}, remove() {} },
  });
  return elements.get(id);
}
element('scenario').value = 'open';
element('model').value = 'ackermann';
element('position').value = '0';
element('inflationMeters').value = '0.0';
const sandbox = {
  document: { getElementById: element },
  window: { devicePixelRatio: 1 },
  ResizeObserver: class { observe() {} },
  MapGeometry, MapEditor,
  URLSearchParams, AbortController,
  fetch: async () => { throw new Error('mock backend deliberately offline'); },
  requestAnimationFrame() {}, setTimeout() {},
  console, Map, Math, Number,
};
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: 'apps/demo/web/app.js' });

function measure() {
  const g = vm.runInContext('canvasGeometry()', sandbox);
  const two = polygons.slice(-2);
  assert.equal(two.length, 2, 'both start and goal footprints rendered without a path');
  const edge = (poly, i, j) => Math.hypot(poly[i][0]-poly[j][0],
                                          poly[i][1]-poly[j][1]) / g.scale;
  return {g, start: two[0], goal: two[1],
          length: edge(two[0], 0, 1), width: edge(two[0], 1, 2)};
}
const near = (value, expected) => assert.ok(Math.abs(value - expected) < 1e-9,
  `expected ${expected}, got ${value}`);

let a = measure();
near(a.length, 4.6);
near(a.width, 1.8);
near(Math.hypot(a.goal[0][0]-a.goal[1][0], a.goal[0][1]-a.goal[1][1])/a.g.scale, 4.6);

// Editable yaw rotates the footprint immediately, without planning success.
element('startyaw').value = '90';
vm.runInContext('draw()', sandbox);
a = measure();
near(a.length, 4.6);
assert.ok(Math.abs(a.start[0][0]-a.start[1][0]) < 1e-9,
  '90-degree pose rotates the physical footprint major axis');

// Model changes update BOTH previews even before planning returns.
element('model').value = 'differential';
vm.runInContext('draw()', sandbox);
a = measure();
near(a.length, 0.70);
near(a.width, 0.50);
near(Math.hypot(a.goal[0][0]-a.goal[1][0],
                a.goal[0][1]-a.goal[1][1])/a.g.scale, 0.70);

// Metric inflation is previewed before any path exists.
element('inflate').checked = true;
element('inflationMeters').value = '0.10';
strokes.length = 0;
vm.runInContext('draw()', sandbox);
a = measure();
assert.ok(strokes.filter(p => p.strokeStyle === 'rgba(255,188,102,.45)' &&
  Math.abs(p.lineWidth - .2*a.g.scale) < 1e-9).length >= 2,
  'start and goal each display the selected metric inflation stroke');

// A rotated map origin must not affect body dimensions.
vm.runInContext('baseMap={width:200,height:160,resolution:.25,origin:[4,-3,.47],mapRuns:[]}', sandbox);
vm.runInContext('draw()', sandbox);
a = measure();
near(a.length, 0.70);
near(a.width, 0.50);
console.log('PASS: start + goal footprint dimensions, editable orientation, model, inflation and rotated map origin');
