const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {makeScene} = require('./scene.cjs');
const {sampleAt} = require('./build/mayhem-art.cjs');

test('renders actual project field decorations and animated playback to 1920×720 PNGs', () => {
  const directory = path.resolve(__dirname,'../../src/main/deploy/mayhem');
  const project = JSON.parse(fs.readFileSync(path.join(directory,'project.mayhem')));
  const output = JSON.parse(fs.readFileSync(path.join(directory,'DepotExit.mtraj'))).output;
  assert(project.field.decorations.some(d=>d.style==='fuel'));
  assert(project.field.decorations.some(d=>d.style==='blueZone'));
  const data = {project,output,times:output.samples.map(s=>s.t)};
  const frame = makeScene({name:'DepotExit',before:data,after:data});
  const start = frame({t:0,speed:1});
  const later = frame({t:2,speed:1});
  assert.equal(start.readUInt32BE(16),1920);
  assert.equal(start.readUInt32BE(20),720);
  assert(start.length>15000,'Field should contain rendered art, not a blank image');
  assert.notDeepEqual(start,later,'Robot and clock should advance during playback');
});

test('uses Mayhem acceleration sampling and preserves continuous headings', () => {
  const a = {t:0,x:0,y:0,heading:6,vx:1,vy:0,omega:2,ax:2,ay:0,alpha:1,fx:[0],fy:[0]};
  const b = {...a,t:1,x:2,heading:8.5,fx:[1],fy:[1]};
  const sample = sampleAt({samples:[a,b]},0.5);
  assert.equal(sample.x,0.75);
  assert.equal(sample.heading,7.125);
  assert.equal(sample.vx,2);
});
