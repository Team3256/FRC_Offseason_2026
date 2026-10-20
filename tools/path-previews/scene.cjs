// Offline rasterization of Mayhem's actual SVG field and playback components.
const React = require('react');
const {renderToStaticMarkup} = require('react-dom/server');
const {Resvg} = require('@resvg/resvg-js');
const {FieldArt, PlaybackRobot, intakeFootprint, sampleAt} = require('./build/mayhem-art.cjs');
const readline = require('node:readline');
const fs = require('node:fs');
const path = require('node:path');

const WIDTH = 1920, HEIGHT = 720;
// Mayhem styles.css, default light theme. Resolve CSS vars before resvg.
const TOKENS = {
  accent: '#6b3fe6', 'blue-alliance': '#2d56d6', 'red-alliance': '#cf3434',
  carpet: '#ffffff', 'field-line': '#d9d6e4', structure: '#e7e3f2',
  'structure-stroke': '#a7a1bf', 'border-strong': '#d9d6e4', amber: '#c27806', faint: '#a19dae',
};
const xml = text => String(text).replace(/[<>&"']/g, c => ({'<':'&lt;', '>':'&gt;', '&':'&amp;', '"':'&quot;', "'":'&apos;'}[c]));
const component = (type, props) => renderToStaticMarkup(React.createElement(type, props));
const pts = samples => samples.map(p => `${p.x},${p.y}`).join(' ');
const speedColor = speed => `hsl(${Math.round(Math.max(0, Math.min(1, speed)) * 120)},85%,45%)`;
const svg = content => `<svg xmlns="http://www.w3.org/2000/svg" width="${WIDTH}" height="${HEIGHT}" viewBox="0 0 ${WIDTH} ${HEIGHT}">${content}</svg>`;
const text = (x,y,value,size=20,color='#1b1a22',extra='') => `<text x="${x}" y="${y}" font-family="Arial,DejaVu Sans,sans-serif" font-size="${size}" fill="${color}" ${extra}>${xml(value)}</text>`;
const raster = content => new Resvg(svg(content).replace(/var\(--([\w-]+)\)/g, (_, key) => {
  if (!TOKENS[key]) throw new Error(`Missing Mayhem theme token: ${key}`);
  return TOKENS[key];
}), {font: {loadSystemFonts: true}}).render().asPng();

function layout(data, index) {
  const field = data.project.field;
  const scale = Math.min(912 / field.length, 456 / field.width);
  return {scale, x: 24 + 960 * index + (912-field.length*scale)/2,
    y: 160 + (456-field.width*scale)/2, field};
}

function fieldGroup(data, index, content) {
  const {scale, x, y, field} = layout(data,index);
  // resvg omits SVG's non-scaling-stroke vector effect. Convert those screen
  // pixel widths to field meters explicitly so they match browser rendering.
  content = content.replace(/<[^>]+>/g, tag => {
    if (!tag.includes('vector-effect="non-scaling-stroke"')) return tag;
    return tag.replace(/stroke-width="([\d.]+)"/, (_, width) => `stroke-width="${Number(width)/scale}"`)
      .replace(/stroke-dasharray="([\d. ]+)"/, (_, values) =>
        `stroke-dasharray="${values.split(' ').map(value=>Number(value)/scale).join(' ')}"`)
      .replace(' vector-effect="non-scaling-stroke"', '');
  });
  return `<g transform="translate(${x} ${y+field.width*scale}) scale(${scale} ${-scale})">${content}</g>`;
}

function pathLines(data) {
  const samples = data.output.samples;
  let lines = '';
  for (let i=0; i+1<samples.length; i++) {
    for (let k=0;k<3;k++) {
      const dt = samples[i+1].t-samples[i].t;
      const a = sampleAt(data.output, samples[i].t + dt*k/3);
      const b = sampleAt(data.output, samples[i].t + dt*(k+1)/3);
      lines += `<line x1="${a.x}" y1="${a.y}" x2="${b.x}" y2="${b.y}" stroke="${speedColor(Math.hypot(a.vx,a.vy)/(data.maxSpeed || 4.5))}" stroke-width="4" stroke-linecap="round" vector-effect="non-scaling-stroke"/>`;
    }
  }
  return lines;
}

function obstacleArt(field, index) {
  let shapes = `<defs><pattern id="hatch-${index}" width="0.12" height="0.12" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="0.12" stroke="var(--accent)" stroke-width="0.03" opacity="0.35"/></pattern></defs>`;
  for (const obstacle of field.obstacles || []) {
    const points = obstacle.kind === 'circle' ? Array.from({length:64},(_,i)=>[
      obstacle.center[0]+Math.cos(i/64*Math.PI*2)*obstacle.radius,
      obstacle.center[1]+Math.sin(i/64*Math.PI*2)*obstacle.radius,
    ]) : obstacle.points;
    const rough = obstacle.terrain && !obstacle.enabled;
    shapes += `<polygon points="${points.map(p=>p.join(',')).join(' ')}" fill="${obstacle.enabled ? `url(#hatch-${index})` : rough ? 'var(--amber)' : 'none'}"
      ${rough ? 'fill-opacity="0.08"' : ''} stroke="${obstacle.enabled ? 'var(--accent)' : rough ? 'var(--amber)' : 'var(--faint)'}"
      stroke-opacity="${obstacle.enabled || rough ? 0.9 : 0.6}" stroke-width="${rough ? 1.6 : 1.2}"
      ${obstacle.enabled ? '' : `stroke-dasharray="${rough ? '6 4' : '4 4'}"`} vector-effect="non-scaling-stroke"/>`;
  }
  return shapes;
}

function makeScene(scene) {
  let background = `<rect width="1920" height="720" fill="#f5f4f9"/>
    <rect width="1920" height="74" fill="#fff"/><path d="M0 74H1920" stroke="#e9e7f0"/>
    ${fs.readFileSync(path.join(__dirname,'mayhem-icon.svg'),'utf8').replace('<svg ', '<svg x="24" y="19" width="36" height="36" ')}
    ${text(76,46,scene.name,27,'#1b1a22','font-weight="600"')}
    ${text(1894,44,`Mayhem · ${(scene.after || scene.before).project.field.name}`,18,'#6e6a7c','text-anchor="end"')}
    <path d="M960 96V656" stroke="#e9e7f0"/>
    <rect x="0" y="674" width="1920" height="46" fill="#fff"/>
    ${text(24,704,'Trajectory playback',16,'#6e6a7c')}`;
  [scene.before,scene.after].forEach((data,index) => {
    const left = 24 + 960*index;
    background += text(left,120,index ? 'After' : 'Before',22,'#1b1a22','font-weight="600"');
    if (!data) {
      background += text(left+280,380,'No playable trajectory',23,'#6e6a7c');
      return;
    }
    let field = component(FieldArt,{field:data.project.field});
    field += obstacleArt(data.project.field,index);
    if (index && scene.before)
      field += `<polyline points="${pts(scene.before.output.samples)}" fill="none" stroke="#a19dae" stroke-width="2" stroke-opacity="0.5" stroke-dasharray="5 5" vector-effect="non-scaling-stroke"/>`;
    field += pathLines(data);
    background += fieldGroup(data,index,field);
    background += text(left,649,`Duration ${data.times.at(-1).toFixed(2)} s`,18,'#6e6a7c');
  });
  const png = raster(background).toString('base64');
  return ({t,speed}) => {
    let overlay = `<image width="1920" height="720" href="data:image/png;base64,${png}"/>`;
    [scene.before,scene.after].forEach((data,index) => {
      if (!data) return;
      const time = Math.min(t,data.times.at(-1));
      const pose = sampleAt(data.output,time);
      let robot = '';
      if ((data.output.intake || []).some(span => time >= span.t-1e-6 && time <= span.endT+1e-6))
        robot += `<polygon points="${intakeFootprint(data.project.robot,pose.x,pose.y,pose.heading).map(p=>p.join(',')).join(' ')}" fill="var(--amber)" fill-opacity="0.55" stroke="var(--amber)" stroke-width="2" vector-effect="non-scaling-stroke"/>`;
      robot += component(PlaybackRobot,{robot:data.project.robot,x:pose.x,y:pose.y,h:pose.heading});
      overlay += fieldGroup(data,index,robot);
      overlay += text(936+960*index,649,
        t>=data.times.at(-1) ? 'Finished' : `${Math.hypot(pose.vx,pose.vy).toFixed(2)} m/s`,
        18,'#6e6a7c','text-anchor="end"');
    });
    const duration = Math.max(...[scene.before,scene.after].filter(Boolean).map(d=>d.times.at(-1)));
    const progress = Math.min(1,t/duration);
    overlay += `<rect x="250" y="695" width="1290" height="5" rx="2.5" fill="#e9e7f0"/>
      <rect x="250" y="695" width="${1290*progress}" height="5" rx="2.5" fill="#6b3fe6"/>
      <circle cx="${250+1290*progress}" cy="697.5" r="6" fill="#6b3fe6"/>`;
    overlay += text(1894,704,`${t.toFixed(2)} / ${duration.toFixed(2)} s   ·   ${speed}×`,16,'#6e6a7c','text-anchor="end"');
    return raster(overlay);
  };
}

if (require.main === module) {
  let render;
  readline.createInterface({input:process.stdin}).on('line',line=>{
    try {
      const message = JSON.parse(line);
      if (!render) {render = makeScene(message); process.stdout.write('ready\n');}
      else process.stdout.write(render(message).toString('base64')+'\n');
    } catch (error) {console.error(error); process.exit(1);}
  });
}
module.exports = {makeScene, pathLines};
