// FieldArt and PlaybackRobot copied from Mayhem FieldCanvas.tsx at
// 1455950915c621cfee3cf1b4c7fa1046818b4d55 (app 0.8.2).
// Keep these components in sync with Mayhem; do not replace field art with collision bounds.
import React, { memo } from "react";
type Vec2 = [number, number];
type Decoration = any;
type Field = any;
type RobotConfig = any;

const pts = (p: Vec2[]) => p.map(([x, y]) => `${x},${y}`).join(" ");

const DECO_STYLE: Record<string, { fill?: string; stroke?: string; op?: number }> = {
  blueZone: { fill: "var(--blue-alliance)", op: 0.045 },
  redZone: { fill: "var(--red-alliance)", op: 0.045 },
  tape: { stroke: "var(--field-line)" },
  blueTape: { stroke: "var(--blue-alliance)", op: 0.7 },
  redTape: { stroke: "var(--red-alliance)", op: 0.7 },
  fuel: { fill: "#e3b341", op: 0.22 },
  structure: { fill: "var(--structure)", stroke: "var(--structure-stroke)" },
  blue: { fill: "var(--blue-alliance)", stroke: "var(--blue-alliance)", op: 0.12 },
  red: { fill: "var(--red-alliance)", stroke: "var(--red-alliance)", op: 0.12 },
};

const FieldArt = memo(function FieldArt({ field, hideFuel }: { field: Field; hideFuel?: boolean }) {
  const d = (field.decorations ?? []).filter((x) => !(hideFuel && x.style === "fuel"));
  const draw = (dec: Decoration, i: number) => {
    const st = DECO_STYLE[dec.style] ?? {};
    if (dec.kind === "circle")
      return <circle key={i} cx={dec.center[0]} cy={dec.center[1]} r={dec.radius} fill={st.fill} opacity={st.op} />;
    if (dec.kind === "line")
      return <polyline key={i} points={pts(dec.points)} stroke={st.stroke ?? st.fill} strokeWidth={dec.width} opacity={st.op} fill="none" />;
    return (
      <polygon key={i} points={pts(dec.points)} fill={st.fill ?? "none"} fillOpacity={st.op ?? 1} stroke={st.stroke}
        strokeWidth={1} vectorEffect="non-scaling-stroke" strokeOpacity={st.stroke ? 0.9 : 0} />
    );
  };
  return (
    <g>
      <rect x={0} y={0} width={field.length} height={field.width} fill="var(--carpet)" />
      {d.filter((x) => x.style.endsWith("Zone")).map(draw)}
      {d.filter((x) => !x.style.endsWith("Zone")).map(draw)}
      <rect x={0} y={0} width={field.length} height={field.width} fill="none" stroke="var(--border-strong)" strokeWidth={2} vectorEffect="non-scaling-stroke" />
    </g>
  );
});

/** Playback robot: translucent rounded body, bold front bumper, and a chevron pointing forward. */
function PlaybackRobot({ robot, x, y, h }: { robot: RobotConfig; x: number; y: number; h: number }) {
  const { front: f, back: bk, left: l, right: r } = robot.bumper;
  const k = Math.min(f + bk, l + r);
  const rx = k * 0.14;
  const tip = f - k * 0.22, d = k * 0.13, w = k * 0.19;
  const line = { fill: "none", stroke: "var(--accent)", strokeLinecap: "round", strokeLinejoin: "round", vectorEffect: "non-scaling-stroke" } as const;
  return (
    <g transform={`translate(${x} ${y}) rotate(${(h * 180) / Math.PI})`}>
      <rect x={-bk} y={-r} width={f + bk} height={l + r} rx={rx} fill="var(--accent)" fillOpacity={0.22}
        stroke="var(--accent)" strokeWidth={2} vectorEffect="non-scaling-stroke" />
      <path d={`M ${f - rx} ${-r} A ${rx} ${rx} 0 0 1 ${f} ${-r + rx} L ${f} ${l - rx} A ${rx} ${rx} 0 0 1 ${f - rx} ${l}`} {...line} strokeWidth={5} />
      <polyline points={`${tip - d},${w} ${tip},0 ${tip - d},${-w}`} {...line} strokeWidth={2.5} />
    </g>
  );
}


export { FieldArt, PlaybackRobot };

// The same intake geometry and acceleration interpolation used by Mayhem.
type Intake = any;
type TrajectoryOutput = any;
type Sample = any;
export const DEFAULT_INTAKE: Intake = { side: "front", extension: 0.3, width: 0, offset: 0 };

/** Extended intake rectangle in robot frame (mirrors drivetrain.intake_rectangle); empty if it doesn't extend. */
export function intakeCorners(robot: RobotConfig): Vec2[] {
  const it = robot.intake ?? DEFAULT_INTAKE, b = robot.bumper;
  if (!(it.extension > 1e-6)) return [];
  if (it.side === "front" || it.side === "back") {
    const [lo, hi] = it.width > 0 ? [it.offset - it.width / 2, it.offset + it.width / 2] : [-b.right, b.left];
    const [x0, x1] = it.side === "front" ? [b.front, b.front + it.extension] : [-b.back, -b.back - it.extension];
    return [[x1, lo], [x1, hi], [x0, hi], [x0, lo]];
  }
  const [lo, hi] = it.width > 0 ? [it.offset - it.width / 2, it.offset + it.width / 2] : [-b.back, b.front];
  const [y0, y1] = it.side === "left" ? [b.left, b.left + it.extension] : [-b.right, -b.right - it.extension];
  return [[hi, y0], [hi, y1], [lo, y1], [lo, y0]];
}

export function intakeFootprint(robot: RobotConfig, x: number, y: number, heading: number): Vec2[] {
  const c = Math.cos(heading), s = Math.sin(heading);
  return intakeCorners(robot).map(([px, py]) => [x + c * px - s * py, y + s * px + c * py]);
}

/** State at time t using the optimizer's constant-acceleration model between samples. */
export function sampleAt(out: TrajectoryOutput, t: number): Sample {
  const s = out.samples;
  if (t <= s[0].t) return s[0];
  const last = s[s.length - 1];
  if (t >= last.t) return last;
  let lo = 0, hi = s.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (s[mid].t <= t) lo = mid; else hi = mid;
  }
  const a = s[lo], b = s[hi];
  const tau = t - a.t;
  const u = b.t > a.t ? tau / (b.t - a.t) : 0;
  return {
    t, x: a.x + a.vx * tau + 0.5 * a.ax * tau * tau, y: a.y + a.vy * tau + 0.5 * a.ay * tau * tau,
    heading: a.heading + a.omega * tau + 0.5 * a.alpha * tau * tau,
    vx: a.vx + a.ax * tau, vy: a.vy + a.ay * tau, omega: a.omega + a.alpha * tau,
    ax: a.ax, ay: a.ay, alpha: a.alpha,
    fx: a.fx.map((f, i) => f + (b.fx[i] - f) * u), fy: a.fy.map((f, i) => f + (b.fy[i] - f) * u),
  };
}
