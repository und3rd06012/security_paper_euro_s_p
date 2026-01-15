'use strict';

const fs = require('fs');
let src = fs.readFileSync(__dirname + '/test_attacks.js', 'utf8');
src = src.split('const arg = process.argv[2];')[0];
src += `
const orb = makeOrbit();
const stride = 5;
const NL = String.fromCharCode(10);

function dump(fname, cols) {
  const n = cols[0].length;
  const lines = [];
  for (let i = 0; i < n; i += stride) {
    const row = cols.map(c => (Number.isFinite(c[i]) ? c[i] : 0).toPrecision(6));
    lines.push(row.join(' '));
  }
  fs.writeFileSync(__dirname + '/' + fname, lines.join(NL) + NL);
  console.log('wrote', fname, lines.length, 'rows');
}

{
  let r = Float64Array.from(orb.r);
  const a_r = attackLib('sine', orb.t, { A: 0.25, f: 500, t0: 0 });
  for (let i = 0; i < N; i++) r[i] += a_r[i];
  const dot_r = recomputeDotR(r);
  const res = runController(3, r, dot_r, { usePrevState: true });
  dump('fig1.csv', [orb.t, r, res.x, res.u]);
}

{
  const a_s = attackLib('bias', orb.t, { A: -5, t0: 0.1 });
  const res6 = runController(6, orb.r, orb.dot_r, { a_s, usePrevState: true });
  const res1 = runController(1, orb.r, orb.dot_r, { a_s, usePrevState: true });
  dump('fig2.csv', [orb.t, res6.x, res1.x]);
}

{
  let r = Float64Array.from(orb.r);
  const a_r = attackLib('sine', orb.t, { A: 0.5, f: 5000, t0: 0 });
  for (let i = 0; i < N; i++) r[i] += a_r[i];
  const dot_r0 = recomputeDotR(r);
  const res0 = runController(3, r, dot_r0, { usePrevState: true });
  const rMit = rateLimitRef(r, 120);
  const dot_r1 = recomputeDotR(rMit);
  const res1 = runController(3, rMit, dot_r1, { usePrevState: true });
  dump('fig3a.csv', [orb.t, res0.u, res1.u]);
}

{
  const a_s = attackLib('bias', orb.t, { A: -5, t0: 0.1 });
  const res0 = runController(6, orb.r, orb.dot_r, { a_s, usePrevState: true });
  const cusum = { Sp: 0, Sm: 0, drift: 0, tau: 0.005, hold: 5000, latched: 0 };
  const res1 = runController(6, orb.r, orb.dot_r, { a_s, usePrevState: true, detectTh: 0, cusum });
  dump('fig3b.csv', [orb.t, res0.x, res1.x]);
}
`;
eval(src);
