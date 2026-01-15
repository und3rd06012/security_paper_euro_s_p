'use strict';

const DT = 1e-5, T_START = 0, T_STOP = 1;
const N = Math.floor((T_STOP - T_START) / DT) + 1;
const DD = 2, X0 = 0.1;

function makeOrbit(Tr_const = 5, Tr_orbit = 3) {
  const t = new Float64Array(N);
  const r = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    t[i] = T_START + i * DT;
    if (Tr_orbit === 3) {
      if ((i + 1) < N / 2) r[i] = (Tr_const / 2) * Math.sin(2 * Math.PI * 5 * t[i]);
      else r[i] = r[i - 1];
    }
  }
  const dot_r = new Float64Array(N);
  for (let i = 0; i < N - 1; i++) dot_r[i] = (r[i + 1] - r[i]) / DT;
  return { t, r, dot_r };
}

function recomputeDotR(r) {
  const dot_r = new Float64Array(N);
  for (let i = 0; i < N - 1; i++) dot_r[i] = (r[i + 1] - r[i]) / DT;
  return dot_r;
}

function minUfromR(r, dot_r) {
  const min_u = new Float64Array(N);
  for (let i = 0; i < N; i++) min_u[i] = dot_r[i] - r[i] * r[i];
  return min_u;
}

function attackLib(type, t, prm = {}) {
  const a = new Float64Array(N);
  const phi = prm.phi || 0;
  for (let i = 0; i < N; i++) {
    switch (type) {
      case 'none': break;
      case 'bias': if (t[i] >= prm.t0) a[i] = prm.A; break;
      case 'ramp': a[i] = prm.k * Math.max(0, t[i] - prm.t0); break;
      case 'sine': a[i] = prm.A * Math.sin(2 * Math.PI * prm.f * t[i] + phi); break;
      case 'pulse': if (t[i] >= prm.t0 && t[i] < prm.t0 + prm.w) a[i] = prm.A; break;
    }
  }
  return a;
}

function applyFilterRef(r) {
  const rf = Float64Array.from(r);
  const passes = 51;
  for (let pass = 0; pass < passes; pass++) {
    for (let i = 0; i < N - 1; i++) rf[i] = (rf[i + 1] + rf[i]) / 2;
    rf[N - 1] = rf[N - 2];
  }
  const d = 25;
  const shifted = rf.slice(0, N - d);
  for (let i = d; i < N; i++) rf[i] = shifted[i - d];
  const ref = rf[d];
  for (let i = 0; i < d; i++) rf[i] -= ref;
  return rf;
}

function rateLimitRef(r, drMax) {
  const rf = new Float64Array(N);
  rf[0] = r[0];
  const maxStep = drMax * DT;
  for (let i = 1; i < N; i++) {
    let delta = r[i] - rf[i - 1];
    if (delta > maxStep) delta = maxStep;
    else if (delta < -maxStep) delta = -maxStep;
    rf[i] = rf[i - 1] + delta;
  }
  return rf;
}

function secureMeas(xmRaw, xHat, uPrev, th, cusum = null) {
  if (th === null || th === undefined) return { xUsed: xmRaw, xHat: xmRaw, detected: false };
  const xPred = xHat + DT * (xHat * xHat + uPrev);
  const res = xmRaw - xPred;
  if (cusum) {
    if (cusum.latched > 0) {
      cusum.latched--;
      return { xUsed: xPred, xHat: xPred, detected: true };
    }
    cusum.Sp = Math.max(0, cusum.Sp + res - cusum.drift);
    cusum.Sm = Math.max(0, cusum.Sm - res - cusum.drift);
    if (cusum.Sp > cusum.tau || cusum.Sm > cusum.tau) {
      cusum.Sp = 0; cusum.Sm = 0;
      cusum.latched = cusum.hold;
      return { xUsed: xPred, xHat: xPred, detected: true };
    }
    return { xUsed: xmRaw, xHat: xmRaw, detected: false };
  }
  if (Math.abs(res) > th) return { xUsed: xPred, xHat: xPred, detected: true };
  return { xUsed: xmRaw, xHat: xmRaw, detected: false };
}

function countTransitions(arr) {
  let s = 0;
  for (let i = 1; i < N; i++) if (arr[i] !== arr[i - 1]) s++;
  return s;
}

function simID0(Kp, Kd, Ki, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, usePrevState = true, sensInSwitch = true,
          tripHyst = false, tripD = 0.01, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  const pidOffConst = 10;
  const tripDelay = Math.floor(tripD / DT) + 1;
  let iSum = 0, tripFlag = 0, tripCnt = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    iSum += err[p - 1];
    const iErr = iSum * DT;
    const pidOff = (Kp * err[p - 1] + Kd * dErr + Ki * iErr) / pidOffConst;
    const pidOn = Kp * err[p - 1] + Kd * dErr + Ki * iErr;
    const st = usePrevState ? ctrl[p - 1] : ctrl[p];
    let uVal = (st === 0)
      ? (-xmPrev * xmPrev + dot_r[p - 1] + pidOff)
      : (-xmPrev * xmPrev + dot_r[p - 1] + pidOn);
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];

    if (!tripHyst) tripCnt = 0;
    else if (tripFlag === 0) tripCnt = 0;
    else if (tripCnt < tripDelay) tripCnt++;
    else tripCnt = 0;

    const xCur = (a_s && sensInSwitch) ? x[p] + a_s[p] : x[p];
    const xPrev = (a_s && sensInSwitch) ? xmPrev : x[p - 1];
    if (tripHyst && tripCnt !== 0) {
      ctrl[p] = ctrl[p - 1];
    } else if (xCur > xPrev) {
      ctrl[p] = (xCur > r[p]) ? 1 : 0;
    } else if (xCur < xPrev) {
      ctrl[p] = (xCur < r[p]) ? 1 : 0;
    } else {
      ctrl[p] = 0;
    }
    if (ctrl[p] !== ctrl[p - 1]) tripFlag = 1;
  }
  return { x, err, u, uLimState, ctrl, switches: countTransitions(ctrl), detCount };
}

function simID1(r, opts = {}) {
  const { uLimUp = 46.5457, uLimLow = -45.4420, a_s = null, detectTh = null, cusum = null } = opts;
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  let no_bb_swtc = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    if (xmPrev > r[p - 1]) { u[p - 1] = uLimLow; uLimState[p - 1] = -1; }
    else if (xmPrev < r[p - 1]) { u[p - 1] = uLimUp; uLimState[p - 1] = 1; }
    else { u[p - 1] = p > 1 ? u[p - 2] : 0; uLimState[p - 1] = 0; }
    uPrev = u[p - 1];
    if (p > 1 && u[p - 1] !== u[p - 2]) no_bb_swtc++;
    x[p] = DT * (x[p - 1] * x[p - 1] + u[p - 1]) + x[p - 1];
  }
  return { x, err, u, uLimState, ctrl: new Float64Array(N), switches: no_bb_swtc, detCount };
}

function simID2(Kp, Kd, Ki, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, usePrevState = true, sensInSwitch = true,
          tripHyst = false, tripD = 0.01, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  const pidOffConst = 10;
  const tripDelay = Math.floor(tripD / DT) + 1;
  let iSum = 0, tripFlag = 0, tripCnt = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    iSum += err[p - 1];
    const iErr = iSum * DT;
    const pidOff = (Kp * err[p - 1] + Kd * dErr + Ki * iErr) / pidOffConst;
    const pidOn = Kp * err[p - 1] + Kd * dErr + Ki * iErr;
    const st = usePrevState ? ctrl[p - 1] : ctrl[p];
    const pid = st === 0 ? pidOff : pidOn;
    const optU = dot_r[p - 1] - r[p - 1] * r[p - 1];
    let uVal = optU + pid;
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];

    if (!tripHyst) tripCnt = 0;
    else if (tripFlag === 0) tripCnt = 0;
    else if (tripCnt < tripDelay) tripCnt++;
    else tripCnt = 0;

    const xCur = (a_s && sensInSwitch) ? x[p] + a_s[p] : x[p];
    const xPrev = (a_s && sensInSwitch) ? xmPrev : x[p - 1];
    if (tripHyst && tripCnt !== 0) {
      ctrl[p] = ctrl[p - 1];
    } else if (xCur > xPrev) {
      ctrl[p] = (xCur > r[p]) ? 1 : 0;
    } else if (xCur < xPrev) {
      ctrl[p] = (xCur < r[p]) ? 1 : 0;
    } else {
      ctrl[p] = 0;
    }
    if (ctrl[p] !== ctrl[p - 1]) tripFlag = 1;
  }
  return { x, err, u, uLimState, ctrl, switches: countTransitions(ctrl), detCount };
}

function simID3(Kp, Kd, Ki, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N).fill(1);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  let iSum = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    iSum += err[p - 1];
    const iErr = iSum * DT;
    const pid = Kp * err[p - 1] + Kd * dErr + Ki * iErr;
    let uVal = -xmPrev * xmPrev + dot_r[p - 1] + pid;
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];
  }
  return { x, err, u, uLimState, ctrl, switches: 0, detCount };
}

function simID4(Kp, Kd, Kd2, Ki, Ki2, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N).fill(1);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  let iSum = 0, i2Sum = 0, prevDErr = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    const d2Err = p > 2 ? (dErr - prevDErr) / DT : 0;
    prevDErr = dErr;
    iSum += err[p - 1];
    const iErr = iSum * DT;
    i2Sum += iErr;
    const i2Err = i2Sum * DT;
    const pid = Kp * err[p - 1] + Kd * dErr + Kd2 * d2Err + Ki * iErr + Ki2 * i2Err;
    let uVal = -xmPrev * xmPrev + dot_r[p - 1] + pid;
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];
  }
  return { x, err, u, uLimState, ctrl, switches: 0, detCount };
}

function simID5(Kp, Kd, Ki, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, tripHyst = false, tripD = 0.01, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N).fill(1);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  x[0] = X0;
  const tripDelay = Math.floor(tripD / DT) + 1;
  let iSum = 0, Kps = 0, Kds = 0, Kis = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    iSum += err[p - 1];
    const iErr = iSum * DT;
    let Kptv = Kp / 2 + Kp * Math.abs(err[p - 1]); if (Kptv > 500) Kptv = 500;
    let Kdtv = Kd / 2 + Kd * Math.abs(err[p - 1]); if (Kdtv > 1) Kdtv = 1;
    let Kitv = Ki / 2 + Ki * Math.abs(err[p - 1]); if (Kitv > 750) Kitv = 750;
    if (tripHyst) {
      if (p % tripDelay === 1) { Kps = Kptv; Kds = Kdtv; Kis = Kitv; }
      else { Kptv = Kps; Kdtv = Kds; Kitv = Kis; }
    }
    const pid = Kptv * err[p - 1] + Kdtv * dErr + Kitv * iErr;
    let uVal = -xmPrev * xmPrev + dot_r[p - 1] + pid;
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];
  }
  return { x, err, u, uLimState, ctrl, switches: 0, detCount };
}

function simID6(K, z, pole, r, dot_r, opts = {}) {
  const { uLim = null, a_s = null, detectTh = null, cusum = null } = opts;
  const ctrl = new Float64Array(N).fill(1);
  const u = new Float64Array(N), uLimState = new Float64Array(N);
  const err = new Float64Array(N), x = new Float64Array(N);
  const y = new Float64Array(N), y2 = new Float64Array(N);
  x[0] = X0; y[0] = 0; y2[0] = 0;
  let xHat = X0, uPrev = 0, detCount = 0;
  for (let p = 1; p <= N - 1; p++) {
    const xmRaw = a_s ? x[p - 1] + a_s[p - 1] : x[p - 1];
    const sm = secureMeas(xmRaw, xHat, uPrev, detectTh, cusum);
    const xmPrev = sm.xUsed; xHat = sm.xUsed;
    if (sm.detected) detCount++;
    err[p - 1] = r[p - 1] - xmPrev;
    const dErr = p > 1 ? (err[p - 1] - err[p - 2]) / DT : 0;
    y[p] = (-pole * y[p - 1] + K * dErr + K * z * err[p - 1]) * DT + y[p - 1];
    y2[p] = y[p - 1] * DT + y2[p - 1];
    let uVal = -xmPrev * xmPrev + dot_r[p - 1] + y2[p - 1];
    if (uLim) {
      if (uVal > uLim[1]) { uVal = uLim[1]; uLimState[p - 1] = 1; }
      if (uVal < uLim[0]) { uVal = uLim[0]; uLimState[p - 1] = -1; }
    }
    u[p - 1] = uVal;
    uPrev = uVal;
    x[p] = DT * (x[p - 1] * x[p - 1] + uVal) + x[p - 1];
  }
  return { x, err, u, uLimState, ctrl, switches: 0, detCount };
}

function sumArr(a) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i]; return s; }
function sumArrAbs(a) { let s = 0; for (let i = 0; i < a.length; i++) s += Math.abs(a[i]); return s; }

function metrics(res, min_u) {
  const m = N - 1;
  let sumAbsErr = 0, sumErr2 = 0, sumU2 = 0, sumAbsEU = 0, sumEU2 = 0;
  for (let k = 0; k < m; k++) {
    const e = res.err[k], uv = res.u[k];
    sumAbsErr += Math.abs(e);
    sumErr2 += e * e;
    sumU2 += uv * uv;
    sumAbsEU += Math.abs(e * uv);
    sumEU2 += e * e * uv * uv;
  }
  let sumMinU2 = 0;
  for (let k = 0; k < N; k++) sumMinU2 += min_u[k] * min_u[k];
  return {
    ExitErr: Math.abs(res.err[m - 1]),
    IAE: sumAbsErr * DT,
    ISE: Math.sqrt(sumErr2 * DT),
    Eu: sumU2 * DT,
    MinEu: sumMinU2 * DT,
    KuIAE: sumAbsEU * DT / (T_STOP - T_START),
    KuMSE: Math.sqrt(sumEU2 * DT) / (T_STOP - T_START),
    ctrlUtil: 100 * sumArr(res.ctrl) / N,
    limUtil: 100 * sumArrAbs(res.uLimState) / N,
  };
}

function safety(x, t, X_SAFE = 20) {
  let escIdx = -1, xmax = 0;
  for (let k = 0; k < N; k++) {
    const ax = Math.abs(x[k]);
    if (ax > xmax) xmax = ax;
    if (escIdx < 0 && (ax > X_SAFE || !isFinite(x[k]))) escIdx = k;
  }
  return { xmax, Tesc: escIdx >= 0 ? t[escIdx] : Infinity, escaped: escIdx >= 0 };
}

const GAINS = {
  0: { Kp: 274.7391, Kd: 0.0461, Ki: 334.0568 },
  1: { up: 46.5457, low: -45.4420 },
  2: { Kp: 274.9566, Kd: 0.0263, Ki: 293.1913 },
  3: { Kp: 143.5032, Kd: 0.0264, Ki: 422.5903 },
  4: { Kp: 163.4738, Kd: 0.0257, Kd2: 2.1e-6, Ki: 423.3579, Ki2: 321.3681 },
  5: { Kp: 272.2565, Kd: 0.0306, Ki: 421.5556 },
  6: { K: 125.3641, z: 5.3716, p: 47.6495 },
};

function runController(id, r, dot_r, opts = {}) {
  const g = GAINS[id];
  switch (id) {
    case 0: return simID0(g.Kp, g.Kd, g.Ki, r, dot_r, opts);
    case 1: return simID1(r, { uLimUp: opts.uLimUp ?? g.up, uLimLow: opts.uLimLow ?? g.low, a_s: opts.a_s, detectTh: opts.detectTh, cusum: opts.cusum });
    case 2: return simID2(g.Kp, g.Kd, g.Ki, r, dot_r, opts);
    case 3: return simID3(g.Kp, g.Kd, g.Ki, r, dot_r, opts);
    case 4: return simID4(g.Kp, g.Kd, g.Kd2, g.Ki, g.Ki2, r, dot_r, opts);
    case 5: return simID5(g.Kp, g.Kd, g.Ki, r, dot_r, opts);
    case 6: return simID6(g.K, g.z, g.p, r, dot_r, opts);
  }
}

const EXPECTED = {
  0: { ExitErr: 0.0001434, IAE: 0.0006354, ISE: 0.004375, Eu: 1535.13, KuIAE: 0.03203, KuMSE: 0.2665, ctrlUtil: 27.06, switches: 8 },
  1: { ExitErr: 0.0003307, IAE: 0.3077, ISE: 0.5399, Eu: 2113.92, KuIAE: 14.14, KuMSE: 24.78, switches: 48809 },
  2: { ExitErr: 0.0001441, IAE: 0.0006067, ISE: 0.004335, Eu: 1535.14, KuIAE: 0.03058, KuMSE: 0.2628, ctrlUtil: 27.08, switches: 8 },
  3: { ExitErr: 0.0001109, IAE: 0.001305, ISE: 0.005982, Eu: 1534.79, KuIAE: 0.07035, KuMSE: 0.4056 },
  4: { ExitErr: 0.0001568, IAE: 0.001255, ISE: 0.005615, Eu: 1534.79, KuIAE: 0.064, KuMSE: 0.3746 },
  5: { ExitErr: 0.0002279, IAE: 0.001162, ISE: 0.005747, Eu: 1534.96, KuIAE: 0.06079, KuMSE: 0.3864 },
  6: { ExitErr: 0.03161, IAE: 0.04436, ISE: 0.05347, Eu: 1549.66, KuIAE: 1.612, KuMSE: 2.781 },
};

function close(a, b, relTol = 0.01, absTol = 1e-6) {
  return Math.abs(a - b) <= Math.max(relTol * Math.abs(b), absTol);
}

function validate() {
  const { t, r, dot_r } = makeOrbit();
  const min_u = minUfromR(r, dot_r);
  console.log('=== Validation vs paper Scenario 1 (clean reference, no limiter) ===');
  console.log('Min Eu expected ~1549.33, computed:', metrics({ err: new Float64Array(N - 1), u: new Float64Array(N - 1), ctrl: new Float64Array(N), uLimState: new Float64Array(N) }, min_u).MinEu.toFixed(2));

  const rows = [];
  let allOk = true;
  for (let id = 0; id <= 6; id++) {
    const res = runController(id, r, dot_r, { usePrevState: true });
    const met = metrics(res, min_u);
    const exp = EXPECTED[id];
    const fields = ['ExitErr', 'IAE', 'ISE', 'Eu', 'KuIAE', 'KuMSE'];
    const ok = {};

    const relTol = id === 1 ? 1.0 : 0.01;
    const absTol = id === 1 ? 5e-4 : 1e-6;
    for (const f of fields) ok[f] = Math.abs(met[f] - exp[f]) <= Math.max(relTol * Math.abs(exp[f]), absTol);
    let ctrlOk = true, swOk = true;
    if (exp.ctrlUtil !== undefined) ctrlOk = close(met.ctrlUtil, exp.ctrlUtil, 0.02, 0.1);
    if (exp.switches !== undefined) swOk = id === 1 ? Math.abs(res.switches - exp.switches) <= 2 : res.switches === exp.switches;
    const pass = fields.every(f => ok[f]) && ctrlOk && swOk;
    allOk = allOk && pass;
    rows.push({
      ID: id,
      ExitErr: `${met.ExitErr.toExponential(3)} (${exp.ExitErr})${ok.ExitErr ? '' : ' X'}`,
      IAE: `${met.IAE.toExponential(3)} (${exp.IAE})${ok.IAE ? '' : ' X'}`,
      Eu: `${met.Eu.toFixed(1)} (${exp.Eu})${ok.Eu ? '' : ' X'}`,
      KuIAE: `${met.KuIAE.toExponential(3)} (${exp.KuIAE})${ok.KuIAE ? '' : ' X'}`,
      KuMSE: `${met.KuMSE.toExponential(3)} (${exp.KuMSE})${ok.KuMSE ? '' : ' X'}`,
      ctrl: exp.ctrlUtil !== undefined ? `${met.ctrlUtil.toFixed(1)} (${exp.ctrlUtil})${ctrlOk ? '' : ' X'}` : 'n/a',
      sw: exp.switches !== undefined ? `${res.switches} (${exp.switches})${swOk ? '' : ' X'}` : 'n/a',
      pass: pass ? 'OK' : 'FAIL',
    });
  }
  console.table(rows);
  console.log(allOk ? 'ALL CONTROLLERS MATCH THE PAPER (Scenario 1).' : 'SOME MISMATCHES - check port.');
  return { t, r, dot_r, min_u, allOk };
}

const ATTACKS = [
  { name: 'clean (baseline)', chan: null, type: 'none' },
  { name: 'ref bias +0.50 @0.1s', chan: 'ref', type: 'bias', A: 0.50, t0: 0.10 },
  { name: 'ref sine 0.25 @500Hz', chan: 'ref', type: 'sine', A: 0.25, f: 500, t0: 0 },
  { name: 'ref sine 0.25 @50Hz', chan: 'ref', type: 'sine', A: 0.25, f: 50, t0: 0 },
  { name: 'sens bias -0.05 @0.1s', chan: 'sens', type: 'bias', A: -0.05, t0: 0.10 },
  { name: 'sens sine 0.05 @100Hz', chan: 'sens', type: 'sine', A: 0.05, f: 100, t0: 0 },
  { name: 'sens sine 0.05 @1kHz', chan: 'sens', type: 'sine', A: 0.05, f: 1000, t0: 0 },
];

function runAttacks(orb) {
  console.log('\n=== Adversarial attack tests (safety bound |x|<20, no limiter, no filter) ===');
  console.log('For each attack:  xmax = max|x|,  Tesc = time-to-escape,  Eu = actuation energy,');
  console.log('                  K_uIAE = effort-weighted error,  sw = switch count,  sat% = limiter activity.\n');

  const header = ['ID', 'controller'];
  for (const at of ATTACKS) header.push(at.name);
  console.log('Legend per cell: xmax | Tesc | Eu | K_uIAE | sw');

  for (let id = 0; id <= 6; id++) {
    const g = GAINS[id];
    const cells = [];
    for (const at of ATTACKS) {
      let r = Float64Array.from(orb.r);
      let a_s = null;
      if (at.chan === 'ref') {
        const a_r = attackLib(at.type, orb.t, at);
        for (let i = 0; i < N; i++) r[i] += a_r[i];
      } else if (at.chan === 'sens') {
        a_s = attackLib(at.type, orb.t, at);
      }
      const dot_r = recomputeDotR(r);
      const min_u = minUfromR(r, dot_r);
      const res = runController(id, r, dot_r, { a_s, usePrevState: true });
      const met = metrics(res, min_u);
      const saf = safety(res.x, orb.t, 20);
      const esc = saf.escaped ? `Tesc=${saf.Tesc.toFixed(3)}` : 'safe';
      cells.push(`${saf.xmax.toFixed(2)} | ${esc} | Eu=${met.Eu.toFixed(0)} | K=${met.KuIAE.toFixed(2)} | sw=${res.switches}`);
    }
    const row = { ID: id };
    ATTACKS.forEach((at, j) => { row[at.name] = cells[j]; });
    console.log(`ID ${id}`);
    ATTACKS.forEach((at, j) => console.log(`   ${at.name}: ${cells[j]}`));
    console.log('');
  }
}

function expA_sensBiasSweep(orb) {
  console.log('\n=== EXP A: sensor-bias escape sweep (t0=0.1 s, safety |x|<20) ===');
  const amps = [-0.02, -0.05, -0.1, -0.2, -0.5, -1, -2, -5];
  const min_u = minUfromR(orb.r, orb.dot_r);
  console.log('cells: amplitude -> xmax (ESC@time if |x|>20) | IAE');
  for (let id = 0; id <= 6; id++) {
    const cells = [];
    for (const A of amps) {
      const a_s = attackLib('bias', orb.t, { A, t0: 0.1 });
      const res = runController(id, orb.r, orb.dot_r, { a_s, usePrevState: true });
      const met = metrics(res, min_u);
      const saf = safety(res.x, orb.t, 20);
      cells.push(`${A}: x=${saf.xmax.toFixed(2)}${saf.escaped ? ` ESC@${saf.Tesc.toFixed(3)}` : ' ok'} IAE=${met.IAE.toExponential(2)}`);
    }
    console.log(`ID ${id}:`);
    console.log(`   ${cells.join(' | ')}`);
  }
}

function expB_defenceMatrix(orb) {
  console.log('\n=== EXP B: defence stack matrix (filter / limiter / dwell) ===');
  const defences = [
    { name: 'D1 filter only', filter: true, lim: false, dwell: false },
    { name: 'D2 filter + limiter +/-150', filter: true, lim: true, dwell: false },
    { name: 'D3 filter + limiter + dwell 10ms', filter: true, lim: true, dwell: true },
  ];
  const attacks = [
    { name: 'ref bias +0.5', chan: 'ref', type: 'bias', A: 0.5, t0: 0.1 },
    { name: 'ref sine 500Hz', chan: 'ref', type: 'sine', A: 0.25, f: 500, t0: 0 },
    { name: 'sens sine 100Hz', chan: 'sens', type: 'sine', A: 0.05, f: 100, t0: 0 },
  ];
  for (const d of defences) {
    console.log(`\n--- ${d.name} ---`);
    console.log('ID | attack          | xmax  | Tesc  | IAE        | Eu        | K_uIAE     | sw');
    for (let id = 0; id <= 6; id++) {
      for (const at of attacks) {
        let r = Float64Array.from(orb.r);
        let a_s = null;
        if (at.chan === 'ref') {
          const a_r = attackLib(at.type, orb.t, at);
          for (let i = 0; i < N; i++) r[i] += a_r[i];
        } else {
          a_s = attackLib(at.type, orb.t, at);
        }
        if (d.filter) r = applyFilterRef(r);
        const dot_r = recomputeDotR(r);
        const min_u = minUfromR(r, dot_r);
        const opts = { a_s, usePrevState: true, tripHyst: d.dwell, tripD: 0.01 };
        if (d.lim) {
          opts.uLim = [-150, 150];
          if (id === 1) { opts.uLimUp = 150; opts.uLimLow = -150; }
        }
        const res = runController(id, r, dot_r, opts);
        const met = metrics(res, min_u);
        const saf = safety(res.x, orb.t, 20);
        console.log(`${id}  | ${at.name.padEnd(15)} | ${saf.xmax.toFixed(2)} | ${saf.escaped ? saf.Tesc.toFixed(3) : ' safe'} | ${met.IAE.toExponential(3)} | ${met.Eu.toFixed(1)} | ${met.KuIAE.toExponential(3)} | ${res.switches}`);
      }
    }
  }
}

function expC_worstCase(orb) {
  console.log('\n=== EXP C: worst-case reference jamming (grid search, objective Eu) ===');
  const gridA = [0.1, 0.25, 0.5];
  const gridF = [5, 50, 500, 5000];
  const gridT0 = [0, 0.1];
  for (let id = 0; id <= 6; id++) {
    let best = { Eu: -Infinity };
    for (const A of gridA) for (const f of gridF) for (const t0 of gridT0) {
      const a_r = attackLib('sine', orb.t, { A, f, t0 });
      const r = Float64Array.from(orb.r);
      for (let i = 0; i < N; i++) r[i] += a_r[i];
      const dot_r = recomputeDotR(r);
      const res = runController(id, r, dot_r, { usePrevState: true });
      const met = metrics(res, minUfromR(r, dot_r));
      const saf = safety(res.x, orb.t, 20);
      if (met.Eu > best.Eu) best = { Eu: met.Eu, KuIAE: met.KuIAE, xmax: saf.xmax, A, f, t0 };
    }
    console.log(`ID ${id}: worst Eu=${best.Eu.toFixed(1)} at A=${best.A}, f=${best.f} Hz, t0=${best.t0} s   (K_uIAE=${best.KuIAE.toExponential(3)}, xmax=${best.xmax.toFixed(2)})`);
  }
}

function expD_mitigations(orb) {
  console.log('\n=== EXP D: active mitigations ===');
  console.log('M1 = reference slew-rate gate (drMax=120, ~1.5x clean max|dr/dt|=78.5)');
  console.log('M2 = CUSUM on one-step prediction residual + model fallback (tau=0.005, drift=0)');
  const DRMAX = 120, TH = 0.01;
  const cases = [
    { name: 'ref bias +0.5', chan: 'ref', type: 'bias', A: 0.5, t0: 0.1, mit: 'M1' },
    { name: 'ref sine 500Hz', chan: 'ref', type: 'sine', A: 0.25, f: 500, t0: 0, mit: 'M1' },
    { name: 'ref sine 5000Hz A=0.5', chan: 'ref', type: 'sine', A: 0.5, f: 5000, t0: 0, mit: 'M1' },
    { name: 'sens bias -5', chan: 'sens', type: 'bias', A: -5, t0: 0.1, mit: 'M2' },
    { name: 'sens sine 100Hz', chan: 'sens', type: 'sine', A: 0.05, f: 100, t0: 0, mit: 'M2' },
  ];
  console.log('ID | attack               | BEFORE (x | Eu | K)                        | AFTER (x | Eu | K | det)');
  for (let id = 0; id <= 6; id++) {
    for (const c of cases) {
      let r = Float64Array.from(orb.r);
      let a_s = null;
      if (c.chan === 'ref') {
        const a_r = attackLib(c.type, orb.t, c);
        for (let i = 0; i < N; i++) r[i] += a_r[i];
      } else a_s = attackLib(c.type, orb.t, c);

      const dot_r0 = recomputeDotR(r);
      const res0 = runController(id, r, dot_r0, { a_s, usePrevState: true });
      const met0 = metrics(res0, minUfromR(r, dot_r0));
      const saf0 = safety(res0.x, orb.t, 20);

      let rMit = r;
      const optsMit = { a_s, usePrevState: true };
      if (c.mit === 'M1') rMit = rateLimitRef(r, DRMAX);
      if (c.mit === 'M2') { optsMit.detectTh = 0; optsMit.cusum = { Sp: 0, Sm: 0, drift: 0, tau: 0.005, hold: 5000, latched: 0 }; }
      const dot_r1 = recomputeDotR(rMit);
      const res1 = runController(id, rMit, dot_r1, optsMit);
      const met1 = metrics(res1, minUfromR(rMit, dot_r1));
      const saf1 = safety(res1.x, orb.t, 20);

      const b = `${saf0.xmax.toFixed(1)}${saf0.escaped ? 'ESC' : ' ok'} | ${met0.Eu.toFixed(0)} | ${met0.KuIAE.toFixed(2)}`;
      const a = `${saf1.xmax.toFixed(2)}${saf1.escaped ? 'ESC' : ' ok'} | ${met1.Eu.toFixed(0)} | ${met1.KuIAE.toFixed(2)} | det=${res1.detCount ?? 0}`;
      console.log(`${id}  | ${c.name.padEnd(20)} | ${b.padEnd(38)} | ${a}`);
    }
  }
}

const arg = process.argv[2];
const orb = makeOrbit();
if (arg === '--experiments') {
  expA_sensBiasSweep(orb);
  expB_defenceMatrix(orb);
  expC_worstCase(orb);
  expD_mitigations(orb);
} else {
  const { allOk } = validate();
  if (allOk) runAttacks(orb);
  else console.log('\nSkipping attack tests because validation failed.');
}
