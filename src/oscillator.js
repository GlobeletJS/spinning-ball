export function oscillatorChange(x, v, t, w0) {
  // For a critically damped oscillator with natural frequency w0, find
  // the change in position x and velocity v over timestep t.  See
  // https://mathworld.wolfram.com/CriticallyDampedSimpleHarmonicMotion.html
  const expTerm = Math.exp(-w0 * t);
  const Bt = (v + w0 * x) * t;
  const dx = (x + Bt) * expTerm - x;
  const dv = (v - w0 * Bt) * expTerm - v;
  return [dx, dv];
}
