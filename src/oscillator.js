export function updateOscillator(pos, vel, ext, w0, dt, i1, i2) {
  // Inputs/outputs pos, vel are pointers to arrays
  // Inputs w0, t are primitive floating point values, indicating the
  //   natural frequency of the oscillator and the time step
  // Inputs i1, i2 are primitive integer values, indicating components to update

  for (let i = i1; i <= i2; i++) {
    const [dx, dv] = oscillatorChange(ext[i], vel[i], dt, w0);
    vel[i] += dv;
    pos[i] += dx;
  }
}

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
