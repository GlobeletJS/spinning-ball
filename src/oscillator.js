export function fdOscillator(pos, vel, ext, springConst, damping, dt, i1, i2) {
  // Update position and velocity for a damped oscillator, using a
  // Euler-Cromer finite difference method. See
  // www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node2.html

  for (let i = i1; i <= i2; i++) {
    // Adjust previous velocity for damping over past time interval
    vel[i] -= vel[i] * damping * dt;
    // Update velocities based on accelerations over past time interval
    vel[i] -= ext[i] * springConst * dt;
    // Update position
    pos[i] += vel[i] * dt;
  }
  return;
}

export function updateOscillator(pos, vel, ext, w0, dt, i1, i2) {
  // Update position and velocity for a critically damped oscillator, following
  // http://mathworld.wolfram.com/CriticallyDampedSimpleHarmonicMotion.html

  // Inputs/outputs pos, vel are pointers to arrays
  // Inputs w0, t are primitive floating point values, indicating the
  //   natural frequency of the oscillator and the time step
  // Inputs i1, i2 are primitive integer values, indicating components to update

  const expTerm = Math.exp( -w0 * dt );

  for (let i = i1; i <= i2; i++) {
    const tmp = (vel[i] + w0 * ext[i]) * dt * expTerm;
    vel[i] += (expTerm - 1) * vel[i] - w0 * tmp;
    pos[i] += (expTerm - 1) * ext[i] + tmp;
  }
  return;
}
