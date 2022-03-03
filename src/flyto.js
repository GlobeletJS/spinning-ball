export function interpolateZoom(p0, p1) {
  const [ux0, uy0, w0] = p0;
  const [ux1, uy1, w1] = p1;

  const { cosh, sinh, tanh, exp, log, hypot, SQRT2 } = Math;
  const rho = SQRT2;
  const epsilon = 1e-6;

  const dx = ux1 - ux0;
  const dy = uy1 - uy0;
  const du = hypot(dx, dy);

  const rho2du = rho * rho * du;
  const uScale = w0 / rho2du;

  const b0 = (w1 * w1 - w0 * w0 + rho2du * rho2du) / (2 * w0 * rho2du);
  const b1 = (w1 * w1 - w0 * w0 - rho2du * rho2du) / (2 * w1 * rho2du);
  const r0 = log(hypot(b0, 1) - b0);
  const r1 = log(hypot(b1, 1) - b1);
  const coshr0 = cosh(r0);
  const sinhr0 = sinh(r0);

  const rhoS = (du < epsilon) ? log(w1 / w0) : (r1 - r0);

  function special(t) { // Special case for u0 =~ u1
    return [
      ux0 + t * dx,
      uy0 + t * dy,
      w0 * exp(rhoS * t),
    ];
  }

  function general(t) { // General case
    const uarg = rhoS * t + r0;
    const u = uScale * (coshr0 * tanh(uarg) - sinhr0);
    return [
      ux0 + u * dx,
      uy0 + u * dy,
      w0 * coshr0 / cosh(uarg)
    ];
  }

  const interp = (du < epsilon) ? special : general;

  return Object.assign(interp, { duration: rhoS / rho });
}
