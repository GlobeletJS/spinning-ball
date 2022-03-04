import { wrapLongitude } from "./bounds.js";
import { interpolateZoom } from "./zinterp.js";

export function initFlights(params, camera) {
  const { ellipsoid, bounds } = params;

  const wScale = 2.0 / (ellipsoid.meanRadius() * Math.PI);
  let t, interp, active = false;

  function flyTo(position) {
    if (!bounds.check(position)) {
      return console.log("spinningBall.flyTo: position out of bounds");
    }

    const [lon0, lat0, alt0] = camera.position();
    const [lon1, lat1, alt1] = position;
    // Scale altitude to be on the same order as lon, lat, and wrap longitude
    const p0 = [lon0, lat0, alt0 * wScale];
    const p1 = [lon0 + wrapLongitude(lon1 - lon0), lat1, alt1 * wScale];

    interp = interpolateZoom(p0, p1);
    t = 0.0;
    active = true;
  }

  function update(position, velocity, dt) {
    if (!active) return;
    t = Math.min(1.0, t + dt / interp.duration);
    const newPos = interp(t);
    newPos[2] /= wScale; // Revert scaling applied in flyTo

    if (t == 1.0) active = false;
    return position.map((c, i) => newPos[i] - c);
  }

  return {
    flyTo, update,
    active: () => active,
    cancel: () => (active = false),
  };
}
