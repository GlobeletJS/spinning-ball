import { interpolateZoom } from "./flyto.js";

export function initFlights(camera, radius) {
  const altScale = 1.0 / (2.0 * radius);
  let t0, t, interp;
  let active = false;

  function flyTo(position) {
    // Scale altitude to be on the same order as lon, lat
    const [lon0, lat0, alt0] = camera.position();
    const p0 = [lon0, lat0, alt0 * altScale];
    const [lon1, lat1, alt1] = position;
    const p1 = [lon1, lat1, alt1 * altScale];

    // TODO: wrap paths around antimeridian
    interp = interpolateZoom(p0, p1);
    t0 = t;
    active = true;
  }

  function update(position, velocity, time) {
    t = time;
    if (!active) return;
    // TODO: apply some tweening or similar
    const dt = (t - t0) / interp.duration;
    if (dt > 1) return (active = false);
    const newPos = interp(dt);
    // Revert scaling of altitude
    newPos[2] /= altScale;

    // TODO: update velocity

    return position.map((c, i) => newPos[i] - c);
  }

  return {
    flyTo, update,
    active: () => active,
    cancel: () => (active = false),
  };
}
