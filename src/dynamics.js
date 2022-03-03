import { initZoom } from "./zoom.js";
import { initRotation, initCoast } from "./rotate.js";

export function initCameraDynamics(ellipsoid, camera, cursor3d, flights) {
  // Velocity is the time differential of camera.position
  const velocity = new Float64Array(3);

  // Initialize some values and working arrays
  let time = 0.0;
  const rayVec = new Float64Array(4);

  // Initialize values & update functions for translations & rotations
  const zoom   = initZoom(ellipsoid, cursor3d);
  const rotate = initRotation(ellipsoid, cursor3d);
  const coast  = initCoast(ellipsoid);

  // Return methods to read/update state
  return {
    update,
    stopZoom: () => velocity.fill(0.0, 2),
    stopCoast: () => velocity.fill(0.0, 0, 2),
  };

  function update(newTime) {
    const deltaTime = newTime - time;
    time = newTime;
    // If timestep too big, wait till next frame to update physics
    if (deltaTime > 0.25) return false;

    const flightStep = flights.update(camera.position(), velocity, time);
    if (cursor3d.isClicked()) flights.cancel();

    const dPos =
      (cursor3d.isClicked()) ? rotate(camera.position(), velocity, deltaTime) :
      (flights.active()) ? flightStep :
      coast(camera.position(), velocity, deltaTime);
    camera.update(dPos);

    const moved = dPos.some(c => c != 0.0);
    if (!cursor3d.isZooming()) return moved;

    flights.cancel();
    // Update 2D screen position of 3D zoom position
    const visible = camera.ecefToScreenRay(rayVec, cursor3d.zoomPosition);
    if (!visible) {
      velocity.fill(0.0, 2); // TODO: is this needed? Or keep coasting?
      cursor3d.stopZoom();
      return moved;
    }

    if (cursor3d.isClicked()) cursor3d.zoomRay.set(rayVec);
    const zoomChange = zoom(camera.position(), velocity, deltaTime);
    camera.update(zoomChange);
    return true;
  }
}
