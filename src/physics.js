import { initECEF } from "./ecef.js";
import { initZoom } from "./zoom.js";
import { initRotation, initCoast } from "./rotate.js";
import { initProjector } from "./projector.js";

export function initCameraDynamics(params, cursor3d) {
  const { view, ellipsoid, initialPosition } = params;

  // Position & velocity are computed in latitude & longitude in radians, and
  //   altitude defined by distance along surface normal, in the same length
  //   units as semiMajor and semiMinor in ellipsoid.js
  const position = new Float64Array(initialPosition);
  const velocity = new Float64Array(3);

  // Initialize ECEF position, rotation matrix, inverse, and update method
  const ecef = initECEF(ellipsoid, position);

  // Initialize transforms from ellipsoid to screen positions
  const projector = initProjector(ellipsoid, ecef.position, ecef.inverse, view);

  // Initialize some values and working arrays
  let time = 0.0;
  const rayVec = new Float64Array(4);

  // Initialize values & update functions for translations & rotations
  const zoom   = initZoom(ellipsoid, cursor3d);
  const rotate = initRotation(ellipsoid, cursor3d);
  const coast  = initCoast(ellipsoid);

  // Return methods to read/update state
  return {
    position, // WARNING: Exposes local array to changes from outside

    ecefPos: ecef.position,
    rotation: ecef.rotation,
    inverse: ecef.inverse,

    lonLatToScreenXY: projector.lonLatToScreenXY,

    update,
    stopZoom: () => velocity.fill(0.0, 2),
    stopCoast: () => velocity.fill(0.0, 0, 2),
  };

  function update(newTime) {
    const deltaTime = newTime - time;
    time = newTime;
    // If timestep too big, wait till next frame to update physics
    if (deltaTime > 0.25) return false;

    const rotation = (cursor3d.isClicked())
      ? rotate(position, velocity, deltaTime)
      : coast(position, velocity, deltaTime);
    position[0] += rotation[0];
    position[1] += rotation[1];
    const needToRender = rotation.some(c => c != 0.0) || cursor3d.isZooming();

    if (cursor3d.isZooming()) {
      // Update ECEF position and rotation/inverse matrices
      ecef.update(position);
      // Update 2D screen position of 3D zoom position
      const visible = projector.ecefToScreenRay(rayVec, cursor3d.zoomPosition);
      if (visible) {
        if (cursor3d.isClicked()) cursor3d.zoomRay.set(rayVec);
        zoom(position, velocity, deltaTime);
      } else {
        velocity.fill(0.0, 2); // TODO: is this needed? Or keep coasting?
        cursor3d.stopZoom();
      }
    }

    if (needToRender) ecef.update(position);
    return needToRender;
  }
}
