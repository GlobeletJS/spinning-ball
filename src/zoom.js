import { updateOscillator } from "./oscillator.js";
import { dragonflyStalk } from "./dragonfly.js";

export function initZoom(ellipsoid, cursor3d) {
  const { zoomTarget, zoomPosition, zoomRay, stopZoom } = cursor3d;

  // Update camera altitude based on target set by mouse wheel events
  //  or two-finger pinch movements
  const w0 = 14.14; // Natural frequency of oscillator
  const minVelocity = 0.001;

  // NOTE: everything below ASSUMES mass = 1
  const minEnergy = 0.5 * minVelocity * minVelocity;
  const dPos = new Float64Array(3);

  return function(position, velocity, deltaTime) {
    const oldAltitude = position[2];
    const targetHeight = zoomTarget();

    dPos[2] = position[2] - targetHeight;
    updateOscillator(position, velocity, dPos, w0, deltaTime, 2, 2);

    const limited = (cursor3d.zoomFixed())
      ? dragonflyStalk(position, zoomPosition, zoomRay, ellipsoid)
      : false;

    // Scale rotational velocity by the ratio of the height change
    const heightScale = position[2] / oldAltitude;
    velocity[0] *= heightScale;
    velocity[1] *= heightScale;

    if (cursor3d.isClicked() || limited) return;

    // Stop if we are already near steady state
    const kineticE = 0.5 * velocity[2] ** 2;
    const extension = position[2] - targetHeight;
    const potentialE = 0.5 * (w0 * extension) ** 2;
    if (kineticE + potentialE < minEnergy * targetHeight) {
      velocity[2] = 0.0;
      stopZoom();
    }
  };
}
