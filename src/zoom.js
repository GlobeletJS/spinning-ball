import { oscillatorChange } from "./oscillator.js";
import { dragonflyStalk } from "./dragonfly.js";

export function initZoom(ellipsoid, cursor3d) {
  // Update camera altitude based on target set by mouse wheel events
  //  or two-finger pinch movements
  const { zoomTarget, zoomPosition, zoomRay, stopZoom } = cursor3d;

  const w0 = 14.14; // Natural frequency of oscillator
  const minVelocity = 0.001;
  const minEnergy = 0.5 * minVelocity ** 2; // ASSUME mass == 1

  return function(position, velocity, dt) {
    const stretch = position[2] - zoomTarget();
    const [dz, dVz] = oscillatorChange(stretch, velocity[2], dt, w0);
    velocity[2] += dVz;

    // Scale rotational velocity by the ratio of the height change
    const heightScale = 1 + dz / position[2];
    velocity[0] *= heightScale;
    velocity[1] *= heightScale;

    position[2] += dz;
    const limited = (cursor3d.zoomFixed())
      ? dragonflyStalk(position, zoomPosition, zoomRay, ellipsoid)
      : false;

    const rotating = cursor3d.isClicked() || limited;
    const energy = 0.5 * velocity[2] ** 2 + // Kinetic
      0.5 * (w0 * (stretch + dz)) ** 2;     // Potential
    // Stop if we are already near steady state
    if (!rotating && energy < minEnergy * zoomTarget()) {
      velocity[2] = 0.0;
      stopZoom();
    }
  };
}
