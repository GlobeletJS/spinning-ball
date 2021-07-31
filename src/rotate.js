import * as vec3 from "gl-matrix/vec3";
import { oscillatorChange } from "./oscillator.js";

export function initRotation(ellipsoid, cursor3d) {
  // Update rotations and rotation velocities based on forces applied
  // via a mouse click & drag event
  const w0 = 40.0;
  const extension = new Float64Array(3);
  const { cursorPosition, clickPosition } = cursor3d;

  return function(position, velocity, dt) {
    // Find the displacement of the clicked position on the globe
    // from the current mouse position
    vec3.subtract(extension, cursorPosition, clickPosition);

    // Convert to changes in longitude, latitude, and altitude
    ellipsoid.ecefToDeltaLonLatAlt(extension, extension,
      clickPosition, position);

    const [x, y] = extension;
    const [dx, dVx] = oscillatorChange(x, velocity[0], dt, w0);
    const [dy, dVy] = oscillatorChange(y, velocity[1], dt, w0);

    velocity[0] += dVx;
    velocity[1] += dVy;
    position[0] += dx;
    position[1] += dy;
    return true;   // Position changed, need to re-render
  };
}

export function initCoast(ellipsoid) {
  // Update rotations based on a freely spinning globe (no forces)
  const damping = 3.0;
  const radius = ellipsoid.meanRadius();
  const minSpeed = 0.03;

  return function(position, velocity, dt) {
    // TODO: switch to exact formula? (not finite difference)

    if (vec3.length(velocity) < minSpeed * position[2] / radius) {
      // Rotation has almost stopped. Go ahead and stop all the way
      velocity.fill(0.0);
      return false; // No change to position, no need to re-render
    }

    // Adjust previous velocities for damping over the past time interval
    const dvDamp = -1.0 * damping * dt;
    velocity[0] += velocity[0] * dvDamp;
    velocity[1] += velocity[1] * dvDamp;

    // Update rotations
    position[0] += velocity[0] * dt;
    position[1] += velocity[1] * dt;
    return true;    // Position changed, need to re-render
  };
}
