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
    const [dLon, dVx] = oscillatorChange(x, velocity[0], dt, w0);
    const [dLat, dVy] = oscillatorChange(y, velocity[1], dt, w0);

    velocity[0] += dVx;
    velocity[1] += dVy;

    return new Float64Array([dLon, dLat, 0.0]);
  };
}

export function initCoast(ellipsoid) {
  // Update rotations based on a freely spinning globe (no forces)
  const damping = 3.0; // Viscous damping
  const radius = ellipsoid.meanRadius();
  const minSpeed = 0.03;

  return function(position, velocity, dt) {
    // TODO: switch to exact formula? (not finite difference)

    const speed = Math.hypot(velocity[0], velocity[1]);
    if (speed < minSpeed * position[2] / radius) {
      velocity.fill(0.0, 0, 2);
      return new Float64Array(3); // No change in position
    }

    // Adjust previous velocities for damping over the past time interval
    const dvDamp = -1.0 * damping * dt;
    velocity[0] += velocity[0] * dvDamp;
    velocity[1] += velocity[1] * dvDamp;

    // Return change in position
    const dLon = velocity[0] * dt;
    const dLat = velocity[1] * dt;
    return new Float64Array([dLon, dLat, 0.0]);
  };
}
