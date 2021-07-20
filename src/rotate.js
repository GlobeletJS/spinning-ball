import * as vec3 from "gl-matrix/vec3";
import { updateOscillator } from "./oscillator.js";

export function initRotation(ellipsoid) {
  // Update rotations and rotation velocities based on forces applied
  // via a mouse click & drag event
  const w0 = 40.0;
  const extension = new Float64Array(3);

  return function(position, velocity, mouse3d, deltaTime) {
    // Input mouse3d is a pointer to a mouse object
    // Inputs position, velocity are pointers to vec3s
    // Input deltaTime is a primitive floating point value

    // Find the displacement of the clicked position on the globe
    // from the current mouse position
    vec3.subtract(extension, mouse3d.position, mouse3d.clickPosition);

    // Convert to changes in longitude, latitude, and altitude
    ellipsoid.ecefToDeltaLonLatAlt(extension, extension,
      mouse3d.clickPosition, position);
    // Ignore altitude change for now
    extension[2] = 0.0;

    updateOscillator(position, velocity, extension, w0, deltaTime, 0, 1);
    return true;   // Position changed, need to re-render
  };
}

export function initCoast(ellipsoid) {
  // Update rotations based on a freely spinning globe (no forces)
  const damping = 3.0;
  const radius = ellipsoid.meanRadius();
  const minSpeed = 0.03;

  return function(position, velocity, deltaTime) {
    // Inputs rotation, rotationVel are pointers to 3-element arrays
    // Input deltaTime is a primitive value (floating point)
    // TODO: switch to exact formula? (not finite difference)

    if (vec3.length(velocity) < minSpeed * position[2] / radius) {
      // Rotation has almost stopped. Go ahead and stop all the way
      velocity.fill(0.0);
      return false; // No change to position, no need to re-render
    }

    // Adjust previous velocities for damping over the past time interval
    const dvDamp = -1.0 * damping * deltaTime;
    velocity[0] += velocity[0] * dvDamp;
    velocity[1] += velocity[1] * dvDamp;

    // Update rotations
    position[0] += velocity[0] * deltaTime;
    position[1] += velocity[1] * deltaTime;
    return true;    // Position changed, need to re-render
  };
}
