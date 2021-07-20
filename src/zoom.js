import * as vec3 from "gl-matrix/vec3";
import { updateOscillator } from "./oscillator.js";

export function initZoom(ellipsoid) {
  // Update camera altitude based on target set by mouse wheel events
  //  or two-finger pinch movements

  const w0 = 14.14; // Natural frequency of oscillator
  const minVelocity = 0.001;

  // NOTE: everything below ASSUMES mass = 1
  const minEnergy = 0.5 * minVelocity * minVelocity;
  const dPos = new Float64Array(3);

  return function(position, velocity, cursor3d, deltaTime, track) {
    // Input cursor3d is a pointer to an object
    // Inputs position, velocity are pointers to 3-element arrays
    // Input deltaTime is a primitive floating point value

    let targetHeight = cursor3d.zoomTarget();

    // Save old altitude
    const oldAltitude = position[2];

    dPos[2] = position[2] - targetHeight;
    updateOscillator(position, velocity, dPos, w0, deltaTime, 2, 2);

    let limited;
    if (track) {
      // Adjust rotation to keep zoom location fixed on screen
      dPos.set(position);
      dragonflyStalk(dPos, cursor3d.zoomRay, cursor3d.zoomPosition, ellipsoid);
      // Restrict size of rotation in one time step
      vec3.subtract(dPos, dPos, position);
      limited = limitRotation(dPos);
      vec3.add(position, position, dPos);
    }

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
      targetHeight = position[2]; // TODO: unnecessary?
      velocity[2] = 0.0;
      cursor3d.stopZoom();
    }
  };
}

function limitRotation(dPos) {
  // Input dPos is a pointer to a 2-element array containing lon, lat changes
  const { abs, min, max, PI } = Math;
  const maxRotation = 0.15;

  // Check for longitude value crossing antimeridian
  if (dPos[0] >  PI) dPos[0] -= 2.0 * PI;
  if (dPos[0] < -PI) dPos[0] += 2.0 * PI;

  if (abs(dPos[0]) < maxRotation) return false;

  const tmp = min(max(-maxRotation, dPos[0]), maxRotation) / dPos[0];
  dPos[0] *= tmp;
  dPos[1] *= tmp;
  return true;
}

// Given a 3D scene coordinate over which a zoom action was initiated,
// and a distance between the screen and the center of the 3D scene,
// compute the rotations required to align the 3D coordinate along
// the original screen ray.  See
// https://en.wikipedia.org/wiki/Dragonfly#Motion_camouflage
// TODO: Clean this up. Just use difference of lat/lon under ray?
function dragonflyStalk(outRotation, ray, scenePos, ellipsoid) {
  const { abs, hypot, asin, cos, atan2 } = Math;

  const [sceneX, sceneY, sceneZ] = scenePos;

  // Find the ray-sphere intersection in unrotated model space coordinates
  const target = new Float64Array(3);
  const unrotatedCamPos = [0.0, 0.0, outRotation[2] + hypot(...scenePos)];
  const onEllipse = ellipsoid.shoot(target, unrotatedCamPos, ray);
  if (!onEllipse) return; // No intersection!

  // Find the rotation about the y-axis required to bring scene point into
  // the  x = target[0]  plane
  // First find distance of scene point from scene y-axis
  const sceneR = hypot(sceneX, sceneZ);
  // If too short, exit rather than tipping poles out of y-z plane
  if (sceneR < abs(target[0])) return;
  const targetRotY = asin(target[0] / sceneR); // Y-angle of target point
  outRotation[0] =
    atan2(sceneX, sceneZ) -     // Y-angle of scene vector
    // asin( target[0] / sceneR ); // Y-angle of target point
    targetRotY;

  // We now know the x and y coordinates of the scene vector after rotation
  // around the y-axis: (x = target[0], y = scenePos[1])
  // Find the z-coordinate so we can compute the remaining required rotation
  const zRotated = sceneR * cos(targetRotY);

  // Find the rotation about the screen x-axis required to bring the scene
  // point into the target y = target[1] plane
  // Assumes 0 angle is aligned along Z, and angle > 0 is rotation toward -y !
  outRotation[1] =
    atan2(-1 * target[1], target[2]) - // X-angle of target point
    atan2(-1 * sceneY, zRotated);      // X-angle of scene vector
}
