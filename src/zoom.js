import * as vec3 from "gl-matrix/vec3";
import { updateOscillator } from "./oscillator.js";

// initZoom: Update camera altitude based on target set by mouse wheel events
//  or two-finger pinch movements
export function initZoom( ellipsoid ) {
  const w0 = 14.14; // Natural frequency of oscillator
  const minVelocity = 0.001;
  const maxRotation = 0.15;

  // NOTE: everything below ASSUMES mass = 1
  var minEnergy = 0.5 * minVelocity * minVelocity;
  var extension, kineticE, potentialE;
  const dPos = new Float64Array(3);

  return function( position, velocity, cursor3d, deltaTime, track ) {
    // Input cursor3d is a pointer to an object
    // Inputs position, velocity are pointers to 3-element arrays
    // Input deltaTime is a primitive floating point value

    var targetHeight = cursor3d.zoomTarget();

    // Save old altitude
    var oldAltitude = position[2];

    dPos[2] = position[2] - targetHeight;
    updateOscillator(position, velocity, dPos, w0, deltaTime, 2, 2);

    if (track) {
      // Adjust rotation to keep zoom location fixed on screen
      dPos.set(position);
      dragonflyStalk(dPos, cursor3d.zoomRay, cursor3d.zoomPosition, ellipsoid);
      // Restrict size of rotation in one time step
      vec3.subtract(dPos, dPos, position);
      var limited = limitRotation(dPos, maxRotation);
      vec3.add(position, position, dPos);
    }

    // Scale rotational velocity by the ratio of the height change
    var heightScale = position[2] / oldAltitude;
    velocity[0] *= heightScale;
    velocity[1] *= heightScale;

    if (cursor3d.isClicked() || limited) return;

    // Stop if we are already near steady state
    kineticE = 0.5 * velocity[2] ** 2;
    extension = position[2] - targetHeight;
    potentialE = 0.5 * (w0 * extension) ** 2;
    if (kineticE + potentialE < minEnergy * targetHeight) {
      targetHeight = position[2];
      velocity[2] = 0.0;
      cursor3d.stopZoom();
    }
    return;
  };
}

function limitRotation(dPos, maxRotation) {
  // Input dPos is a pointer to a 2-element array containing lon, lat changes
  // maxRotation is a primitive floating point value

  // Check for longitude value crossing antimeridian
  if (dPos[0] >  Math.PI) dPos[0] -= 2.0 * Math.PI;
  if (dPos[0] < -Math.PI) dPos[0] += 2.0 * Math.PI;

  if (Math.abs(dPos[0]) > maxRotation) {
    var tmp = Math.min(Math.max(-maxRotation, dPos[0]), maxRotation) / dPos[0];
    dPos[0] *= tmp;
    dPos[1] *= tmp;
    return true;
  }
  return false;
}

// Given a 3D scene coordinate over which a zoom action was initiated,
// and a distance between the screen and the center of the 3D scene,
// compute the rotations required to align the 3D coordinate along
// the original screen ray.  See
// https://en.wikipedia.org/wiki/Dragonfly#Motion_camouflage
// TODO: Clean this up. Just use difference of lat/lon under ray?
function dragonflyStalk(outRotation, ray, scenePos, ellipsoid) {
  // Output outRotation is a pointer to a vec3
  // Input ray is a pointer to a vec3
  // Input scenePos is a pointer to a 3D cursor object

  // Find the ray-sphere intersection in unrotated model space coordinates
  var target = new Float64Array(3);
  var unrotatedCamPos = [0.0, 0.0, outRotation[2] + vec3.length(scenePos)];
  var onEllipse = ellipsoid.shoot(target, unrotatedCamPos, ray);
  if (!onEllipse) return; // No intersection!

  // Find the rotation about the y-axis required to bring scene point into
  // the  x = target[0]  plane
  // First find distance of scene point from scene y-axis
  var sceneR = Math.sqrt( scenePos[0] ** 2 + scenePos[2] ** 2 );
  // If too short, exit rather than tipping poles out of y-z plane
  if ( sceneR < Math.abs(target[0]) ) return;
  var targetRotY = Math.asin( target[0] / sceneR );
  outRotation[0] =
    Math.atan2( scenePos[0], scenePos[2] ) - // Y-angle of scene vector
    // Math.asin( target[0] / sceneR );       // Y-angle of target point
    targetRotY;

  // We now know the x and y coordinates of the scene vector after rotation
  // around the y-axis: (x = target[0], y = scenePos[1])
  // Find the z-coordinate so we can compute the remaining required rotation
  var zRotated = sceneR * Math.cos(targetRotY);

  // Find the rotation about the screen x-axis required to bring the scene
  // point into the target y = target[1] plane
  // Assumes 0 angle is aligned along Z, and angle > 0 is rotation toward -y !
  outRotation[1] =
    Math.atan2( -1 * target[1], target[2] ) -  // X-angle of target point
    Math.atan2( -1 * scenePos[1], zRotated );  // X-angle of scene vector

  return;
}
