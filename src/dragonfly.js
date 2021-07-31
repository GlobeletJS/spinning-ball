// TODO: Clean this up. Just use difference of lat/lon under ray?
export function dragonflyStalk(camPos, zoomPos, zoomRay, ellipsoid) {
  // See https://en.wikipedia.org/wiki/Dragonfly#Motion_camouflage
  // Input camPos is the current geodetic position of the camera
  // zoomPos is the ECEF position towards which we are zooming
  // zoomRay is the camera ray (corresponding to a screen pixel) which pointed
  //  toward zoomPos when the zoom action began
  // The lon, lat in camPos will be adjusted to re-align zoomPos along zoomRay
  //  (e.g., after a change in camera altitude)

  // Find the ray-sphere intersection in unrotated model space coordinates
  const centerDist = camPos[2] + ellipsoid.meanRadius();
  const [lon, lat] = getCamPos(centerDist, zoomPos, zoomRay, ellipsoid);

  const dLonLat = [lon - camPos[0], lat - camPos[1]];
  const limited = limitRotation(dLonLat);
  camPos[0] += dLonLat[0];
  camPos[1] += dLonLat[1];

  return limited;
}

export function getCamPos(centerDist, zoomPos, zoomRay, ellipsoid) {
  // Returns the [lon, lat] where a camera at centerDist km from the
  // ellipsoid center will have zoomPos aligned along zoomRay
  const { abs, hypot, asin, cos, atan2 } = Math;

  // Find the ray-sphere intersection in unrotated model space coordinates
  const unrotatedCamPos = [0.0, 0.0, centerDist];
  const target = new Float64Array(3);
  const intersected = ellipsoid.shoot(target, unrotatedCamPos, zoomRay);
  if (!intersected) return;

  // Find the rotation about the y-axis required to bring zoomPos into the
  // x = target[0] plane
  const [zoomX, zoomY, zoomZ] = zoomPos;
  const zoomR = hypot(zoomX, zoomZ);
  if (zoomR < abs(target[0])) return;
  const targetRotY = asin(target[0] / zoomR); // ?= atan2(target[0], target[2])
  const rotY = atan2(zoomX, zoomZ) - targetRotY;

  // After rotation around Y, zoomPos will be aligned with x = target[0].
  // Now find the rotation around X to align with y = target[1]
  const targetRotX = atan2(target[2], target[1]);
  const zRotated = zoomR * cos(targetRotY);
  const rotX = atan2(zRotated, zoomY) - targetRotX;

  // Note signs: rotX ~ -latitude
  return [rotY, -rotX];
}

export function limitRotation(dPos) {
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
