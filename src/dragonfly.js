export function getCamPos(centerDist, zoomPos, zoomRay, ellipsoid) {
  // See https://en.wikipedia.org/wiki/Dragonfly#Motion_camouflage
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
