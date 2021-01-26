import * as vec3 from 'gl-matrix/vec3';

export function initProjector(ellipsoid, camPosition, camInverse, screen) {
  const rayVec = new Float64Array(3);
  const ecefTmp = new Float64Array(3);

  return {
    ecefToScreenRay,
    lonLatToScreenXY,
  };

  function lonLatToScreenXY(xy, lonLat) {
    ellipsoid.geodetic2ecef(ecefTmp, lonLat);
    const visible = ecefToScreenRay(rayVec, ecefTmp); // Overwrites rayVec!

    xy[0] = screen.width() * ( 1 + rayVec[0] / screen.rightEdge() ) / 2;
    xy[1] = screen.height() * ( 1 - rayVec[1] / screen.topEdge() ) / 2;
    return visible;
  }

  function ecefToScreenRay(screenRay, ecefPosition) {
    // For a given point on the ellipsoid (in ECEF coordinates) find the
    // rayVec from a given camera position that will intersect it
    
    // Translate to camera position
    vec3.subtract(rayVec, ecefPosition, camPosition);
    // rayVec now points from camera to ecef. The sign of the
    // dot product tells us whether it is beyond the horizon
    const visible = ( vec3.dot(rayVec, ecefPosition) < 0 );

    // Rotate to camera orientation
    vec3.transformMat4(screenRay, rayVec, camInverse);

    // Normalize to z = -1
    screenRay[0] /= -screenRay[2];
    screenRay[1] /= -screenRay[2];
    screenRay[2] = -1.0;

    return visible;
  }
}
