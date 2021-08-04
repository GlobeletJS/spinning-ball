import { initECEF } from "./ecef.js";
import * as vec3 from "gl-matrix/vec3";

export function initCamera(params) {
  const { view, ellipsoid, initialPosition } = params;
  const rayVec = new Float64Array(3);
  const ecefTmp = new Float64Array(3);

  // [longitude (radians), latitude (radians), altitude (kilometers)]
  const position = new Float64Array(initialPosition);

  const ecef = initECEF(ellipsoid, position);

  return {
    position: () => position.slice(),
    ecefPos: ecef.position, // WARNING: exposed to changes from outside!
    rotation: ecef.rotation,

    project,
    ecefToScreenRay,
    update,
  };

  function update(dPos) {
    if (dPos.every(c => c == 0.0)) return;
    position.set(position.map((c, i) => c + dPos[i]));
    ecef.update(position);
  }

  function project(xy, geodetic) {
    // Project a geodetic position on the ellipsoid (lon, lat, alt)
    //  to a position on the display (x, y in pixels)
    ellipsoid.geodetic2ecef(ecefTmp, geodetic);
    const visible = ecefToScreenRay(rayVec, ecefTmp);

    xy[0] = view.width() * (1 + rayVec[0] / view.rightEdge()) / 2;
    xy[1] = view.height() * (1 - rayVec[1] / view.topEdge()) / 2;
    return visible;
  }

  function ecefToScreenRay(screenRay, ecefPoint) {
    // Find the screenRay (from camera position) that intersects ecefPoint

    // Translate to camera position
    vec3.subtract(rayVec, ecefPoint, ecef.position);
    // rayVec now points from camera to ecef. The sign of the
    // dot product tells us whether it is beyond the horizon
    const visible = vec3.dot(rayVec, ecefPoint) < 0;

    // Rotate to camera orientation
    vec3.transformMat4(screenRay, rayVec, ecef.inverse);

    // Normalize to z = -1
    screenRay[0] /= -screenRay[2];
    screenRay[1] /= -screenRay[2];
    screenRay[2] = -1.0;

    return visible;
  }
}
