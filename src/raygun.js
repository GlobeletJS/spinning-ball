import * as vec3 from "gl-matrix/vec3";

export function initRayGun(M, meanRadius) {
  const { sqrt } = Math;

  return { shoot: shootEllipsoid, findHorizon };

  function shootEllipsoid(intersection, camera, rayVec) {
    // Inputs camera, rayVec are pointers to vec3s indicating the
    //   position of the camera and the direction of a ray shot from the camera,
    //   both in earth-centered earth-fixed (ECEF) coordinates
    // Output intersection is a pointer to a vec3 in ECEF coordinates indicating
    //   the position of the intersection of the ray with the ellipsoid

    // Math: solving for values t where || M (camera + t*rayVec) || = 1,
    const mCam = M(camera);
    const mRay = M(rayVec);

    // We now have <mRay,mRay>*t^2 + 2*<mRay,mCam>*t + <mCam,mCam> - 1 = 0
    const a = vec3.dot(mRay, mRay);
    const b = 2.0 * vec3.dot(mRay, mCam);
    const c = vec3.dot(mCam, mCam) - 1.0;
    const discriminant = b ** 2 - 4 * a * c;

    const intersected = (discriminant >= 0);

    // There are generally 2 intersections. We want the closer one, with
    // smallest positive t. (b < 0, if ray is back from camera to ellipsoid)
    // If no intersection, find the point on the ray that comes closest to the
    // unit sphere: minimize a*t^2 + b*t + c (get zero of derivative)
    // NOTE: NOT the closest point on the ellipsoid! And NOT on the horizon!
    const t = (intersected)
      ? (-b - sqrt(discriminant)) / (2.0 * a)
      : -0.5 * b / a;

    // NOTE: rayVec is actually a vec4
    vec3.scaleAndAdd(intersection, camera, rayVec, t);
    return intersected; // Indicates whether the ray did in fact hit
  }

  function findHorizon(horizon, camera, rayVec) {
    // Find the point on the horizon under rayvec.
    // We first adjust rayVec to point it toward the horizon, and then
    // re-shoot the ellipsoid with the corrected ray
    const dRay = new Float64Array(3);

    // 1. Find the component of rayVec parallel to camera direction
    vec3.normalize(dRay, camera); // Unit vector along camera direction
    const paraLength = vec3.dot(dRay, rayVec);
    vec3.scale(dRay, dRay, paraLength);

    // 2. Find the component perpendicular to camera direction
    vec3.subtract(dRay, rayVec, dRay);
    const perpLength = vec3.length(dRay);
    if (perpLength == 0) return false; // No solution if ray is vertical

    // 3. Find the error of the length of the perpendicular component
    const sinAlpha = meanRadius / vec3.length(camera); // sin(angle to horizon)
    const tanAlpha = sinAlpha / sqrt(1.0 - sinAlpha * sinAlpha);
    const dPerp = -paraLength * tanAlpha - perpLength;

    // 4. Find the corrected rayVec
    vec3.scaleAndAdd(dRay, rayVec, dRay, dPerp / perpLength);

    // 5. Re-shoot the ellipsoid with the corrected rayVec
    shootEllipsoid(horizon, camera, dRay);
    return true;
  }
}
