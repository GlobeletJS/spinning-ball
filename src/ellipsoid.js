import * as vec3 from "gl-matrix/vec3";
import { initEcefToLocalGeo } from "./geodelta";

export function initEllipsoid() {
  const { atan2, sin, cos, sqrt } = Math;

  // Store ellipsoid parameters
  const semiMajor = 6371.0;  // kilometers
  const semiMinor = 6371.0;  // kilometers
  const e2 = 1.0 - semiMinor ** 2 / semiMajor ** 2; // Ellipticity squared
  // https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
  const meanRadius = (2.0 * semiMajor + semiMinor) / 3.0;

  // M: matrix to scale ellipsoid to unit sphere. The ellipsoid is
  // aligned with the coordinate axes, so we can store only the diagonal
  const Mdiag = [semiMajor, semiMinor, semiMajor];
  const M = vec => vec.map((c, i) => c / Mdiag[i]);

  return {
    meanRadius: () => meanRadius,
    ecef2geocentric,
    ecefToDeltaLonLatAlt: initEcefToLocalGeo(),
    geodetic2ecef,
    shoot: shootEllipsoid,
    findHorizon,
  };

  function ecef2geocentric(gcPos, ecefPos) {
    // Output gcPos is a pointer to a 3-element array, containing geocentric
    //  longitude & latitude (radians) and altitude (meters) coordinates
    // Input ecefPos is a pointer to a 3-element array, containing earth-
    //  centered earth-fixed x,y,z coordinates in the WebGL axis definition

    // Note: order of calculations is chosen to allow calls with same array
    // as input & output (gcPos, ecefPos point to same array)

    // Compute squared distance from polar axis
    const p2 = ecefPos[0] ** 2 + ecefPos[2] ** 2;

    gcPos[0] = atan2(ecefPos[0], ecefPos[2]);     // Longitude
    gcPos[1] = atan2(ecefPos[1], sqrt(p2));  // Latitude

    // NOTE: this "altitude" is distance from SPHERE, not ellipsoid
    gcPos[2] = sqrt(p2 + ecefPos[1] ** 2) - meanRadius; // Altitude
    return;
  }

  function geodetic2ecef(ecef, geodetic) {
    // Output ecef is a pointer to a 3-element array containing X,Y,Z values
    //   of the point in earth-centered earth-fixed (ECEF) coordinates
    // Input geodetic is a pointer to a 3-element array, containing
    //   longitude & latitude (in radians) and altitude (in meters)

    // Start from prime vertical radius of curvature -- see
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    const sinLat = sin( geodetic[1] );
    const primeVertRad = semiMajor / sqrt( 1.0 - e2 * sinLat ** 2 );
    // Radial distance from y-axis:
    const p = (primeVertRad + geodetic[2]) * cos(geodetic[1]);

    // Compute ECEF position
    ecef[0] = p * sin(geodetic[0]);
    ecef[1] = (primeVertRad + geodetic[2]) * sinLat * (1.0 - e2);
    ecef[2] = p * cos(geodetic[0]);
    return;
  }

  function shootEllipsoid(intersection, camera, rayVec) {
    // Inputs camera, rayVec are pointers to vec3s indicating the
    //   position of the camera and the direction of a ray shot from the camera,
    //   both in earth-centered earth-fixed (ECEF) coordinates
    // Output intersection is a pointer to a vec3 in ECEF coordinates indicating
    //   the position of the intersection of the ray with the ellipsoid
    // Return value indicates whether the ray did in fact intersect the spheroid

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
    return intersected;
  }

  function findHorizon(horizon, camera, rayVec) {
    // Find the point on the horizon under rayvec.
    // We first adjust rayVec to point it toward the horizon, and then
    // re-shoot the ellipsoid with the corrected ray
    const dRay = new Float64Array(3);

    // 1. Find the component of rayVec parallel to camera direction
    vec3.normalize(dRay, camera); // Unit vector along camera direction
    const paraLength = vec3.dot(dRay, rayVec);
    vec3.scale( dRay, dRay, paraLength );

    // 2. Find the component perpendicular to camera direction
    vec3.subtract( dRay, rayVec, dRay );
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
