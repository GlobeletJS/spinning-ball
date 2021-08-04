import { initRayGun } from "./raygun.js";
import { initEcefToLocalGeo } from "./geodelta.js";

export function initEllipsoid() {
  const { atan2, sin, cos, sqrt, hypot } = Math;

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

  const { shoot, findHorizon } = initRayGun(M, meanRadius);

  return {
    meanRadius: () => meanRadius,
    ecef2geocentric,
    ecefToDeltaLonLatAlt: initEcefToLocalGeo(),
    geodetic2ecef,
    shoot,
    findHorizon,
  };

  function ecef2geocentric(gcPos, ecefPos) {
    // Input earth-centered earth-fixed coords are in WebGL axis definition
    const [x, y, z] = ecefPos;

    // Output coords are geocentric: valid for a SPHERE, not an ellipsoid
    gcPos[0] = atan2(x, z); // Longitude (radians)
    gcPos[1] = atan2(y, hypot(x, z)); // Latitude (radians)
    gcPos[2] = hypot(x, y, z) - meanRadius; // Altitude (kilometers)
  }

  function geodetic2ecef(ecef, geodetic) {
    // Input geodetic units: [radians, radians, kilometers]
    const [lon, lat, alt] = geodetic;

    // Start from prime vertical radius of curvature -- see
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    const sinLat = sin(lat);
    const primeVertRad = semiMajor / sqrt(1.0 - e2 * sinLat ** 2);
    // Radial distance from y-axis:
    const p = (primeVertRad + alt) * cos(lat);

    // Compute earth-centered earth-fixed (ECEF) coordinates
    ecef[0] = p * sin(lon);
    ecef[1] = (primeVertRad + alt) * sinLat * (1.0 - e2);
    ecef[2] = p * cos(lon);
  }
}
