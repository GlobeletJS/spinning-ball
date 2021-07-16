import * as vec3 from "gl-matrix/vec3";

export function initEcefToLocalGeo() {
  const { cos, sin, hypot } = Math;
  const toENU = new Float64Array(9);

  return function ecefToDeltaLonLatAlt(delta, diff, anchor, viewPos) {
    // Inputs are pointers to vec3s.  WARNING: diff will be overwritten
    // diff represents a differential change (e.g. motion?) near anchor.
    // viewPos represents the position of the model coordinates (ECEF)
    //   relative to the view coordinates.
    // Output delta will be the corresponding differentials in lon/lat/alt

    // 0. Compute sines and cosines of longitude and latitude at anchor, which
    // is a surface normal on an ellipsoid (or an ECEF position on a sphere)
    const [x, y, z] = anchor;
    const p = hypot(x, z);
    const cosLon = (p > 0) ? z / p : 0.0;
    const sinLon = (p > 0) ? x / p : 0.0;
    const r = hypot(x, y, z);
    const cosLat = p / r;
    const sinLat = y / r;

    // 1. Transform to local East-North-Up coordinates at the anchor location
    setupENU(cosLon, sinLon, cosLat, sinLat);
    vec3.transformMat3(diff, diff, toENU);

    // 2. Convert horizontal component to changes in longitude, latitude
    delta[0] = diff[0] / r / (cosLat + 0.0001); // +0.0001 avoids /0
    delta[1] = diff[1] / r;
    delta[2] = diff[2];

    // 3. Latitudinal change is a rotation about an axis in the x-z plane, with
    // direction vec3.cross(anchor,North), or -East. We only want the component
    // rotating about the x-axis in view coordinates.
    delta[1] *= (cos(viewPos[0]) * cosLon + sin(viewPos[0]) * sinLon);
  };

  function setupENU(cosLon, sinLon, cosLat, sinLat) {
    // Build matrix to rotate from global Earth-Centered-Earth-Fixed
    // to local East-North-Up coordinates.
    // Follows Widnal & Peraire (MIT) p.7, with axes renamed:
    //   z -> y, y -> x, x -> z
    // Using OpenGL COLUMN-MAJOR format!!
    toENU[0] =  cosLon;
    toENU[1] = -sinLat * sinLon;
    toENU[2] =  cosLat * sinLon;

    toENU[3] =  0.0;
    toENU[4] =  cosLat;
    toENU[5] =  sinLat;

    toENU[6] = -sinLon;
    toENU[7] = -sinLat * cosLon;
    toENU[8] =  cosLat * cosLon;
    // Note: the rows of the matrix are the unit vectors along each axis:
    // Elements (0, 3, 6) = unit vector in East direction
    // Elements (1, 4, 7) = unit vector in North direction
    // Elements (2, 5, 8) = unit vector in Up direction
  }
}
