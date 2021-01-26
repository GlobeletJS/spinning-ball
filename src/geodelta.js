import * as vec3 from 'gl-matrix/vec3';

export function initEcefToLocalGeo() {
  var sinLon, cosLon, sinLat, cosLat;
  const toENU = new Float64Array(9);

  return ecefToDeltaLonLatAlt;

  function ecefToDeltaLonLatAlt( delta, diff, anchor, viewPos ) {
    // Inputs are pointers to vec3s. anchor is a position in ECEF coordinates.
    // diff represents a differential change (e.g. motion?) near anchor.
    // Output delta will be the corresponding differentials in lon/lat/alt
    // viewPos represents the position of the model coordinates (ECEF) relative
    // to the view coordinates.    WARNING: diff will be overwritten

    // 1. Transform to local East-North-Up coordinates at the anchor location
    setupENU( anchor );
    vec3.transformMat3( diff, diff, toENU );

    // 2. Convert horizontal component to changes in longitude, latitude
    let r = vec3.length(anchor);
    delta[0] = diff[0] / r / (cosLat + 0.0001); // +0.0001 avoids /0
    delta[1] = diff[1] / r;
    delta[2] = diff[2];
    
    // 3. Latitudinal change is a rotation about an axis in the x-z plane, with
    // direction vec3.cross(anchor,North), or -East. We only want the component
    // rotating about the x-axis in view coordinates.
    delta[1] *= (
        Math.cos(viewPos[0]) * cosLon +
        Math.sin(viewPos[0]) * sinLon 
        );
    return;
  }

  function setupENU( normal ) {
    // Setup the matrix to rotate from global Earth-Centered-Earth-Fixed
    // to local East-North-Up coordinates. Assumptions for input ECEF:
    //    y-axis is the polar axis
    //   +z-axis points toward latitude = longitude = 0.
    // Input normal is an ellipsoid surface normal at the desired ENU origin

    // Update sines and cosines of the latitude and longitude of the normal
    const p2 = normal[0]**2 + normal[2]**2;
    const p = Math.sqrt(p2);
    if (p > 0) {
      sinLon = normal[0] / p;
      cosLon = normal[2] / p;
    } else {
      sinLon = 0.0;
      cosLon = 0.0;
    }
    const r = Math.sqrt(p2 + normal[1]**2);
    sinLat = normal[1] / r;
    cosLat = p / r;

    // Build matrix. Follows Widnal & Peraire (MIT) p.7, with the axes renamed:
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
    return;
  }
}
