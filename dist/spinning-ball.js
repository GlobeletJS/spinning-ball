function initView(porthole, fieldOfView) {
  // The porthole is an HTML element acting as a window into a 3D world
  // fieldOfView is the vertical view angle range in degrees (floating point)

  // Compute values for transformation between the 3D world and the 2D porthole
  let portRect, width, height, aspect;
  const tanFOV = Math.tan(fieldOfView * Math.PI / 180.0 / 2.0);
  const maxRay = [];

  computeRayParams(); // Set initial values

  return {
    element: porthole, // Back-reference
    changed: computeRayParams,

    width: () => width,
    height: () => height,
    topEdge: () => maxRay[1],   // tanFOV
    rightEdge: () => maxRay[0], // aspect * tanFOV
    maxRay, // TODO: is it good to expose local state?
    getRayParams,
  };

  function computeRayParams() {
    // Compute porthole size
    portRect = porthole.getBoundingClientRect();
    const newWidth = portRect.right - portRect.left;
    const newHeight = portRect.bottom - portRect.top;

    // Exit if no change
    if (width === newWidth && height === newHeight) return false;

    // Update stored values
    width = newWidth;
    height = newHeight;
    aspect = width / height;
    maxRay[0] = aspect * tanFOV;
    maxRay[1] = tanFOV; // Probably no change, but it is exposed externally

    // Let the calling program know that the porthole changed
    return true;
  }

  // Convert a position on the screen into tangents of the angles
  // (relative to screen normal) of a ray shooting off into the 3D space
  function getRayParams(rayVec, clientX, clientY) {
    // NOTE strange behavior of getBoundingClientRect()
    // rect.left and .top are equal to the coordinates given by clientX/Y
    // when the mouse is at the left top pixel in the box.
    // rect.right and .bottom are NOT equal to clientX/Y at the bottom
    // right pixel -- they are one more than the clientX/Y values.
    // Thus the number of pixels in the box is given by
    //    porthole.clientWidth = rect.right - rect.left  (NO +1 !!)
    const x = clientX - portRect.left;
    const y = portRect.bottom - clientY - 1; // Flip sign to make +y upward

    // Normalized distances from center of box. We normalize by pixel DISTANCE
    // rather than pixel count, to ensure we get -1 and +1 at the ends.
    // (Confirm by considering the 2x2 case)
    const xratio = 2 * x / (width - 1) - 1;
    const yratio = 2 * y / (height - 1) - 1;

    rayVec[0] = xratio * maxRay[0];
    rayVec[1] = yratio * maxRay[1];
    // rayVec[2] = -1.0;
    // rayVec[3] = 0.0;
    return;
  }
}

/**
 * Common utilities
 * @module glMatrix
 */
var ARRAY_TYPE = typeof Float32Array !== 'undefined' ? Float32Array : Array;
if (!Math.hypot) Math.hypot = function () {
  var y = 0,
      i = arguments.length;

  while (i--) {
    y += arguments[i] * arguments[i];
  }

  return Math.sqrt(y);
};

/**
 * 3 Dimensional Vector
 * @module vec3
 */

/**
 * Creates a new, empty vec3
 *
 * @returns {vec3} a new 3D vector
 */

function create$2() {
  var out = new ARRAY_TYPE(3);

  if (ARRAY_TYPE != Float32Array) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
  }

  return out;
}
/**
 * Calculates the length of a vec3
 *
 * @param {ReadonlyVec3} a vector to calculate length of
 * @returns {Number} length of a
 */

function length(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  return Math.hypot(x, y, z);
}
/**
 * Subtracts vector b from vector a
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a the first operand
 * @param {ReadonlyVec3} b the second operand
 * @returns {vec3} out
 */

function subtract(out, a, b) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
  return out;
}
/**
 * Scales a vec3 by a scalar number
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a the vector to scale
 * @param {Number} b amount to scale the vector by
 * @returns {vec3} out
 */

function scale(out, a, b) {
  out[0] = a[0] * b;
  out[1] = a[1] * b;
  out[2] = a[2] * b;
  return out;
}
/**
 * Adds two vec3's after scaling the second operand by a scalar value
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a the first operand
 * @param {ReadonlyVec3} b the second operand
 * @param {Number} scale the amount to scale b by before adding
 * @returns {vec3} out
 */

function scaleAndAdd(out, a, b, scale) {
  out[0] = a[0] + b[0] * scale;
  out[1] = a[1] + b[1] * scale;
  out[2] = a[2] + b[2] * scale;
  return out;
}
/**
 * Normalize a vec3
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a vector to normalize
 * @returns {vec3} out
 */

function normalize(out, a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  var len = x * x + y * y + z * z;

  if (len > 0) {
    //TODO: evaluate use of glm_invsqrt here?
    len = 1 / Math.sqrt(len);
  }

  out[0] = a[0] * len;
  out[1] = a[1] * len;
  out[2] = a[2] * len;
  return out;
}
/**
 * Calculates the dot product of two vec3's
 *
 * @param {ReadonlyVec3} a the first operand
 * @param {ReadonlyVec3} b the second operand
 * @returns {Number} dot product of a and b
 */

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
/**
 * Transforms the vec3 with a mat4.
 * 4th vector component is implicitly '1'
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a the vector to transform
 * @param {ReadonlyMat4} m matrix to transform with
 * @returns {vec3} out
 */

function transformMat4$1(out, a, m) {
  var x = a[0],
      y = a[1],
      z = a[2];
  var w = m[3] * x + m[7] * y + m[11] * z + m[15];
  w = w || 1.0;
  out[0] = (m[0] * x + m[4] * y + m[8] * z + m[12]) / w;
  out[1] = (m[1] * x + m[5] * y + m[9] * z + m[13]) / w;
  out[2] = (m[2] * x + m[6] * y + m[10] * z + m[14]) / w;
  return out;
}
/**
 * Transforms the vec3 with a mat3.
 *
 * @param {vec3} out the receiving vector
 * @param {ReadonlyVec3} a the vector to transform
 * @param {ReadonlyMat3} m the 3x3 matrix to transform with
 * @returns {vec3} out
 */

function transformMat3(out, a, m) {
  var x = a[0],
      y = a[1],
      z = a[2];
  out[0] = x * m[0] + y * m[3] + z * m[6];
  out[1] = x * m[1] + y * m[4] + z * m[7];
  out[2] = x * m[2] + y * m[5] + z * m[8];
  return out;
}
/**
 * Perform some operation over an array of vec3s.
 *
 * @param {Array} a the array of vectors to iterate over
 * @param {Number} stride Number of elements between the start of each vec3. If 0 assumes tightly packed
 * @param {Number} offset Number of elements to skip at the beginning of the array
 * @param {Number} count Number of vec3s to iterate over. If 0 iterates over entire array
 * @param {Function} fn Function to call for each vector in the array
 * @param {Object} [arg] additional argument to pass to fn
 * @returns {Array} a
 * @function
 */

(function () {
  var vec = create$2();
  return function (a, stride, offset, count, fn, arg) {
    var i, l;

    if (!stride) {
      stride = 3;
    }

    if (!offset) {
      offset = 0;
    }

    if (count) {
      l = Math.min(count * stride + offset, a.length);
    } else {
      l = a.length;
    }

    for (i = offset; i < l; i += stride) {
      vec[0] = a[i];
      vec[1] = a[i + 1];
      vec[2] = a[i + 2];
      fn(vec, vec, arg);
      a[i] = vec[0];
      a[i + 1] = vec[1];
      a[i + 2] = vec[2];
    }

    return a;
  };
})();

function initRayGun(M, meanRadius) {
  // All method arguments are vec3s in ECEF coordinates
  return { shoot: shootEllipsoid, findHorizon };

  function shootEllipsoid(intersection, camera, rayVec) {
    // Input rayVec: direction of a ray shot from camera
    // Outputs position of the intersection of the ray with the ellipsoid

    // Math: solving for values t where || M (camera + t*rayVec) || = 1,
    const mCam = M(camera);
    const mRay = M(rayVec);

    // We now have <mRay,mRay>*t^2 + 2*<mRay,mCam>*t + <mCam,mCam> - 1 = 0
    const a = dot(mRay, mRay);
    const b = 2.0 * dot(mRay, mCam);
    const c = dot(mCam, mCam) - 1.0;
    const discriminant = b ** 2 - 4 * a * c;

    const intersected = (discriminant >= 0);

    // There are generally 2 intersections. We want the closer one, with
    // smallest positive t. (b < 0, if ray is back from camera to ellipsoid)
    // If no intersection, find the point on the ray that comes closest to the
    // unit sphere: minimize a*t^2 + b*t + c (get zero of derivative)
    // NOTE: NOT the closest point on the ellipsoid! And NOT on the horizon!
    const t = (intersected)
      ? (-b - Math.sqrt(discriminant)) / (2.0 * a)
      : -0.5 * b / a;

    // NOTE: rayVec is actually a vec4
    scaleAndAdd(intersection, camera, rayVec, t);
    return intersected; // Indicates whether the ray did in fact hit
  }

  function findHorizon(horizon, camera, rayVec) {
    // Find the point on the horizon under rayvec.
    // We first adjust rayVec to point it toward the horizon, and then
    // re-shoot the ellipsoid with the corrected ray
    const dRay = new Float64Array(3);

    // 1. Find the component of rayVec parallel to camera direction
    normalize(dRay, camera); // Unit vector along camera direction
    const paraLength = dot(dRay, rayVec);
    scale(dRay, dRay, paraLength);

    // 2. Find the component perpendicular to camera direction
    subtract(dRay, rayVec, dRay);
    const perpLength = length(dRay);
    if (perpLength == 0) return false; // No solution if ray is vertical

    // 3. Find the error of the length of the perpendicular component
    const sinAlpha = meanRadius / length(camera); // sin(angle to horizon)
    const tanAlpha = sinAlpha / Math.sqrt(1.0 - sinAlpha * sinAlpha);
    const dPerp = -paraLength * tanAlpha - perpLength;

    // 4. Find the corrected rayVec
    scaleAndAdd(dRay, rayVec, dRay, dPerp / perpLength);

    // 5. Re-shoot the ellipsoid with the corrected rayVec
    shootEllipsoid(horizon, camera, dRay);
    return true;
  }
}

function initEcefToLocalGeo() {
  const { cos, sin, hypot } = Math;
  const toENU = new Float64Array(9);

  return function ecefToDeltaLonLatAlt(delta, diff, anchor, viewPos) {
    // Inputs are in ECEF coords. viewPos represents the position of the model
    //   coordinates relative to the view coordinates.
    // Input diff represents a differential change (e.g. motion?) near anchor
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
    transformMat3(delta, diff, toENU);

    // 2. Convert horizontal component to changes in longitude, latitude
    delta[0] = delta[0] / r / (cosLat + 0.0001); // +0.0001 avoids /0
    delta[1] = delta[1] / r;

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

function initEllipsoid() {
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

function getUnitConversion(units) {
  // Internally, spinning-ball assumes geodetic coordinates in these units:
  //   [longitude (radians), latitude (radians), altitude (kilometers)]
  // Externally, the user may want longitude and latitude in degrees.
  // Construct the functions that convert user inputs to internal coordinates,
  // and invert internal coordinates to the user's units
  const maxLon = (units === "degrees") ? 180.0 : Math.PI;
  const maxLat = maxLon / 2;
  const uPerRad = maxLon / Math.PI;

  return {
    maxLon, maxLat,
    convert: (c) => new Float64Array([c[0] / uPerRad, c[1] / uPerRad, c[2]]),
    invert: (c) => new Float64Array([c[0] * uPerRad, c[1] * uPerRad, c[2]]),
  };
}

function checkCoords(p, n) {
  const isArray = Array.isArray(p) ||
    (ArrayBuffer.isView(p) && !(p instanceof DataView));
  return isArray && p.length >= n &&
    p.slice(0, n).every(Number.isFinite);
}

function wrapLongitude(lon) {
  const { floor, PI } = Math;
  const period = floor((lon + PI) / (2 * PI));
  return lon - period * 2 * PI;
}

function initBounds([minLon, minLat, minAlt], [maxLon, maxLat, maxAlt]) {
  const { min, max, PI } = Math;

  const hWidth = (minLon < maxLon)
    ? (maxLon - minLon) / 2
    : (maxLon - minLon) / 2 + PI;

  const centerLon = minLon + hWidth;

  return { check, apply };

  function check([lon, lat, alt]) {
    const dLon = wrapLongitude(lon - centerLon);
    if (dLon < -hWidth || hWidth < dLon) return false;
    if (lat < minLat || maxLat < lat) return false;
    if (alt < minAlt || maxAlt < alt) return false;
    return true;
  }

  function apply([lon, lat, alt]) {
    const dLon = wrapLongitude(lon - centerLon);
    const limdlon = min(max(-hWidth, dLon), hWidth);
    const clipLon = wrapLongitude(centerLon + limdlon);

    const clipLat = min(max(minLat, lat), maxLat);
    const clipAlt = min(max(minAlt, alt), maxAlt);

    return [clipLon, clipLat, clipAlt];
  }
}

function setParams(params) {
  // TODO: Get user-supplied semiMinor & semiMajor axes?
  const ellipsoid = initEllipsoid();

  const { units: userUnits = "degrees" } = params;
  if (!["degrees", "radians"].includes(userUnits)) {
    fail("units must be either degrees or radians");
  }
  const units = getUnitConversion(userUnits);

  const {
    display,
    position = [0.0, 0.0, ellipsoid.meanRadius * 4.0],
    minAltitude: minAlt = ellipsoid.meanRadius() * 0.00001,
    maxAltitude: maxAlt = ellipsoid.meanRadius() * 8.0,
    minLongitude: minLon = -units.maxLon,
    maxLongitude: maxLon = units.maxLon,
    minLatitude: minLat = -units.maxLat,
    maxLatitude: maxLat = units.maxLat,
  } = params;

  if (!(display instanceof Element)) fail("missing display element");

  check(minAlt, 0, ellipsoid.meanRadius() * 100000.0, "minAltitude");
  check(maxAlt, 0, ellipsoid.meanRadius() * 100000.0, "maxAltitude");
  if (minAlt > maxAlt) fail("minAltitude must be <= maxAltitude");

  check(minLon, -units.maxLon, units.maxLon, "minLongitude");
  check(maxLon, -units.maxLon, units.maxLon, "maxLongitude");
  check(minLat, -units.maxLat, units.maxLat, "minLatitude");
  check(maxLat, -units.maxLat, units.maxLat, "maxLatitude");
  if (minLat > maxLat) fail("minLatitude must be <= maxLatitude");

  const b1 = units.convert([minLon, minLat, minAlt]);
  const b2 = units.convert([maxLon, maxLat, maxAlt]);
  const bounds = initBounds(b1, b2);

  if (!checkCoords(position, 3)) fail("position must be an Array of 3 numbers");
  const initialPosition = units.convert(position);
  if (!bounds.check(initialPosition)) fail ("initial position out of range");

  return {
    ellipsoid, display, units, initialPosition, bounds, minAlt, maxAlt,
    view: initView(display, 25.0), // Computes ray params at point on display
  };
}

function check(c, lb, ub, name) {
  if (Number.isFinite(c) && (lb <= c) && (c <= ub)) return true;
  fail(name + " must be a Number between " + lb + " and " + ub);
}

function fail(message) {
  // TODO: Should some errors be RangeErrors or TypeErrors instead?
  throw Error("spinning-ball parameters check: " + message);
}

/**
 * 4x4 Matrix<br>Format: column-major, when typed out it looks like row-major<br>The matrices are being post multiplied.
 * @module mat4
 */

/**
 * Creates a new identity mat4
 *
 * @returns {mat4} a new 4x4 matrix
 */

function create$1() {
  var out = new ARRAY_TYPE(16);

  if (ARRAY_TYPE != Float32Array) {
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
    out[9] = 0;
    out[11] = 0;
    out[12] = 0;
    out[13] = 0;
    out[14] = 0;
  }

  out[0] = 1;
  out[5] = 1;
  out[10] = 1;
  out[15] = 1;
  return out;
}
/**
 * Transpose the values of a mat4
 *
 * @param {mat4} out the receiving matrix
 * @param {ReadonlyMat4} a the source matrix
 * @returns {mat4} out
 */

function transpose(out, a) {
  // If we are transposing ourselves we can skip a few steps but have to cache some values
  if (out === a) {
    var a01 = a[1],
        a02 = a[2],
        a03 = a[3];
    var a12 = a[6],
        a13 = a[7];
    var a23 = a[11];
    out[1] = a[4];
    out[2] = a[8];
    out[3] = a[12];
    out[4] = a01;
    out[6] = a[9];
    out[7] = a[13];
    out[8] = a02;
    out[9] = a12;
    out[11] = a[14];
    out[12] = a03;
    out[13] = a13;
    out[14] = a23;
  } else {
    out[0] = a[0];
    out[1] = a[4];
    out[2] = a[8];
    out[3] = a[12];
    out[4] = a[1];
    out[5] = a[5];
    out[6] = a[9];
    out[7] = a[13];
    out[8] = a[2];
    out[9] = a[6];
    out[10] = a[10];
    out[11] = a[14];
    out[12] = a[3];
    out[13] = a[7];
    out[14] = a[11];
    out[15] = a[15];
  }

  return out;
}
/**
 * Rotates a matrix by the given angle around the X axis
 *
 * @param {mat4} out the receiving matrix
 * @param {ReadonlyMat4} a the matrix to rotate
 * @param {Number} rad the angle to rotate the matrix by
 * @returns {mat4} out
 */

function rotateX(out, a, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad);
  var a10 = a[4];
  var a11 = a[5];
  var a12 = a[6];
  var a13 = a[7];
  var a20 = a[8];
  var a21 = a[9];
  var a22 = a[10];
  var a23 = a[11];

  if (a !== out) {
    // If the source and destination differ, copy the unchanged rows
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
    out[12] = a[12];
    out[13] = a[13];
    out[14] = a[14];
    out[15] = a[15];
  } // Perform axis-specific matrix multiplication


  out[4] = a10 * c + a20 * s;
  out[5] = a11 * c + a21 * s;
  out[6] = a12 * c + a22 * s;
  out[7] = a13 * c + a23 * s;
  out[8] = a20 * c - a10 * s;
  out[9] = a21 * c - a11 * s;
  out[10] = a22 * c - a12 * s;
  out[11] = a23 * c - a13 * s;
  return out;
}
/**
 * Creates a matrix from the given angle around the Y axis
 * This is equivalent to (but much faster than):
 *
 *     mat4.identity(dest);
 *     mat4.rotateY(dest, dest, rad);
 *
 * @param {mat4} out mat4 receiving operation result
 * @param {Number} rad the angle to rotate the matrix by
 * @returns {mat4} out
 */

function fromYRotation(out, rad) {
  var s = Math.sin(rad);
  var c = Math.cos(rad); // Perform axis-specific matrix multiplication

  out[0] = c;
  out[1] = 0;
  out[2] = -s;
  out[3] = 0;
  out[4] = 0;
  out[5] = 1;
  out[6] = 0;
  out[7] = 0;
  out[8] = s;
  out[9] = 0;
  out[10] = c;
  out[11] = 0;
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}

function initECEF(ellipsoid, initialPos) {
  // From the geodetic position, we derive Earth-Centered Earth-Fixed (ECEF)
  // coordinates and a rotation matrix
  // These are suitable for rendering Relative To Eye (RTE), as described in
  // P Cozzi, 3D Engine Design for Virtual Globes, www.virtualglobebook.com
  const position = new Float64Array([0.0, 0.0, 0.0, 1.0]);
  const rotation = create$1();  // Note: single precision!! (Float32Array)
  const inverse  = create$1();

  update(initialPos); // Set initial values

  return {
    position, // WARNING: Exposes local array to changes from outside
    rotation,
    inverse,
    update,
  };

  function update(geodetic) {
    // Compute ECEF coordinates. NOTE WebGL coordinate convention:
    // +x to right, +y to top of screen, and +z into the screen
    ellipsoid.geodetic2ecef(position, geodetic);

    // Rotation: y first, so it will be left of x operator in final matrix
    // (gl-matrix library 'post-multplies' by each new matrix)
    // Positive angles about Y are towards the +X axis, or East longitude.
    fromYRotation(rotation, geodetic[0]);
    // Positive angles about X are towards the -Y axis!
    // (from Y to Z, and Z to -Y). But geodetic[1] is a latitude, toward N
    rotateX(rotation, rotation, -geodetic[1]);

    // The inverse of a rotation matrix is its transpose
    transpose(inverse, rotation);
  }
}

function initCamera(params) {
  const { view, ellipsoid, bounds, initialPosition } = params;
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
    const newPos = position.map((c, i) => c + dPos[i]);
    position.set(bounds.apply(newPos));
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
    subtract(rayVec, ecefPoint, ecef.position);
    // rayVec now points from camera to ecef. The sign of the
    // dot product tells us whether it is beyond the horizon
    const visible = dot(rayVec, ecefPoint) < 0;

    // Rotate to camera orientation
    transformMat4$1(screenRay, rayVec, ecef.inverse);

    // Normalize to z = -1
    screenRay[0] /= -screenRay[2];
    screenRay[1] /= -screenRay[2];
    screenRay[2] = -1.0;

    return visible;
  }
}

function initCursor() {
  // What does an animation need to know about the cursor at each frame?
  // First, whether the user did any of the following since the last frame:
  //  - Started new actions
  let touchStarted = false; // Touched or clicked the element
  let zoomStarted  = false; // Rotated mousewheel, or started two-finger touch
  //  - Changed something
  let moved  = false;       // Moved mouse or touch point
  let zoomed = false;       // Rotated mousewheel, or adjusted two-finger touch
  //  - Is potentially in the middle of something
  let tapping = false;      // No touchEnd, and no cursor motion
  //  - Ended actions
  let touchEnded = false;   // mouseup or touchend/cancel/leave
  let tapped = false;       // Completed a click or tap action

  // We also need to know the current cursor position and zoom scale
  let cursorX = 0;
  let cursorY = 0;
  let zscale = 1.0;

  // For tap/click reporting, we need to remember where the touch started
  let startX = 0;
  let startY = 0;
  // What is a click/tap and what is a drag? If the cursor moved more than
  // this threshold between touchStart and touchEnd, it is a drag
  const threshold = 6;

  return {
    // Methods to report local state. Return a copy to protect local values
    touchStarted: () => touchStarted,
    zoomStarted: () => zoomStarted,
    moved: () => moved,
    zoomed: () => zoomed,
    tapped: () => tapped,
    touchEnded: () => touchEnded,
    hasChanged: () => (moved || zoomed || tapped),
    zscale: () => zscale,
    x: () => cursorX,
    y: () => cursorY,

    // Methods to update local state
    startTouch,
    startZoom,
    move,
    zoom,
    endTouch,
    reset,
  };

  function startTouch(evnt) {
    cursorX = evnt.clientX;
    cursorY = evnt.clientY;
    touchStarted = true;
    startX = cursorX;
    startY = cursorY;
    tapping = true;
  }

  function startZoom(evnt) {
    // Store the cursor position
    cursorX = evnt.clientX;
    cursorY = evnt.clientY;
    zoomStarted = true;
    tapping = false;
  }

  function move(evnt) {
    cursorX = evnt.clientX;
    cursorY = evnt.clientY;
    moved = true;
    const dist = Math.hypot(cursorX - startX, cursorY - startY);
    if (dist > threshold) tapping = false;
  }

  function zoom(scale) {
    zscale *= scale;
    zoomed = true;
    tapping = false;
  }

  function endTouch() {
    if (touchStarted) {
      // Ending a new touch? Just ignore both // TODO: is this a good idea?
      touchStarted = false;
      touchEnded = false;
    } else {
      touchEnded = true;
    }
    tapped = tapping;
    tapping = false;
  }

  function reset() {
    touchStarted = false;
    zoomStarted  = false;
    moved  = false;
    zoomed = false;
    touchEnded = false;
    // NOTE: we do NOT reset tapping... this could carry over to next check
    tapped = false;
    zscale = 1.0;
  }
}

function getMidPoint(p0, p1) {
  // Convert a two-touch event to a single event at the midpoint
  const dx = p1.clientX - p0.clientX;
  const dy = p1.clientY - p0.clientY;
  return {
    clientX: p0.clientX + dx / 2,
    clientY: p0.clientY + dy / 2,
    distance: Math.hypot(dx, dy),
  };
}

function initWheelScale(wheelDelta) {
  return (wheelDelta === "constant") ? wheelScale_const : wheelScale;
}

function wheelScale(turn) {
  const { deltaY, deltaMode } = turn;
  if (!deltaY) return 1.0; // Could be a deltaX or deltaZ event

  switch (deltaMode) {
    case 0:
      // Chrome on Windows 10 Surface Book 2: deltaY = -100 * devicePixelRatio
      return 1.0 + deltaY * 0.002 / window.devicePixelRatio;
    case 1:
      // Firefox on Windows 10 Surface Book 2: deltaY = -3
      return 1.0 + deltaY * 0.067;
    case 2:
      // Untested. Ratio vs. case 0 is from d3-zoom
      return 1.0 + deltaY;
  }
}

function wheelScale_const(turn) {
  // Ignore dY from the browser - may be arbitrarily scaled. Keep only the sign
  return 1.0 + 0.2 * Math.sign(turn.deltaY);
}

function initTouch(div, { wheelDelta = "default" } = {}) {
  // Add event listeners to update the state of a cursor object
  // Input div is an HTML element on which events will be registered
  const cursor = initCursor();
  const getWheelScale = initWheelScale(wheelDelta);

  // Remember the distance between two pointers
  let lastDistance = 1.0;

  div.addEventListener("dragstart", d => d.preventDefault(), false);

  // Add mouse events
  div.addEventListener("mousedown",   cursor.startTouch, false);
  div.addEventListener("mousemove",   cursor.move,       false);
  div.addEventListener("mouseup",     cursor.endTouch,   false);
  div.addEventListener("mouseleave",  cursor.endTouch,   false);
  div.addEventListener("wheel",       wheelZoom,         false);

  // Add touch events
  div.addEventListener("touchstart",  initTouch,       false);
  div.addEventListener("touchmove",   moveTouch,       false);
  div.addEventListener("touchend",    cursor.endTouch, false);
  div.addEventListener("touchcancel", cursor.endTouch, false);

  return cursor;

  function initTouch(evnt) {
    const { touches } = evnt;
    evnt.preventDefault();
    switch (touches.length) {
      case 1:
        cursor.startTouch(touches[0]);
        break;
      case 2: {
        const midpoint = getMidPoint(touches[0], touches[1]);
        cursor.startTouch(midpoint);
        cursor.startZoom(midpoint);
        // Initialize the starting distance between touches
        lastDistance = midpoint.distance;
        break;
      }
      default:
        cursor.endTouch(evnt);
    }
  }

  function moveTouch(evnt) {
    const { touches } = evnt;
    evnt.preventDefault();
    // NOTE: MDN says to add the touchmove handler within the touchstart handler
    // https://developer.mozilla.org/en-US/docs/Web/API/Touch_events/Using_Touch_Events
    switch (touches.length) {
      case 1:
        cursor.move(touches[0]);
        break;
      case 2: {
        const midpoint = getMidPoint(touches[0], touches[1]);
        // Move the cursor to the midpoint
        cursor.move(midpoint);
        // Zoom based on the change in distance between the two touches
        cursor.zoom(lastDistance / midpoint.distance);
        // Remember the new touch distance
        lastDistance = midpoint.distance;
        break;
      }
      default:
        return false;
    }
  }

  function wheelZoom(turn) {
    turn.preventDefault();
    cursor.startZoom(turn);
    cursor.zoom(getWheelScale(turn));
  }
}

/**
 * 4 Dimensional Vector
 * @module vec4
 */

/**
 * Creates a new, empty vec4
 *
 * @returns {vec4} a new 4D vector
 */

function create() {
  var out = new ARRAY_TYPE(4);

  if (ARRAY_TYPE != Float32Array) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
  }

  return out;
}
/**
 * Transforms the vec4 with a mat4.
 *
 * @param {vec4} out the receiving vector
 * @param {ReadonlyVec4} a the vector to transform
 * @param {ReadonlyMat4} m matrix to transform with
 * @returns {vec4} out
 */

function transformMat4(out, a, m) {
  var x = a[0],
      y = a[1],
      z = a[2],
      w = a[3];
  out[0] = m[0] * x + m[4] * y + m[8] * z + m[12] * w;
  out[1] = m[1] * x + m[5] * y + m[9] * z + m[13] * w;
  out[2] = m[2] * x + m[6] * y + m[10] * z + m[14] * w;
  out[3] = m[3] * x + m[7] * y + m[11] * z + m[15] * w;
  return out;
}
/**
 * Perform some operation over an array of vec4s.
 *
 * @param {Array} a the array of vectors to iterate over
 * @param {Number} stride Number of elements between the start of each vec4. If 0 assumes tightly packed
 * @param {Number} offset Number of elements to skip at the beginning of the array
 * @param {Number} count Number of vec4s to iterate over. If 0 iterates over entire array
 * @param {Function} fn Function to call for each vector in the array
 * @param {Object} [arg] additional argument to pass to fn
 * @returns {Array} a
 * @function
 */

(function () {
  var vec = create();
  return function (a, stride, offset, count, fn, arg) {
    var i, l;

    if (!stride) {
      stride = 4;
    }

    if (!offset) {
      offset = 0;
    }

    if (count) {
      l = Math.min(count * stride + offset, a.length);
    } else {
      l = a.length;
    }

    for (i = offset; i < l; i += stride) {
      vec[0] = a[i];
      vec[1] = a[i + 1];
      vec[2] = a[i + 2];
      vec[3] = a[i + 3];
      fn(vec, vec, arg);
      a[i] = vec[0];
      a[i + 1] = vec[1];
      a[i + 2] = vec[2];
      a[i + 3] = vec[3];
    }

    return a;
  };
})();

function initCursor2d(params, camera) {
  const { display, view, ellipsoid } = params;

  const cursor2d = initTouch(display);

  const screenRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  const ecefRay = new Float64Array(4);
  const cursorLonLat = new Float64Array(3);

  function project(cursorPosition) {
    // Project a cursor position on the display (x, y in pixels)
    //  to an ECEF coordinate on the ellipsoid

    view.getRayParams(screenRay, cursor2d.x(), cursor2d.y());
    transformMat4(ecefRay, screenRay, camera.rotation);
    // NOTE: cursorPosition will be overwritten!
    const onScene = ellipsoid.shoot(cursorPosition, camera.ecefPos, ecefRay);

    // Convert to longitude/latitude/altitude
    if (onScene) ellipsoid.ecef2geocentric(cursorLonLat, cursorPosition);

    return onScene;
  }

  return Object.assign(cursor2d, { project, screenRay, cursorLonLat });
}

function initCursor3d(params, camera) {
  const { initialPosition, minAlt, maxAlt } = params;

  const cursor2d = initCursor2d(params, camera);

  // Cursor positions are computed & stored in ECEF coordinates (x,y,z)
  const cursorPosition = new Float64Array(3);
  const clickPosition = new Float64Array(3);
  const zoomPosition = new Float64Array(3);
  // Track target screen ray and altitude for zooming
  const zoomRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  let targetAlt = initialPosition[2];

  // Flags about the cursor state
  let onScene = false;
  let clicked = false;
  let zooming = false;
  let wasTapped = false;
  let zoomFix = false; // Whether to fix the screen position of the zoom

  return {
    // POINTERs to local arrays. WARNING: local vals can be changed outside!
    cursorLonLat: cursor2d.cursorLonLat,
    cursorPosition,
    clickPosition,
    zoomPosition,
    zoomRay,

    // Methods to report local state
    hasChanged: () => cursor2d.hasChanged() || wasTapped,
    isOnScene: () => onScene,
    isClicked: () => clicked,
    wasTapped: () => wasTapped,
    isZooming: () => zooming,
    zoomFixed: () => zoomFix,
    zoomTarget: () => targetAlt,

    // Functions to update local state
    update,
    stopZoom,
  };

  function update(position, dynamics) {
    onScene = cursor2d.project(cursorPosition);
    if (!onScene) {
      clicked = zoomFix = false;
      return cursor2d.reset();
    }

    if (cursor2d.touchEnded()) clicked = zoomFix = false;
    wasTapped = cursor2d.tapped();

    if (cursor2d.touchStarted()) {
      clicked = true;
      clickPosition.set(cursorPosition);
      // Assuming this is a click or single touch, stop zooming
      stopZoom(position[2]);
      dynamics.stopZoom(); // Stops coasting in altitude direction
      // If this was actually a two-touch zoom, then cursor2d.zoomStarted()...
    }

    if (cursor2d.zoomStarted()) {
      zooming = zoomFix = true;
      zoomPosition.set(cursorPosition);
      zoomRay.set(cursor2d.screenRay);
      if (!clicked) dynamics.stopCoast();
    }

    if (cursor2d.zoomed()) {
      zooming = true;
      targetAlt *= cursor2d.zscale();
      targetAlt = Math.min(Math.max(minAlt, targetAlt), maxAlt);
    }

    cursor2d.reset();
  }

  function stopZoom(alt) {
    zooming = zoomFix = false;
    if (alt !== undefined) targetAlt = alt;
  }
}

function interpolateZoom(p0, p1) {
  const [ux0, uy0, w0] = p0;
  const [ux1, uy1, w1] = p1;

  const { cosh, sinh, tanh, exp, log, hypot, SQRT2, abs } = Math;
  const rho = SQRT2;
  const epsilon = 1e-6;

  const dx = ux1 - ux0;
  const dy = uy1 - uy0;
  const du = hypot(dx, dy);

  const rho2du = rho * rho * du;
  const uScale = w0 / rho2du;

  const b0 = (w1 * w1 - w0 * w0 + rho2du * rho2du) / (2 * w0 * rho2du);
  const b1 = (w1 * w1 - w0 * w0 - rho2du * rho2du) / (2 * w1 * rho2du);
  const r0 = log(hypot(b0, 1) - b0);
  const r1 = log(hypot(b1, 1) - b1);
  const coshr0 = cosh(r0);
  const sinhr0 = sinh(r0);

  const rhoS = (du < epsilon) ? log(w1 / w0) : (r1 - r0);

  function special(t) { // Special case for u0 =~ u1
    return [
      ux0 + t * dx,
      uy0 + t * dy,
      w0 * exp(rhoS * t),
    ];
  }

  function general(t) { // General case
    const uarg = rhoS * t + r0;
    const u = uScale * (coshr0 * tanh(uarg) - sinhr0);
    return [
      ux0 + u * dx,
      uy0 + u * dy,
      w0 * coshr0 / cosh(uarg)
    ];
  }

  const interp = (du < epsilon) ? special : general;

  return Object.assign(interp, { duration: abs(rhoS) / rho });
}

function initFlights(params, camera) {
  const { ellipsoid, bounds } = params;

  const wScale = 2.0 / (ellipsoid.meanRadius() * Math.PI);
  let t, interp, active = false;

  function flyTo(position) {
    if (!bounds.check(position)) {
      return console.log("spinningBall.flyTo: position out of bounds");
    }

    const [lon0, lat0, alt0] = camera.position();
    const [lon1, lat1, alt1] = position;
    // Scale altitude to be on the same order as lon, lat, and wrap longitude
    const p0 = [lon0, lat0, alt0 * wScale];
    const p1 = [lon0 + wrapLongitude(lon1 - lon0), lat1, alt1 * wScale];

    interp = interpolateZoom(p0, p1);
    t = 0.0;
    active = true;
  }

  function update(position, velocity, dt) {
    if (!active) return;
    t = Math.min(1.0, t + dt / interp.duration);
    const newPos = interp(t);
    newPos[2] /= wScale; // Revert scaling applied in flyTo

    if (t == 1.0) active = false;
    return position.map((c, i) => newPos[i] - c);
  }

  return {
    flyTo, update,
    active: () => active,
    cancel: () => (active = false),
  };
}

function oscillatorChange(x, v, t, w0) {
  // For a critically damped oscillator with natural frequency w0, find
  // the change in position x and velocity v over timestep t.  See
  // https://mathworld.wolfram.com/CriticallyDampedSimpleHarmonicMotion.html
  const expTerm = Math.exp(-w0 * t);
  const Bt = (v + w0 * x) * t;
  const dx = (x + Bt) * expTerm - x;
  const dv = (v - w0 * Bt) * expTerm - v;
  return [dx, dv];
}

function getCamPos(centerDist, zoomPos, zoomRay, ellipsoid) {
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

function limitRotation(dPos) {
  const { abs, min, max } = Math;
  const maxRotation = 0.15;

  dPos[0] = wrapLongitude(dPos[0]);

  if (abs(dPos[0]) < maxRotation) return false;

  const tmp = min(max(-maxRotation, dPos[0]), maxRotation) / dPos[0];
  dPos[0] *= tmp;
  dPos[1] *= tmp;
  return true;
}

function initZoom(ellipsoid, cursor3d) {
  // Update camera altitude based on target set by mouse wheel events
  //  or two-finger pinch movements
  const { zoomTarget, zoomPosition, zoomRay, stopZoom } = cursor3d;

  const w0 = 14.14; // Natural frequency of oscillator
  const minVelocity = 0.001;
  const minEnergy = 0.5 * minVelocity ** 2; // ASSUME mass == 1

  return function(position, velocity, dt) {
    const stretch = position[2] - zoomTarget();
    const [dz, dVz] = oscillatorChange(stretch, velocity[2], dt, w0);
    velocity[2] += dVz;

    // Scale rotational velocity by the ratio of the altitude change
    const altScale = 1.0 + dz / position[2];
    velocity[0] *= altScale;
    velocity[1] *= altScale;

    const dPos = new Float64Array([0.0, 0.0, dz]);
    const centerDist = position[2] + dz + ellipsoid.meanRadius();
    const newRotation = (cursor3d.zoomFixed())
      ? getCamPos(centerDist, zoomPosition, zoomRay, ellipsoid)
      : null;
    if (newRotation) {
      dPos[0] = newRotation[0] - position[0];
      dPos[1] = newRotation[1] - position[1];
    }
    const limited = limitRotation(dPos);

    const rotating = cursor3d.isClicked() || limited;
    const energy = 0.5 * velocity[2] ** 2 + // Kinetic
      0.5 * (w0 * (stretch + dz)) ** 2;     // Potential
    // Stop if we are already near steady state
    if (!rotating && energy < minEnergy * zoomTarget()) {
      velocity[2] = 0.0;
      stopZoom();
    }

    return dPos;
  };
}

function initRotation(ellipsoid, cursor3d) {
  // Update rotations and rotation velocities based on forces applied
  // via a mouse click & drag event
  const w0 = 40.0;
  const extension = new Float64Array(3);
  const { cursorPosition, clickPosition } = cursor3d;

  return function(position, velocity, dt) {
    // Find the displacement of the clicked position on the globe
    // from the current mouse position
    subtract(extension, cursorPosition, clickPosition);

    // Convert to changes in longitude, latitude, and altitude
    ellipsoid.ecefToDeltaLonLatAlt(extension, extension,
      clickPosition, position);

    const [x, y] = extension;
    const [dLon, dVx] = oscillatorChange(x, velocity[0], dt, w0);
    const [dLat, dVy] = oscillatorChange(y, velocity[1], dt, w0);

    velocity[0] += dVx;
    velocity[1] += dVy;

    return new Float64Array([dLon, dLat, 0.0]);
  };
}

function initCoast(ellipsoid) {
  // Update rotations based on a freely spinning globe (no forces)
  const damping = 3.0; // Viscous damping
  const radius = ellipsoid.meanRadius();
  const minSpeed = 0.03;

  return function(position, velocity, dt) {
    // TODO: switch to exact formula? (not finite difference)

    const speed = Math.hypot(velocity[0], velocity[1]);
    if (speed < minSpeed * position[2] / radius) {
      velocity.fill(0.0, 0, 2);
      return new Float64Array(3); // No change in position
    }

    // Adjust previous velocities for damping over the past time interval
    const dvDamp = -1.0 * damping * dt;
    velocity[0] += velocity[0] * dvDamp;
    velocity[1] += velocity[1] * dvDamp;

    // Return change in position
    const dLon = velocity[0] * dt;
    const dLat = velocity[1] * dt;
    return new Float64Array([dLon, dLat, 0.0]);
  };
}

function initCameraDynamics(ellipsoid, camera, cursor3d, flights) {
  // Velocity is the time differential of camera.position
  const velocity = new Float64Array(3);

  // Initialize some values and working arrays
  let time = 0.0;
  const rayVec = new Float64Array(4);

  // Initialize values & update functions for translations & rotations
  const zoom   = initZoom(ellipsoid, cursor3d);
  const rotate = initRotation(ellipsoid, cursor3d);
  const coast  = initCoast(ellipsoid);

  // Return methods to read/update state
  return {
    update,
    stopZoom: () => velocity.fill(0.0, 2),
    stopCoast: () => velocity.fill(0.0, 0, 2),
  };

  function update(newTime) {
    const deltaT = newTime - time;
    time = newTime;
    // If timestep too big, wait till next frame to update physics
    if (deltaT > 0.25) return false;

    if (cursor3d.isClicked()) flights.cancel();

    const dPos =
      (cursor3d.isClicked()) ? rotate(camera.position(), velocity, deltaT) :
      (flights.active()) ? flights.update(camera.position(), velocity, deltaT) :
      coast(camera.position(), velocity, deltaT);
    camera.update(dPos);

    const moved = dPos.some(c => c != 0.0);
    if (!cursor3d.isZooming()) return moved;

    flights.cancel();
    // Update 2D screen position of 3D zoom position
    const visible = camera.ecefToScreenRay(rayVec, cursor3d.zoomPosition);
    if (!visible) {
      velocity.fill(0.0, 2); // TODO: is this needed? Or keep coasting?
      cursor3d.stopZoom();
      return moved;
    }

    if (cursor3d.isClicked()) cursor3d.zoomRay.set(rayVec);
    const zoomChange = zoom(camera.position(), velocity, deltaT);
    camera.update(zoomChange);
    return true;
  }
}

function init(userParams) {
  const params = setParams(userParams);
  const { ellipsoid, view, units } = params;

  const camera = initCamera(params);
  const cursor = initCursor3d(params, camera);
  const flights = initFlights(params, camera);
  const dynamics = initCameraDynamics(ellipsoid, camera, cursor, flights);

  let camMoving, cursorChanged;

  return {
    view,
    radius: ellipsoid.meanRadius,

    project: (xy, geodetic) => camera.project(xy, units.convert(geodetic)),
    cameraPos: () => units.invert(camera.position()),
    cursorPos: () => units.invert(cursor.cursorLonLat),

    camMoving: () => camMoving,
    isOnScene: cursor.isOnScene,
    wasTapped: cursor.wasTapped,
    cursorChanged: () => cursorChanged,

    flyTo: (destination) => flights.flyTo(units.convert(destination)),
    update,
  };

  function update(time) {
    // Input represents the time this function was called, in seconds
    const resized = view.changed();

    camMoving = dynamics.update(time) || resized;
    cursorChanged = cursor.hasChanged() || camMoving;
    if (cursorChanged) cursor.update(camera.position(), dynamics);

    return camMoving;
  }
}

export { init };
