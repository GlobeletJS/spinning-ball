// Very similar to greggman's module:

function initView(porthole, fieldOfView) {
  // The porthole is an HTML element acting as a window into a 3D world
  // fieldOfView is the vertical view angle range in degrees (floating point)

  // Compute values for transformation between the 3D world and the 2D porthole
  var portRect, width, height, aspect;
  var tanFOV = Math.tan(fieldOfView * Math.PI / 180.0 / 2.0);
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
    let newWidth = portRect.right - portRect.left;
    let newHeight = portRect.bottom - portRect.top;

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
    var x = clientX - portRect.left;
    var y = portRect.bottom - clientY - 1; // Flip sign to make +y upward

    // Normalized distances from center of box. We normalize by pixel DISTANCE
    // rather than pixel count, to ensure we get -1 and +1 at the ends.
    // (Confirm by considering the 2x2 case)
    var xratio = 2 * x / (width - 1) - 1;
    var yratio = 2 * y / (height - 1) -1;

    rayVec[0] = xratio * maxRay[0];
    rayVec[1] = yratio * maxRay[1];
    //rayVec[2] = -1.0;
    //rayVec[3] = 0.0;
    return;
  }
}

function initCursor() {
  // What does an animation need to know about the cursor at each frame?
  // First, whether the user did any of the following since the last frame:
  //  - Started new actions
  var touchStarted = false; // Touched or clicked the element
  var zoomStarted  = false; // Rotated mousewheel, or started two-finger touch
  //  - Changed something
  var moved  = false;       // Moved mouse or touch point
  var zoomed = false;       // Rotated mousewheel, or adjusted two-finger touch
  //  - Is potentially in the middle of something
  var tapping = false;      // No touchEnd, and no cursor motion
  //  - Ended actions
  var touchEnded = false;   // mouseup or touchend/cancel/leave
  var tapped = false;       // Completed a click or tap action

  // We also need to know the current cursor position and zoom scale
  var cursorX = 0;
  var cursorY = 0;
  var zscale = 1.0;

  // For tap/click reporting, we need to remember where the touch started
  var startX = 0;
  var startY = 0;
  // What is a click/tap and what is a drag? If the cursor moved more than
  // this threshold between touchStart and touchEnd, it is a drag
  const threshold = 6;

  return {
    // Methods to report local state. These protect local values, returning a copy
    touchStarted: () => touchStarted,
    zoomStarted:  () => zoomStarted,
    moved:        () => moved,
    zoomed:       () => zoomed,
    tapped:       () => tapped,
    touchEnded:   () => touchEnded,
    hasChanged:   () => (moved || zoomed || tapped),
    zscale:       () => zscale,
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
    var dist = Math.abs(cursorX - startX) + Math.abs(cursorY - startY);
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

// Add event listeners to update the state of a cursor object
// Input div is an HTML element on which events will be registered
function initTouchy(div) {
  const cursor = initCursor();

  // Remember the distance between two pointers
  var lastDistance = 1.0;
  
  // Capture the drag event so we can disable any default actions
  div.addEventListener('dragstart', function(drag) {
    drag.preventDefault();
    return false;
  }, false);

  // Add mouse events
  div.addEventListener('mousedown',   cursor.startTouch, false);
  div.addEventListener('mousemove',   cursor.move,       false);
  div.addEventListener('mouseup',     cursor.endTouch,   false);
  div.addEventListener('mouseleave',  cursor.endTouch,   false);
  div.addEventListener('wheel',       wheelZoom,         false);

  // Add touch events
  div.addEventListener('touchstart',  initTouch,       false);
  div.addEventListener('touchmove',   moveTouch,       false);
  div.addEventListener('touchend',    cursor.endTouch, false);
  div.addEventListener('touchcancel', cursor.endTouch, false);

  // Return a pointer to the cursor object
  return cursor;

  function initTouch(evnt) {
    evnt.preventDefault();
    switch (evnt.touches.length) {
      case 1: 
        cursor.startTouch(evnt.touches[0]);
        break;
      case 2:
        var midpoint = getMidPoint(evnt.touches[0], evnt.touches[1]);
        cursor.startTouch(midpoint);
        cursor.startZoom(midpoint);
        // Initialize the starting distance between touches
        lastDistance = midpoint.distance;
        break;
      default:
        cursor.endTouch(evnt);
    }
  }

  function moveTouch(evnt) {
    evnt.preventDefault();
    // NOTE: MDN says to add the touchmove handler within the touchstart handler
    // https://developer.mozilla.org/en-US/docs/Web/API/Touch_events/Using_Touch_Events
    switch (evnt.touches.length) {
      case 1:
        cursor.move(evnt.touches[0]);
        break;
      case 2:
        var midpoint = getMidPoint(evnt.touches[0], evnt.touches[1]);
        // Move the cursor to the midpoint
        cursor.move(midpoint);
        // Zoom based on the change in distance between the two touches
        cursor.zoom(lastDistance / midpoint.distance);
        // Remember the new touch distance
        lastDistance = midpoint.distance;
        break;
      default:
        return false;
    }
  }

  // Convert a two-touch event to a single event at the midpoint
  function getMidPoint(p0, p1) {
    var dx = p1.clientX - p0.clientX;
    var dy = p1.clientY - p0.clientY;
    return {
      clientX: p0.clientX + dx / 2,
      clientY: p0.clientY + dy / 2,
      distance: Math.sqrt(dx * dx + dy * dy),
    }
  }

  function wheelZoom(turn) {
    turn.preventDefault();
    cursor.startZoom(turn);
    // We ignore the dY from the browser, since it may be arbitrarily scaled
    // based on screen resolution or other factors. We keep only the sign.
    // See https://github.com/Leaflet/Leaflet/issues/4538
    var zoomScale = 1.0 + 0.2 * Math.sign(turn.deltaY);
    cursor.zoom(zoomScale);
  }
}

/**
 * Common utilities
 * @module glMatrix
 */
var ARRAY_TYPE = typeof Float32Array !== 'undefined' ? Float32Array : Array;
var degree = Math.PI / 180;

/**
 * 3 Dimensional Vector
 * @module vec3
 */

/**
 * Creates a new, empty vec3
 *
 * @returns {vec3} a new 3D vector
 */

function create() {
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
 * @param {vec3} a vector to calculate length of
 * @returns {Number} length of a
 */

function length(a) {
  var x = a[0];
  var y = a[1];
  var z = a[2];
  return Math.sqrt(x * x + y * y + z * z);
}
/**
 * Set the components of a vec3 to the given values
 *
 * @param {vec3} out the receiving vector
 * @param {Number} x X component
 * @param {Number} y Y component
 * @param {Number} z Z component
 * @returns {vec3} out
 */

function set(out, x, y, z) {
  out[0] = x;
  out[1] = y;
  out[2] = z;
  return out;
}
/**
 * Adds two vec3's
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
 * @returns {vec3} out
 */

function add(out, a, b) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
  out[2] = a[2] + b[2];
  return out;
}
/**
 * Subtracts vector b from vector a
 *
 * @param {vec3} out the receiving vector
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
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
 * @param {vec3} a the vector to scale
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
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
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
 * @param {vec3} a vector to normalize
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
 * @param {vec3} a the first operand
 * @param {vec3} b the second operand
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
 * @param {vec3} a the vector to transform
 * @param {mat4} m matrix to transform with
 * @returns {vec3} out
 */

function transformMat4(out, a, m) {
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
 * @param {vec3} a the vector to transform
 * @param {mat3} m the 3x3 matrix to transform with
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

var forEach = function () {
  var vec = create();
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
}();

function initEcefToLocalGeo() {
  var p, p2, sinLon, cosLon, sinLat, cosLat;
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
    transformMat3( diff, diff, toENU );

    // 2. Convert horizontal component to changes in longitude, latitude
    let r = length(anchor);
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
    p2 = normal[0]**2 + normal[2]**2;
    p = Math.sqrt(p2);
    if (p > 0) {
      sinLon = normal[0] / p;
      cosLon = normal[2] / p;
    } else {
      sinLon = 0.0;
      cosLon = 0.0;
    }
    let r = Math.sqrt(p2 + normal[1]**2);
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

function initEllipsoid() {
  // Store ellipsoid parameters
  const semiMajor = 6371.0;  // kilometers
  const semiMinor = 6371.0;  // kilometers

  const e2 = 1.0 - semiMinor**2 / semiMajor**2; // Ellipticity squared
  // Mean radius as defined by the International Union of Geodesy and Geophysics
  // See https://en.wikipedia.org/wiki/Earth_radius#Mean_radius
  const meanRadius = (2.0 * semiMajor + semiMinor) / 3.0;

  var p, p2, sinLat, primeVertRad;

  // Working vectors for shootEllipsoid, findHorizon
  const mCam = new Float64Array(3);
  const mRay = new Float64Array(3);
  const dRay = new Float64Array(3);

  return {
    meanRadius: () => meanRadius,
    ecef2geocentric,
    ecefToDeltaLonLatAlt: initEcefToLocalGeo(),
    geodetic2ecef,
    shoot: shootEllipsoid,
    findHorizon,
  };

  function ecef2geocentric( gcPos, ecefPos ) {
    // Output gcPos is a pointer to a 3-element array, containing geocentric
    //  longitude & latitude (radians) and altitude (meters) coordinates
    // Input ecefPos is a pointer to a 3-element array, containing earth-
    //  centered earth-fixed x,y,z coordinates in the WebGL axis definition

    // Note: order of calculations is chosen to allow calls with same array
    // as input & output (gcPos, ecefPos point to same array)
    p2 = ecefPos[0]**2 + ecefPos[2]**2; // Squared distance from polar axis

    gcPos[0] = Math.atan2( ecefPos[0], ecefPos[2] );     // Longitude

    // NOTE: this "altitude" is distance from SPHERE, not ellipsoid
    gcPos[2] = Math.sqrt( p2 + ecefPos[1]**2 ) - meanRadius; // Altitude

    gcPos[1] = Math.atan2( ecefPos[1], Math.sqrt(p2) );  // Latitude
    return;
  }

  function geodetic2ecef( ecef, geodetic ) {
    // Output ecef is a pointer to a 3-element array containing X,Y,Z values
    //   of the point in earth-centered earth-fixed (ECEF) coordinates
    // Input geodetic is a pointer to a 3-element array, containing
    //   longitude & latitude (in radians) and altitude (in meters)

    // Start from prime vertical radius of curvature -- see
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
    sinLat = Math.sin( geodetic[1] );
    primeVertRad = semiMajor / Math.sqrt( 1.0 - e2 * sinLat**2 );
    // Radial distance from y-axis:
    p = (primeVertRad + geodetic[2]) * Math.cos(geodetic[1]);

    // Compute ECEF position
    ecef[0] = p * Math.sin(geodetic[0]);
    ecef[1] = (primeVertRad + geodetic[2]) * sinLat * (1.0 - e2);
    ecef[2] = p * Math.cos(geodetic[0]);
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
    //  where M is the matrix that scales the ellipsoid to the unit sphere,
    //  i.e., for P = (x,y,z), MP = (x/a, y/b, z/c). Since M is diagonal
    //  (ellipsoid aligned along coordinate axes) we just scale each coordinate.
    mCam.set([
        camera[0] / semiMajor, 
        camera[1] / semiMinor,
        camera[2] / semiMajor 
    ]);
    mRay.set([
        rayVec[0] / semiMajor, 
        rayVec[1] / semiMinor, 
        rayVec[2] / semiMajor 
    ]);

    // We now have <mRay,mRay>*t^2 + 2*<mRay,mCam>*t + <mCam,mCam> - 1 = 0
    var a = dot(mRay, mRay);
    var b = 2.0 * dot(mRay, mCam);
    var c = dot(mCam, mCam) - 1.0;
    var discriminant = b**2 - 4*a*c;

    var intersected, t;
    if (discriminant < 0) {
      intersected = false;
      // Find the point that comes closest to the unit sphere
      //   NOTE: this is NOT the closest point to the ellipsoid!
      //   And it is not even the point on the horizon! It is closer...
      // Minimize a*t^2 + b*t + c, by finding the zero of the derivative
      t = -0.5 * b / a;
    } else {
      intersected = true;
      // We want the closest intersection, with smallest positive t
      // We assume b < 0, if ray is pointing back from camera to ellipsoid
      t = (-b - Math.sqrt(discriminant)) / (2.0*a);
    }

    // NOTE: rayVec is actually a vec4
    scaleAndAdd(intersection, camera, rayVec, t);
    return intersected;
  }

  function findHorizon(horizon, camera, rayVec) {
    // Find the point on the horizon under rayvec.
    // We first adjust rayVec to point it toward the horizon, and then
    // re-shoot the ellipsoid with the corrected ray

    // 1. Find the component of rayVec parallel to camera direction
    normalize(dRay, camera); // Unit vector along camera direction
    var paraLength = dot(dRay, rayVec);
    scale( dRay, dRay, paraLength );

    // 2. Find the component perpendicular to camera direction
    subtract( dRay, rayVec, dRay );
    var perpLength = length(dRay);
    if (perpLength == 0) return false; // No solution if ray is vertical

    // 3. Find the error of the length of the perpendicular component
    var sinAlpha = meanRadius / length(camera); // sin(angle to horizon)
    var tanAlpha = sinAlpha / Math.sqrt(1.0 - sinAlpha * sinAlpha);
    var dPerp = -paraLength * tanAlpha - perpLength;

    // 4. Find the corrected rayVec
    scaleAndAdd(dRay, rayVec, dRay, dPerp / perpLength);

    // 5. Re-shoot the ellipsoid with the corrected rayVec
    shootEllipsoid(horizon, camera, dRay);

    return true;
  }
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
 * @param {mat4} a the source matrix
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
 * @param {mat4} a the matrix to rotate
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

  const halfPi = Math.PI / 2.0;

  // Set initial values
  update(initialPos);

  return {
    position, // WARNING: Exposes local array to changes from outside
    rotation,
    inverse,
    update,
  };

  function update(geodetic) {
    // Limit rotation around screen x-axis to keep global North pointing up
    geodetic[1] = Math.min(Math.max(-halfPi, geodetic[1]), halfPi);
    // Avoid accumulation of large values in longitude
    if (geodetic[0] >  Math.PI) geodetic[0] -= 2.0 * Math.PI;
    if (geodetic[0] < -Math.PI) geodetic[0] += 2.0 * Math.PI;

    // Compute ECEF coordinates. NOTE WebGL coordinate convention: 
    // +x to right, +y to top of screen, and +z into the screen
    ellipsoid.geodetic2ecef( position, geodetic );

    // Rotation: y first, so it will be left of x operator in final matrix
    // (gl-matrix library 'post-multplies' by each new matrix)
    // Positive angles about Y are towards the +X axis, or East longitude.
    fromYRotation( rotation, geodetic[0] );
    // Positive angles about X are towards the -Y axis!
    // (from Y to Z, and Z to -Y). But geodetic[1] is a latitude, toward N
    rotateX( rotation, rotation, -geodetic[1] );

    // The inverse of a rotation matrix is its transpose
    transpose( inverse, rotation );
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

function create$2() {
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
 * @param {vec4} a the vector to transform
 * @param {mat4} m matrix to transform with
 * @returns {vec4} out
 */

function transformMat4$1(out, a, m) {
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

var forEach$1 = function () {
  var vec = create$2();
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
}();

function initEdgePoints(ellipsoid, camPos, camRot, screen) {
  // Allocate working arrays and variables
  const rayVec = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  const camRay = new Float64Array(4);
  const rayHit = new Float64Array(3);
  var tanX, tanY;

  // Construct a list of points around the screen edges
  const screenPoints = [
    [-1.0, -1.0], // Bottom left
    [-0.5, -1.0],
    [ 0.0, -1.0], // Bottom center
    [ 0.5, -1.0],
    [ 1.0, -1.0], // Bottom right
    [ 1.0, -0.5],
    [ 1.0,  0.0], // Right center
    [ 1.0,  0.5],
    [ 1.0,  1.0], // Top right
    [ 0.5,  1.0],
    [ 0.0,  1.0], // Top center
    [-0.5,  1.0],
    [-1.0,  1.0], // Top left
    [-1.0,  0.5],
    [-1.0,  0.0], // Left center
    [-1.0, -0.5],
    [-1.0, -1.0], // Loop back to bottom left
  ];

  // An edgePoint is the point on the ellipsoid visible from screenPoint
  const edgePoints = [];
  screenPoints.forEach( (point, index) => { 
    edgePoints[index] = [];
  });
  update();

  return {
    lonLats: edgePoints,  // WARNING: exposed to updates from outside!
    update,
  };

  function update() {
    // Update the view angles at the screen edges
    tanX = screen.rightEdge();
    tanY = screen.topEdge();

    // Find the ellipsoid intersection at each screen point
    screenPoints.forEach(shoot);
  }

  function shoot(screenPos, index) {
    // Construct the ray vector
    rayVec[0] = screenPos[0] * tanX;
    rayVec[1] = screenPos[1] * tanY;
    // Rotate to model coordinates (Earth-Centered Earth-Fixed)
    transformMat4$1(camRay, rayVec, camRot);

    // Find intersection of ray with ellipsoid
    var hit = ellipsoid.shoot(rayHit, camPos, camRay);
    // If it didn't intersect, find the nearest point on the horizon
    if (!hit) ellipsoid.findHorizon(rayHit, camPos, camRay);

    // Convert to longitude/latitude. NOTE: geocentric!!
    ellipsoid.ecef2geocentric(rayHit, rayHit);

    edgePoints[index][0] = rayHit[0];
    edgePoints[index][1] = rayHit[1];
  }
}

function updateOscillator(pos, vel, ext, w0, dt, i1, i2) {
  // Update position and velocity for a critically damped oscillator, following
  // http://mathworld.wolfram.com/CriticallyDampedSimpleHarmonicMotion.html
  
  // Inputs/outputs pos, vel are pointers to arrays
  // Inputs w0, t are primitive floating point values, indicating the
  //   natural frequency of the oscillator and the time step
  // Inputs i1, i2 are primitive integer values, indicating components to update

  var expTerm = Math.exp( -w0 * dt );

  for (let i = i1; i <= i2; i++) {
    var tmp = (vel[i] + w0 * ext[i]) * dt * expTerm;
    vel[i] += (expTerm - 1) * vel[i] - w0 * tmp;
    pos[i] += (expTerm - 1) * ext[i] + tmp;
  }
  return;
}

// initZoom: Update camera altitude based on target set by mouse wheel events
//  or two-finger pinch movements
function initZoom( ellipsoid ) {
  const w0 = 14.14; // Natural frequency of oscillator
  const minHeight = ellipsoid.meanRadius() * 0.00001;
  const maxHeight = ellipsoid.meanRadius() * 8.0;
  const minVelocity = 0.001;
  const maxRotation = 0.15;

  // NOTE: everything below ASSUMES mass = 1
  var minEnergy = 0.5 * minVelocity * minVelocity;
  var extension, kineticE, potentialE;
  const dPos = new Float64Array(3);

  return function( position, velocity, cursor3d, deltaTime, track ) {
    // Input cursor3d is a pointer to an object
    // Inputs position, velocity are pointers to 3-element arrays
    // Input deltaTime is a primitive floating point value

    var targetHeight = cursor3d.zoomTarget();

    // Save old altitude
    var oldAltitude = position[2];

    dPos[2] = position[2] - targetHeight;
    updateOscillator(position, velocity, dPos, w0, deltaTime, 2, 2);

    if (track) {
      // Adjust rotation to keep zoom location fixed on screen
      dPos.set(position);
      dragonflyStalk( dPos, cursor3d.zoomRay, cursor3d.zoomPosition, ellipsoid );
      // Restrict size of rotation in one time step
      subtract( dPos, dPos, position );
      var limited = limitRotation( dPos, maxRotation );
      add( position, position, dPos );
    }

    // Scale rotational velocity by the ratio of the height change
    var heightScale = position[2] / oldAltitude;
    velocity[0] *= heightScale;
    velocity[1] *= heightScale;

    if (cursor3d.isClicked() || limited) return;

    // Stop if we are already near steady state
    kineticE = 0.5 * velocity[2] ** 2;
    extension = position[2] - targetHeight;
    potentialE = 0.5 * (w0 * extension) ** 2;
    if (kineticE + potentialE < minEnergy * targetHeight) {
      targetHeight = position[2];
      velocity[2] = 0.0;
      cursor3d.stopZoom();
    }
    return;
  }
}

function limitRotation( dPos, maxRotation ) {
  // Input dPos is a pointer to a 2-element array containing lon, lat changes
  // maxRotation is a primitive floating point value

  // Check for longitude value crossing antimeridian
  if (dPos[0] >  Math.PI) dPos[0] -= 2.0 * Math.PI;
  if (dPos[0] < -Math.PI) dPos[0] += 2.0 * Math.PI;

  if (Math.abs(dPos[0]) > maxRotation) {
    var tmp = Math.min(Math.max(-maxRotation, dPos[0]), maxRotation) / dPos[0];
    dPos[0] *= tmp;
    dPos[1] *= tmp;
    return true;
  }
  return false;
}

// Given a 3D scene coordinate over which a zoom action was initiated,
// and a distance between the screen and the center of the 3D scene,
// compute the rotations required to align the 3D coordinate along
// the original screen ray.  See
// https://en.wikipedia.org/wiki/Dragonfly#Motion_camouflage
// TODO: Clean this up. Just use difference of lat/lon under ray?
function dragonflyStalk(outRotation, ray, scenePos, ellipsoid) {
  // Output outRotation is a pointer to a vec3
  // Input ray is a pointer to a vec3
  // Input scenePos is a pointer to a 3D cursor object

  // Find the ray-sphere intersection in unrotated model space coordinates
  var target = new Float64Array(3);
  var unrotatedCamPos = [0.0, 0.0, outRotation[2] + length(scenePos)];
  var onEllipse = ellipsoid.shoot(target, unrotatedCamPos, ray);
  if (!onEllipse) return; // No intersection!

  // Find the rotation about the y-axis required to bring scene point into 
  // the  x = target[0]  plane
  // First find distance of scene point from scene y-axis
  var sceneR = Math.sqrt( scenePos[0] ** 2 + scenePos[2] ** 2 );
  // If too short, exit rather than tipping poles out of y-z plane
  if ( sceneR < Math.abs(target[0]) ) return;
  var targetRotY = Math.asin( target[0] / sceneR );
  outRotation[0] = 
    Math.atan2( scenePos[0], scenePos[2] ) - // Y-angle of scene vector
    //Math.asin( target[0] / sceneR );       // Y-angle of target point
    targetRotY;

  // We now know the x and y coordinates of the scene vector after rotation
  // around the y-axis: (x = target[0], y = scenePos[1])
  // Find the z-coordinate so we can compute the remaining required rotation
  var zRotated = sceneR * Math.cos(targetRotY);

  // Find the rotation about the screen x-axis required to bring the scene
  // point into the target y = target[1] plane
  // Assumes 0 angle is aligned along Z, and angle > 0 is rotation toward -y !
  outRotation[1] = 
    Math.atan2( -1 * target[1], target[2] ) -  // X-angle of target point
    Math.atan2( -1 * scenePos[1], zRotated );  // X-angle of scene vector

  return;
}

// initRotation: Updates rotations and rotation velocities based on forces
// applied via a mouse click & drag event.
function initRotation( ellipsoid ) {
  const w0 = 40.0;
  const extension = new Float64Array(3);

  return function( position, velocity, mouse3d, deltaTime ) {
    // Input mouse3d is a pointer to a mouse object
    // Inputs position, velocity are pointers to vec3s
    // Input deltaTime is a primitive floating point value

    // Find the displacement of the clicked position on the globe
    // from the current mouse position
    subtract( extension, mouse3d.position, mouse3d.clickPosition );

    // Convert to changes in longitude, latitude, and altitude
    ellipsoid.ecefToDeltaLonLatAlt( extension, extension, 
        mouse3d.clickPosition, position );
    // Ignore altitude change for now
    extension[2] = 0.0;

    updateOscillator(position, velocity, extension, w0, deltaTime, 0, 1);
    return;
  }
}

// initCoast: Update rotations based on a freely spinning globe (no forces)
function initCoast( ellipsoid ) {
  const damping = 3.0;
  const radius = ellipsoid.meanRadius();
  const minSpeed = 0.03;

  var dvDamp = 0.0;

  return function( position, velocity, deltaTime ) {
    // Inputs rotation, rotationVel are pointers to 3-element arrays
    // Input deltaTime is a primitive value (floating point)
    // TODO: switch to exact formula? (not finite difference)

    if ( length(velocity) < minSpeed * position[2] / radius ) {
      // Rotation has almost stopped. Go ahead and stop all the way.
      set(velocity, 0.0, 0.0, 0.0);
      return false; // No change to position, no need to re-render
    }

    // Adjust previous velocities for damping over the past time interval
    dvDamp = -1.0 * damping * deltaTime;
    //vec3.scaleAndAdd(velocity, velocity, velocity, dvDamp);
    velocity[0] += velocity[0] * dvDamp;
    velocity[1] += velocity[1] * dvDamp;

    // Update rotations
    //vec3.scaleAndAdd(position, position, velocity, deltaTime);
    position[0] += velocity[0] * deltaTime;
    position[1] += velocity[1] * deltaTime;
    return true;    // Position changed, need to re-render
  };
}

function initProjector(ellipsoid, camPosition, camInverse, screen) {
  const rayVec = new Float64Array(3);
  const ecefTmp = new Float64Array(3);

  return {
    ecefToScreenRay,
    lonLatToScreenXY,
  };

  function lonLatToScreenXY(xy, lonLat) {
    ellipsoid.geodetic2ecef(ecefTmp, lonLat);
    let visible = ecefToScreenRay(rayVec, ecefTmp); // Overwrites rayVec!

    xy[0] = screen.width() * ( 1 + rayVec[0] / screen.rightEdge() ) / 2;
    xy[1] = screen.height() * ( 1 - rayVec[1] / screen.topEdge() ) / 2;
    return visible;
  }

  function ecefToScreenRay(screenRay, ecefPosition) {
    // For a given point on the ellipsoid (in ECEF coordinates) find the
    // rayVec from a given camera position that will intersect it
    
    // Translate to camera position
    subtract(rayVec, ecefPosition, camPosition);
    // rayVec now points from camera to ecef. The sign of the
    // dot product tells us whether it is beyond the horizon
    let visible = ( dot(rayVec, ecefPosition) < 0 );

    // Rotate to camera orientation
    transformMat4(screenRay, rayVec, camInverse);

    // Normalize to z = -1
    screenRay[0] /= -screenRay[2];
    screenRay[1] /= -screenRay[2];
    screenRay[2] = -1.0;

    return visible;
  }
}

function initCameraDynamics(screen, ellipsoid, initialPosition) {
  // Position & velocity are computed in latitude & longitude in radians, and
  //   altitude defined by distance along surface normal, in the same length
  //   units as semiMajor and semiMinor in ellipsoid.js
  const position = new Float64Array(initialPosition);
  const velocity = new Float64Array(3); // Initializes to [0,0,0]

  // Initialize ECEF position, rotation matrix, inverse, and update method
  const ecef = initECEF(ellipsoid, position);

  // Keep track of the longitude/latitude of the edges of the screen
  const edges = initEdgePoints(ellipsoid, ecef.position, ecef.rotation, screen);
  // Initialize transforms from ellipsoid to screen positions
  const projector = initProjector(ellipsoid, ecef.position, ecef.inverse, screen);

  // Initialize some values and working arrays
  var time = 0.0;
  var deltaTime = 0.0;
  const rayVec = new Float64Array(4);

  // Initialize values & update functions for translations & rotations
  const zoom   = initZoom(ellipsoid);
  const rotate = initRotation(ellipsoid);
  const coast  = initCoast(ellipsoid);
  var needToRender = true;

  // Return methods to read/update state
  return {
    position, // WARNING: Exposes local array to changes from outside
    edgesPos: edges.lonLats,

    ecefPos: ecef.position,
    rotation: ecef.rotation,
    inverse: ecef.inverse,

    lonLatToScreenXY: projector.lonLatToScreenXY,

    update,
    stopCoast,
    stopZoom,
  };

  function stopCoast() {
    velocity[0] = 0.0;
    velocity[1] = 0.0;
  }
  function stopZoom() { 
    velocity[2] = 0.0; 
  }

  function update(newTime, resized, cursor3d) {
    // Input time is a primitive floating point value
    // Input cursor3d is a pointer to an object
    deltaTime = newTime - time;
    time = newTime;
    // If timestep too big, wait till next frame to update physics
    if (deltaTime > 0.25) return resized;

    if ( cursor3d.isClicked() ) {       // Rotate globe based on cursor drag
      rotate( position, velocity, cursor3d, deltaTime );
      needToRender = true;
    } else {                           // Let globe spin freely
      needToRender = coast( position, velocity, deltaTime );
    }
    if ( cursor3d.isZooming() ) {       // Update zoom
      // Update ECEF position and rotation/inverse matrices
      ecef.update(position);
      // Update 2D screen position of 3D zoom position
      var visible = projector.ecefToScreenRay( rayVec, cursor3d.zoomPosition );
      if (visible) {
        if ( cursor3d.isClicked() ) cursor3d.zoomRay.set(rayVec);
        zoom( position, velocity, cursor3d, deltaTime, cursor3d.zoomFixed() );
      } else {
        stopZoom(); // TODO: is this needed? Might want to keep coasting
        cursor3d.stopZoom();
      }
      needToRender = true;
    }

    needToRender = needToRender || resized;
    if (needToRender) {
      ecef.update(position);
      edges.update();
    }
    return needToRender;
  }
}

function initCursor3d(getRayParams, ellipsoid, initialPosition) {
  // Input getRayParams is a method from yawgl.screen, converting screen X/Y
  //  to a ray shooting into 3D space
  // Input initialPosition is a geodetic lon/lat/alt vector

  // Cursor positions are computed & stored in ECEF coordinates (x,y,z)
  const cursorPosition = new Float64Array(3);
  const clickPosition = new Float64Array(3);
  const zoomPosition = new Float64Array(3);
  // Derived geocentric longitude, latitude, altitude
  const cursorLonLat = new Float64Array(3);
  // Screen ray for the 2D cursor position
  const cursorRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);

  // Flags about the cursor state
  var onScene = false;
  var clicked = false;
  var zooming = false;
  var wasTapped = false;
  // Whether to fix the screen position of the zoom
  var zoomFix = false;

  // Track target altitude for zooming
  var targetHeight = initialPosition[2];
  const minHeight = ellipsoid.meanRadius() * 0.00001;
  const maxHeight = ellipsoid.meanRadius() * 8.0;
  // Target screen ray for zooming
  const zoomRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);

  // Local working vector
  const ecefRay = new Float64Array(4);

  // Return methods to read/update cursorPosition
  return {
    // POINTERs to local arrays. WARNING: local values can be changed from outside!
    position: cursorPosition, // TODO: why make the name more ambiguous?
    cursorLonLat,
    clickPosition,
    zoomPosition,
    zoomRay,

    // Methods to report local state.
    // These protect the local value, since primitives are passed by value
    isOnScene:  () => onScene,
    isClicked:  () => clicked,
    wasTapped:  () => wasTapped,
    isZooming:  () => zooming,
    zoomFixed:  () => zoomFix,
    zoomTarget: () => targetHeight,

    // Functions to update local state
    update,
    stopZoom,
  };

  function update(cursor2d, camera) {
    // Get screen ray in model coordinates (ECEF)
    getRayParams(cursorRay, cursor2d.x(), cursor2d.y());
    transformMat4$1(ecefRay, cursorRay, camera.rotation);

    // Find intersection of ray with ellipsoid
    onScene = ellipsoid.shoot(cursorPosition, camera.ecefPos, ecefRay);
    if (!onScene) {
      clicked = false;
      stopZoom(camera.position[2]);
      cursor2d.reset();
      return;
    }

    // Update cursor longitude/latitude
    ellipsoid.ecef2geocentric(cursorLonLat, cursorPosition);

    if ( cursor2d.touchEnded() ) {
      clicked = false;
      zoomFix = false;
    }
    wasTapped = cursor2d.tapped();

    if ( cursor2d.touchStarted() ) {
      // Set click position
      clicked = true;
      clickPosition.set(cursorPosition);
      // Assuming this is a click or single touch, stop zooming
      stopZoom(camera.position[2]);
      // Also stop any coasting in the altitude direction
      camera.stopZoom();
      // If this was actually a two-touch zoom, then cursor2d.zoomStarted()...
    }

    if ( cursor2d.zoomStarted() ) {
      zooming = true;
      zoomFix = true;
      zoomPosition.set(cursorPosition);
      zoomRay.set(cursorRay);
      if (!clicked) camera.stopCoast();
    }

    if ( cursor2d.zoomed() ) {
      zooming = true;
      targetHeight *= cursor2d.zscale();
      targetHeight = Math.min(Math.max(minHeight, targetHeight), maxHeight);
    }

    cursor2d.reset();
    return;
  }

  function stopZoom(height) {
    zooming = false;
    zoomFix = false;
    if (height !== undefined) targetHeight = height;
  }
}

const degrees = 180.0 / Math.PI;

function init(display, center, altitude) {
  // Input display is an HTML element where the ball will be represented
  // Input center is a pointer to a 2-element array containing initial
  // longitude and latitude for the camera
  // Input altitude is a floating point value indicating initial altitude

  // Add event handlers and position tracking to display element
  const cursor2d = initTouchy(display);
  // Add a view object to compute ray parameters at points on the display
  const view = initView(display, 25.0);

  // Initialize ellipsoid, and methods for computing positions relative to it
  const ellipsoid = initEllipsoid();

  // Initialize camera dynamics: time, position, velocity, etc.
  // First check and convert user parameters for initial position
  var initialPos = (center && Array.isArray(center) && center.length === 2)
    ? [center[0] / degrees, center[1] / degrees]
    : [0.0, 0.0];
  initialPos[2] = (altitude)
    ? altitude
    : 4.0 * ellipsoid.meanRadius();
  const camera = initCameraDynamics(view, ellipsoid, initialPos);

  // Initialize interaction with the ellipsoid via the mouse and screen
  const cursor3d = initCursor3d(view.getRayParams, ellipsoid, camera.position);

  var camMoving, cursorChanged;

  return {
    view,

    radius:    ellipsoid.meanRadius,

    camMoving: () => camMoving,
    cameraPos: camera.position,
    edgesPos:  camera.edgesPos,

    lonLatToScreenXY: camera.lonLatToScreenXY,

    cursorPos: cursor3d.cursorLonLat,
    isOnScene: cursor3d.isOnScene,
    cursorChanged: () => cursorChanged,
    wasTapped: cursor3d.wasTapped,

    update,
  };

  function update(time) {
    // Input time is a primitive floating point value representing the 
    // time this function was called, in seconds

    // Check for changes in display size
    let resized = view.changed();

    // Update camera dynamics
    camMoving = camera.update(time, resized, cursor3d);

    // Update cursor positions, if necessary
    cursorChanged = cursor2d.hasChanged() || camMoving || cursor3d.wasTapped();
    if (cursorChanged) cursor3d.update(cursor2d, camera);

    return camMoving;
  }
}

export { init };
