import { initTouchy } from 'touchy';
import { initEllipsoid } from "./ellipsoid.js";
import { initCameraDynamics } from "./physics.js";
import { initCursor3d } from "./cursor3d.js";

export function initSpinningBall(display, center, altitude) {
  // Input display is an object created by yawgl.initView()
  // Input initialPos is an array containing longitude, latitude, altitude

  // Add event handlers and position tracking to display element
  const cursor2d = initTouchy(display.element);

  // Initialize ellipsoid, and methods for computing positions relative to it
  const ellipsoid = initEllipsoid();

  // Initialize camera dynamics: time, position, velocity, etc.
  // First check and convert user parameters for initial position
  var initialPos = (center && Array.isArray(center) && center.length === 2)
    ? [toRadians(center[0]), toRadians(center[1])]
    : [0.0, 0.0];
  initialPos[2] = (altitude)
    ? altitude
    : 4.0 * ellipsoid.meanRadius();
  const camera = initCameraDynamics(display, ellipsoid, initialPos);

  // Initialize interaction with the ellipsoid via the mouse and screen
  const cursor3d = initCursor3d( display.getRayParams, 
      ellipsoid, camera.position );

  var camMoving, cursorChanged;

  return {
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

  function update(time, resized) {
    // Input time is a primitive floating point value representing the 
    // number of seconds since the last call
    // Input resized is a primitive Boolean indicating whether the display
    // has been resized since the last call

    // Update camera dynamics
    camMoving = camera.update(time, resized, cursor3d);

    // Update cursor positions, if necessary
    cursorChanged = cursor2d.hasChanged() || camMoving || cursor3d.wasTapped();
    if (cursorChanged) cursor3d.update(cursor2d, camera);

    return camMoving;
  }
}

function toRadians(degrees) {
  return degrees * Math.PI / 180.0;
}
