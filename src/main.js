import { initView } from 'yawgl';
import { initTouch } from 'touch-sampler';
import { initEllipsoid } from "./ellipsoid.js";
import { initCameraDynamics } from "./physics.js";
import { initCursor3d } from "./cursor3d.js";

const degrees = 180.0 / Math.PI;

export function init(display, center, altitude) {
  // Input display is an HTML element where the ball will be represented
  // Input center is a pointer to a 2-element array containing initial
  // longitude and latitude for the camera
  // Input altitude is a floating point value indicating initial altitude

  // Add event handlers and position tracking to display element
  const cursor2d = initTouch(display);
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
