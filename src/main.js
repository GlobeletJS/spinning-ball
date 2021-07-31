import { setParams } from "./params.js";
import { initCamera } from "./camera.js";
import { initTouch } from "touch-sampler";
import { initCursor3d } from "./cursor3d.js";
import { initCameraDynamics } from "./dynamics.js";

export function init(userParams) {
  const params = setParams(userParams);
  const { ellipsoid, display, view } = params;

  // Initialize camera: transforms related to view position
  const camera = initCamera(params);

  // Add event handlers and position tracking to display element
  const cursor2d = initTouch(display);
  // Initialize interaction with the ellipsoid via the mouse and screen
  const cursor3d = initCursor3d(params, camera);

  // Initialize camera dynamics: time, position, velocity, etc.
  const dynamics = initCameraDynamics(ellipsoid, camera, cursor3d);

  let camMoving, cursorChanged;

  return {
    view,

    radius: ellipsoid.meanRadius,

    camMoving: () => camMoving,
    cameraPos: camera.position,

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
    const resized = view.changed();

    // Update camera dynamics, and 3D cursor position (if necessary)
    camMoving = dynamics.update(time) || resized;
    cursorChanged = cursor2d.hasChanged() || camMoving || cursor3d.wasTapped();
    if (cursorChanged) cursor3d.update(cursor2d, dynamics);

    return camMoving;
  }
}
