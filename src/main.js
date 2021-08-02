import { setParams } from "./params.js";
import { initCamera } from "./camera.js";
import { initTouch } from "touch-sampler";
import { initCursor3d } from "./cursor3d.js";
import { initCameraDynamics } from "./dynamics.js";

export function init(userParams) {
  const params = setParams(userParams);
  const { ellipsoid, display, view, unitConversion } = params;

  const camera = initCamera(params); // Transforms related to view position
  const cursor2d = initTouch(display); // Event handlers, position tracking
  const cursor3d = initCursor3d(params, camera); // Maps cursor2d to ellipsoid

  const dynamics = initCameraDynamics(ellipsoid, camera, cursor3d);
  let camMoving, cursorChanged;

  const cursorTmp = new Float64Array(3);
  const cursorPos = new Float64Array(3);

  return {
    view,

    radius: ellipsoid.meanRadius,
    lonLatToScreenXY: camera.lonLatToScreenXY,

    camMoving: () => camMoving,
    cameraPos: camera.position,

    cursorPos: () => cursorPos.slice(),
    isOnScene: cursor3d.isOnScene,
    cursorChanged: () => cursorChanged,
    wasTapped: cursor3d.wasTapped,

    update,
  };

  function update(time) {
    // Input represents the time this function was called, in seconds
    const resized = view.changed();

    camMoving = dynamics.update(time) || resized;
    cursorChanged = cursor2d.hasChanged() || camMoving || cursor3d.wasTapped();
    if (cursorChanged) cursor3d.update(cursor2d, dynamics);

    if (cursor3d.isOnScene()) {
      // Update cursor longitude/latitude/altitude
      ellipsoid.ecef2geocentric(cursorTmp, cursor3d.cursorPosition);
      cursorPos.set(unitConversion(cursorTmp));
    }

    return camMoving;
  }
}
