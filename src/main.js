import { setParams } from "./params.js";
import { initCamera } from "./camera.js";
import { initCursor3d } from "./cursor3d.js";
import { initCameraDynamics } from "./dynamics.js";

export function init(userParams) {
  const params = setParams(userParams);
  const { ellipsoid, view, unitConversion } = params;

  const camera = initCamera(params);
  const cursor = initCursor3d(params, camera);
  const dynamics = initCameraDynamics(ellipsoid, camera, cursor);

  const cursorTmp = new Float64Array(3);
  const cursorPos = new Float64Array(3);
  let camMoving, cursorChanged;

  return {
    view,
    radius: ellipsoid.meanRadius,
    lonLatToScreenXY: camera.lonLatToScreenXY,

    camMoving: () => camMoving,
    cameraPos: camera.position,

    cursorPos: () => cursorPos.slice(),
    isOnScene: cursor.isOnScene,
    cursorChanged: () => cursorChanged,
    wasTapped: cursor.wasTapped,

    update,
  };

  function update(time) {
    // Input represents the time this function was called, in seconds
    const resized = view.changed();

    camMoving = dynamics.update(time) || resized;
    cursorChanged = cursor.hasChanged() || camMoving;
    if (cursorChanged) cursor.update(camera.position(), dynamics);

    if (cursor.isOnScene()) {
      // Update cursor longitude/latitude/altitude
      ellipsoid.ecef2geocentric(cursorTmp, cursor.cursorPosition);
      cursorPos.set(unitConversion(cursorTmp));
    }

    return camMoving;
  }
}
