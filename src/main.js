import { setParams } from "./params.js";
import { initCamera } from "./camera.js";
import { initCursor3d } from "./cursor3d.js";
import { initCameraDynamics } from "./dynamics.js";

export function init(userParams) {
  const params = setParams(userParams);
  const { ellipsoid, view, units } = params;

  const camera = initCamera(params);
  const cursor = initCursor3d(params, camera);
  const dynamics = initCameraDynamics(ellipsoid, camera, cursor);

  let camMoving, cursorChanged;

  return {
    view,
    radius: ellipsoid.meanRadius,

    project: (pt) => camera.project(units.convert(pt)),
    cameraPos: () => units.invert(camera.position()),
    cursorPos: () => units.invert(cursor.cursorLonLat),

    camMoving: () => camMoving,
    isOnScene: cursor.isOnScene,
    wasTapped: cursor.wasTapped,
    cursorChanged: () => cursorChanged,

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
