import { initTouch } from "touch-sampler";
import * as vec4 from "gl-matrix/vec4";

export function initCursor2d(params, camera) {
  const { display, view, ellipsoid } = params;

  const cursor2d = initTouch(display);

  const screenRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  const ecefRay = new Float64Array(4);
  const cursorLonLat = new Float64Array(3);

  function project(cursorPosition) {
    view.getRayParams(screenRay, cursor2d.x(), cursor2d.y());
    vec4.transformMat4(ecefRay, screenRay, camera.rotation);
    // NOTE: cursorPosition will be overwritten!
    const onScene = ellipsoid.shoot(cursorPosition, camera.ecefPos, ecefRay);

    // Convert to longitude/latitude/altitude
    if (onScene) ellipsoid.ecef2geocentric(cursorLonLat, cursorPosition);

    return onScene;
  }

  return Object.assign(cursor2d, { project, screenRay, cursorLonLat });
}
