import * as vec4 from "gl-matrix/vec4";

export function initCursor3d(params, camera) {
  const { view, ellipsoid, initialPosition, minHeight, maxHeight } = params;

  // Cursor positions are computed & stored in ECEF coordinates (x,y,z)
  const cursorPosition = new Float64Array(3);
  const clickPosition = new Float64Array(3);
  const zoomPosition = new Float64Array(3);
  // Derived geocentric longitude, latitude, altitude
  const cursorLonLat = new Float64Array(3);
  // Screen ray for the 2D cursor position
  const cursorRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);

  // Flags about the cursor state
  let onScene = false;
  let clicked = false;
  let zooming = false;
  let wasTapped = false;
  let zoomFix = false; // Whether to fix the screen position of the zoom

  // Track target altitude for zooming
  let targetHeight = initialPosition[2];
  // Target screen ray for zooming
  const zoomRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  // Local working vector
  const ecefRay = new Float64Array(4);

  return {
    // POINTERs to local arrays. WARNING: local vals can be changed outside!
    cursorPosition,
    cursorLonLat,
    clickPosition,
    zoomPosition,
    zoomRay,

    // Methods to report local state
    isOnScene: () => onScene,
    isClicked: () => clicked,
    wasTapped: () => wasTapped,
    isZooming: () => zooming,
    zoomFixed: () => zoomFix,
    zoomTarget: () => targetHeight,

    // Functions to update local state
    update,
    stopZoom,
  };

  function update(cursor2d, dynamics) {
    // Get screen ray in model coordinates (ECEF)
    view.getRayParams(cursorRay, cursor2d.x(), cursor2d.y());
    vec4.transformMat4(ecefRay, cursorRay, camera.rotation);

    // Find intersection of ray with ellipsoid
    onScene = ellipsoid.shoot(cursorPosition, camera.ecefPos, ecefRay);
    if (!onScene) {
      clicked = zoomFix = false;
      return cursor2d.reset();
    }

    // Update cursor longitude/latitude
    ellipsoid.ecef2geocentric(cursorLonLat, cursorPosition);

    if (cursor2d.touchEnded()) clicked = zoomFix = false;
    wasTapped = cursor2d.tapped();

    if (cursor2d.touchStarted()) {
      clicked = true;
      clickPosition.set(cursorPosition);
      // Assuming this is a click or single touch, stop zooming
      stopZoom(camera.position()[2]);
      // Also stop any coasting in the altitude direction
      dynamics.stopZoom();
      // If this was actually a two-touch zoom, then cursor2d.zoomStarted()...
    }

    if (cursor2d.zoomStarted()) {
      zooming = zoomFix = true;
      zoomPosition.set(cursorPosition);
      zoomRay.set(cursorRay);
      if (!clicked) dynamics.stopCoast();
    }

    if (cursor2d.zoomed()) {
      zooming = true;
      targetHeight *= cursor2d.zscale();
      targetHeight = Math.min(Math.max(minHeight, targetHeight), maxHeight);
    }

    cursor2d.reset();
  }

  function stopZoom(height) {
    zooming = zoomFix = false;
    if (height !== undefined) targetHeight = height;
  }
}
