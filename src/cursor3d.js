import { initCursor2d } from "./cursor2d.js";

export function initCursor3d(params, camera) {
  const { initialPosition, minHeight, maxHeight } = params;

  const cursor2d = initCursor2d(params, camera);

  // Cursor positions are computed & stored in ECEF coordinates (x,y,z)
  const cursorPosition = new Float64Array(3);
  const clickPosition = new Float64Array(3);
  const zoomPosition = new Float64Array(3);
  // Track target screen ray and altitude for zooming
  const zoomRay = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  let targetHeight = initialPosition[2];

  // Flags about the cursor state
  let onScene = false;
  let clicked = false;
  let zooming = false;
  let wasTapped = false;
  let zoomFix = false; // Whether to fix the screen position of the zoom

  return {
    // POINTERs to local arrays. WARNING: local vals can be changed outside!
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
    zoomTarget: () => targetHeight,

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
      // Also stop any coasting in the altitude direction
      dynamics.stopZoom();
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
