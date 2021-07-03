import * as vec4 from 'gl-matrix/vec4';

export function initCursor3d(getRayParams, ellipsoid, initialPosition) {
  // Input getRayParams is a method from yawgl.initView, converting screen X/Y
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
    vec4.transformMat4(ecefRay, cursorRay, camera.rotation);

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
