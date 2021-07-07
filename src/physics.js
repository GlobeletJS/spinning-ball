import { initECEF } from "./ecef.js";
import { initEdgePoints } from "./edge-points.js";
import { initZoom } from "./zoom.js";
import { initRotation, initCoast } from "./rotate.js";
import { initProjector } from "./projector.js";

export function initCameraDynamics(screen, ellipsoid, initialPosition) {
  // Position & velocity are computed in latitude & longitude in radians, and
  //   altitude defined by distance along surface normal, in the same length
  //   units as semiMajor and semiMinor in ellipsoid.js
  const position = new Float64Array(initialPosition);
  const velocity = new Float64Array(3); // Initializes to [0,0,0]

  // Initialize ECEF position, rotation matrix, inverse, and update method
  const ecef = initECEF(ellipsoid, position);

  // Keep track of the longitude/latitude of the edges of the screen
  const edges = initEdgePoints(ellipsoid, ecef.position, ecef.rotation, screen);
  // Initialize transforms from ellipsoid to screen positions
  const projector = initProjector(ellipsoid,
    ecef.position, ecef.inverse, screen);

  // Initialize some values and working arrays
  var time = 0.0;
  const rayVec = new Float64Array(4);

  // Initialize values & update functions for translations & rotations
  const zoom   = initZoom(ellipsoid);
  const rotate = initRotation(ellipsoid);
  const coast  = initCoast(ellipsoid);

  // Return methods to read/update state
  return {
    position, // WARNING: Exposes local array to changes from outside
    edgesPos: edges.lonLats,

    ecefPos: ecef.position,
    rotation: ecef.rotation,
    inverse: ecef.inverse,

    lonLatToScreenXY: projector.lonLatToScreenXY,

    update,
    stopCoast,
    stopZoom,
  };

  function stopCoast() {
    velocity[0] = 0.0;
    velocity[1] = 0.0;
  }
  function stopZoom() {
    velocity[2] = 0.0;
  }

  function update(newTime, resized, cursor3d) {
    // Input time is a primitive floating point value
    // Input cursor3d is a pointer to an object
    const deltaTime = newTime - time;
    time = newTime;
    // If timestep too big, wait till next frame to update physics
    if (deltaTime > 0.25) return resized;

    var needToRender;
    if ( cursor3d.isClicked() ) {       // Rotate globe based on cursor drag
      rotate( position, velocity, cursor3d, deltaTime );
      needToRender = true;
    } else {                           // Let globe spin freely
      needToRender = coast( position, velocity, deltaTime );
    }
    if ( cursor3d.isZooming() ) {       // Update zoom
      // Update ECEF position and rotation/inverse matrices
      ecef.update(position);
      // Update 2D screen position of 3D zoom position
      var visible = projector.ecefToScreenRay( rayVec, cursor3d.zoomPosition );
      if (visible) {
        if ( cursor3d.isClicked() ) cursor3d.zoomRay.set(rayVec);
        zoom( position, velocity, cursor3d, deltaTime, cursor3d.zoomFixed() );
      } else {
        stopZoom(); // TODO: is this needed? Might want to keep coasting
        cursor3d.stopZoom();
      }
      needToRender = true;
    }

    needToRender = needToRender || resized;
    if (needToRender) {
      ecef.update(position);
      edges.update();
    }
    return needToRender;
  }
}
