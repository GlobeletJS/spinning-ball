import * as vec4 from 'gl-matrix/vec4';

export function initEdgePoints(ellipsoid, camPos, camRot, screen) {
  // Allocate working arrays and variables
  const rayVec = new Float64Array([0.0, 0.0, -1.0, 0.0]);
  const camRay = new Float64Array(4);
  const rayHit = new Float64Array(3);
  var tanX, tanY;

  // Construct a list of points around the screen edges
  const screenPoints = [
    [-1.0, -1.0], // Bottom left
    [-0.5, -1.0],
    [ 0.0, -1.0], // Bottom center
    [ 0.5, -1.0],
    [ 1.0, -1.0], // Bottom right
    [ 1.0, -0.5],
    [ 1.0,  0.0], // Right center
    [ 1.0,  0.5],
    [ 1.0,  1.0], // Top right
    [ 0.5,  1.0],
    [ 0.0,  1.0], // Top center
    [-0.5,  1.0],
    [-1.0,  1.0], // Top left
    [-1.0,  0.5],
    [-1.0,  0.0], // Left center
    [-1.0, -0.5],
    [-1.0, -1.0], // Loop back to bottom left
  ];

  // An edgePoint is the point on the ellipsoid visible from screenPoint
  const edgePoints = [];
  screenPoints.forEach( (point, index) => { 
    edgePoints[index] = [];
  });
  update();

  return {
    lonLats: edgePoints,  // WARNING: exposed to updates from outside!
    update,
  };

  function update() {
    // Update the view angles at the screen edges
    tanX = screen.rightEdge();
    tanY = screen.topEdge();

    // Find the ellipsoid intersection at each screen point
    screenPoints.forEach(shoot);
  }

  function shoot(screenPos, index) {
    // Construct the ray vector
    rayVec[0] = screenPos[0] * tanX;
    rayVec[1] = screenPos[1] * tanY;
    // Rotate to model coordinates (Earth-Centered Earth-Fixed)
    vec4.transformMat4(camRay, rayVec, camRot);

    // Find intersection of ray with ellipsoid
    var hit = ellipsoid.shoot(rayHit, camPos, camRay);
    // If it didn't intersect, find the nearest point on the horizon
    if (!hit) ellipsoid.findHorizon(rayHit, camPos, camRay);

    // Convert to longitude/latitude. NOTE: geocentric!!
    ellipsoid.ecef2geocentric(rayHit, rayHit);

    edgePoints[index][0] = rayHit[0];
    edgePoints[index][1] = rayHit[1];
  }
}
