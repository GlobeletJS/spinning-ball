import * as mat4 from "gl-matrix/mat4";

export function initECEF(ellipsoid, initialPos) {
  // From the geodetic position, we derive Earth-Centered Earth-Fixed (ECEF)
  // coordinates and a rotation matrix
  // These are suitable for rendering Relative To Eye (RTE), as described in
  // P Cozzi, 3D Engine Design for Virtual Globes, www.virtualglobebook.com
  const position = new Float64Array([0.0, 0.0, 0.0, 1.0]);
  const rotation = mat4.create();  // Note: single precision!! (Float32Array)
  const inverse  = mat4.create();

  const { min, max, PI } = Math;

  // Set initial values
  update(initialPos);

  return {
    position, // WARNING: Exposes local array to changes from outside
    rotation,
    inverse,
    update,
  };

  function update(geodetic) {
    // Limit rotation around screen x-axis to keep global North pointing up
    geodetic[1] = min(max(-PI / 2.0, geodetic[1]), PI / 2.0);
    // Avoid accumulation of large values in longitude
    if (geodetic[0] >  PI) geodetic[0] -= 2.0 * PI;
    if (geodetic[0] < -PI) geodetic[0] += 2.0 * PI;

    // Compute ECEF coordinates. NOTE WebGL coordinate convention:
    // +x to right, +y to top of screen, and +z into the screen
    ellipsoid.geodetic2ecef(position, geodetic);

    // Rotation: y first, so it will be left of x operator in final matrix
    // (gl-matrix library 'post-multplies' by each new matrix)
    // Positive angles about Y are towards the +X axis, or East longitude.
    mat4.fromYRotation(rotation, geodetic[0]);
    // Positive angles about X are towards the -Y axis!
    // (from Y to Z, and Z to -Y). But geodetic[1] is a latitude, toward N
    mat4.rotateX(rotation, rotation, -geodetic[1]);

    // The inverse of a rotation matrix is its transpose
    mat4.transpose(inverse, rotation);
  }
}
