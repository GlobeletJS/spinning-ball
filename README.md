# spinning ball

![tests](https://github.com/GlobeletJS/spinning-ball/actions/workflows/node.js.yml/badge.svg)

Simulate the position and motion of a camera above the Earth

Camera position above a spherical Earth is represented as a 3-vector of
longitude, latitude, altitude, with associated angular and vertical
velocities. 

Accelerations can be induced by user interaction with the HTML div where
the sphere will be rendered. Handled interactions include:
- Single-touch or mouse-click and drag motions (rotational acceleration)
- Two-touch pinches or scroll wheel rotations (vertical acceleration)

Velocities are damped, with weak damping when there are no active touches
or clicks, to allow coasting. With an active touch/click/zoom, the damping
constant is chosen for critical damping of the relevant induced spring force,
to avoid any oscillation.

Note that the camera and the spherical Earth as modeled by spinning-ball are
both purely conceptual. To display what would be seen by the camera, a separate
renderer is required. See the [d3-world-atlas example][] for a demo with a 
simple D3 renderer.

[d3-world-atlas example]: https://globeletjs.github.io/spinning-ball/examples/d3-world-atlas/index.html

## Initialization
spinningBall.init takes three parameters:
- display: An HTML element where the globe will be represented, and where the
  user's client will generate interaction events
- center: A 2-element array of [longitude, latitude] in degrees, indicating
  the initial horizontal position of the camera
- altitude: A floating point value indicating the initial altitude of the
  camera in kilometers

## API
Initialization returns an object with the following properties and methods:
- radius(): Returns the (floating point) radius of the sphere
- camMoving(): Returns a (Boolean) flag indicating whether the camera is moving
- cameraPos: Pointer to a 3-element array containing the current longitude and
  latitude (in radians) and altitude (in kilometers) of the camera
- edgesPos: Pointer to a list of [longitude, latitude] pairs, where each pair
  represents the location on the globe that would be intersected by a ray shot
  from a (2D) position along the edge of the display element. (These locations
  can be used to determine the geographical extent of the map data required to
  render the visible portion of the globe.)
- lonLatToScreenXY(xy, lonLat): Projects a given [longitude, latitude] pair to
  an [x,y] pair of screen pixel coordinates, representing where that longitude
  and latitude would be rendered on the display element
- cursorPos: Pointer to a 2-element array containing the longitude and latitude
  that would be rendered at the current screen position of the cursor
- isOnScene(): Returns a (Boolean) flag indicating whether a ray shot from the 
  current cursor position would intersect the globe
- cursorChanged(): Returns a (Boolean) flag indicating whether there has been
  any change in the position or status of the cursor relative to the globe
- wasTapped(): Returns a (Boolean) flag indicating whether the globe has been
  tapped or clicked since the last update
- update(time, resized): Updates the position and velocity of the camera,
  taking into account any current velocities, and computing new accelerations
  induced by mouse or touch interactions.
  - Input time (floating point) is the current time in seconds. If using
    requestAnimationFrame, its argument should be multiplied by 0.001
  - Input resized (Boolean) is a flag indicating whether the display element
    has been resized since the last call to update()
  - Return value is a flag indicating whether the display should be
    re-rendered, due to motion of the globe or resizing of the display

## Notes about the code
Some functions include math for an ellipsoid, with different values for the
polar and equatorial radius. However, the two radii MUST be kept equal for now,
until the remaining functions are updated to handle a non-spherical Earth.
