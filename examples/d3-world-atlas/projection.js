import { geoSatellite } from "d3-geo-projection";

const fieldOfView = 25.0;
const degrees = 180 / Math.PI;

export function setupProjection(canvas) {
  const projection = geoSatellite()
    .translate([canvas.clientWidth / 2, canvas.clientHeight / 2])
    .precision(0.1);

  function update(ball) {
    const [lon, lat, alt] = ball.cameraPos();
    const snyderP = 1 + alt / ball.radius();
    const visibleYextent = 2 * alt * Math.tan(0.5 * fieldOfView / degrees);
    const scale = ball.radius() * canvas.clientHeight / visibleYextent;

    projection
      .translate([canvas.clientWidth / 2, canvas.clientHeight / 2])
      .scale(scale)
      .rotate([-lon, -lat, 0])
      .distance(snyderP)
      .clipAngle(Math.acos(1 / snyderP) * degrees);
  }

  return { projection, update };
}
