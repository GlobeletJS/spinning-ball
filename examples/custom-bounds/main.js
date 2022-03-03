import { resizeCanvasToDisplaySize } from "yawgl";
import * as topojson from "topojson-client";
import * as spinningBall from "../../";
import { setupProjection } from "./projection.js";
import { initRenderer } from "./render.js";

export function main() {
  Promise.all([
    getJSON("./50m.json"),
    getJSON("./110m.json"),
  ]).then(start);
}

function getJSON(href) {
  return fetch(href).then(response => response.json())
    .then(world => topojson.feature(world, world.objects.land));
}

function start([land50m, land110m]) {
  const ball = spinningBall.init({
    display: document.getElementById("globe"),
    position: [-170, 62, 11000],
    minLongitude: 160,
    maxLongitude: -130,
    minLatitude: 40,
    maxLatitude: 75,
    minAltitude: 600,
    maxAltitude: 30000,
  });

  const canvas = document.getElementById("globeCanvas");
  resizeCanvasToDisplaySize(canvas);

  const proj = setupProjection(canvas);
  const renderer = initRenderer(canvas, proj.projection);
  let hiRes = false;
  requestAnimationFrame(animate);
 
  function animate(time) {
    const resized = resizeCanvasToDisplaySize(canvas);
    const moving = ball.update(time * 0.001) || resized;

    if (moving) {         // Draw low-res globe
      hiRes = false;
      proj.update(ball);
      renderer.drawLoRes(land110m);
    } else if (!hiRes) {  // Motion stopped. Draw hi-res globe
      proj.update(ball);
      renderer.drawHiRes(land50m);
      hiRes = true;
    }

    requestAnimationFrame(animate);
  }
}
