import { resizeCanvasToDisplaySize } from 'yawgl';
import { geoSatellite } from 'd3-geo-projection';
import * as d3 from 'd3-geo';
import * as topojson from 'topojson-client';
import * as spinningBall from "../../dist/spinning-ball.bundle.js";

const map50mURL = "https://cdn.jsdelivr.net/npm/world-atlas@1/world/50m.json";
const map110mURL = "https://cdn.jsdelivr.net/npm/world-atlas@1/world/110m.json";

const center = [-95.3656049, 29.7537002]; // Longitude, latitude (degrees)
const altitude = 11000;
const fieldOfView = 25.0;
const degrees = 180 / Math.PI;

export function main() {
  const globeDiv = document.getElementById("globe");

  const canvas = document.getElementById("globeCanvas");
  resizeCanvasToDisplaySize(canvas);
  var numPixelsX = canvas.clientWidth;
  var numPixelsY = canvas.clientHeight;
  const context = canvas.getContext("2d");
  // Save default styles
  context.save();

  const ball = spinningBall.init(globeDiv, center, altitude);

  const projection = geoSatellite()
    .translate([numPixelsX / 2, numPixelsY / 2])
    .precision(0.1);
 
  const path = d3.geoPath(projection, context);

  const graticuleMinor = d3.geoGraticule().step([5,5])();
  const graticuleMajor = d3.geoGraticule().step([15,15])();
  const horizon = {type: "Sphere"};

  // Get map data, start animation when ready
  var land110m, land50m, requestID, hiRes = false;
  var ready110m = fetch(map110mURL)
    .then( response => response.json() )
    .then( world => land110m = topojson.feature(world, world.objects.land) );
  var ready50m = fetch(map50mURL)
    .then( response => response.json() )
    .then( world => land50m = topojson.feature(world, world.objects.land) );

  Promise.all([ready110m, ready50m]).then(startAnimation);
 
  function startAnimation() {
    drawHiRes();
    requestID = requestAnimationFrame(animate);
  }

  function animate(time) {
    time *= 0.001; // Convert milliseconds to seconds
    let moving = ball.update(time);

    // Check for changes in display sizes
    let resized = resizeCanvasToDisplaySize(canvas);
    if (resized) {
      numPixelsX = canvas.clientWidth;
      numPixelsY = canvas.clientHeight;
      projection.translate([numPixelsX / 2, numPixelsY / 2]);
    }

    if (moving) {         // Draw low-res globe
      hiRes = false;
      drawLoRes();
    } else if (!hiRes) {  // Motion stopped. Draw hi-res globe
      drawHiRes();
      hiRes = true;
    }

    requestID = requestAnimationFrame(animate);
  }

  function drawHiRes() {
    renderPrep();
    drawLand(land50m);
    drawGraticule(graticuleMinor, 0.4);
    drawGraticule(graticuleMajor, 0.8);
    drawHorizon();
  }

  function drawLoRes() {
    renderPrep();
    drawLand(land110m);
    drawGraticule(graticuleMajor, 0.4);
    drawHorizon();
  }

  function renderPrep() {
    context.restore();
    context.save();
    context.clearRect(0, 0, numPixelsX, numPixelsY);

    let lon = ball.cameraPos[0] * degrees;
    let lat = ball.cameraPos[1] * degrees;
    let alt = ball.cameraPos[2];
    let snyderP = 1 + alt / ball.radius();
    let visibleYextent = 2 * alt * Math.tan(0.5 * fieldOfView / degrees);
    let scale = ball.radius() * numPixelsY / visibleYextent;

    projection.scale(scale)
      .rotate([-lon, -lat, 0])
      .distance(snyderP)
      .clipAngle(Math.acos(1 / snyderP) * degrees);
  }

  function drawLand(land) {
    context.beginPath();
    path(land);
    context.fill();
  }

  function drawGraticule(graticule, alpha) {
    context.beginPath();
    path(graticule);
    context.strokeStyle = "#aaa";
    context.globalAlpha = alpha;
    context.stroke();
  }

  function drawHorizon() {
    context.beginPath();
    path(horizon);
    context.strokeStyle = "#000";
    //context.globalAlpha = 1;
    context.stroke();
  }
}
