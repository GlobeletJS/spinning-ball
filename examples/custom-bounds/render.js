import * as d3 from 'd3-geo';

export function initRenderer(canvas, projection) {
  const context = canvas.getContext("2d");
  context.save(); // Save default styles

  const path = d3.geoPath(projection, context);

  const graticuleMinor = d3.geoGraticule().step([5,5])();
  const graticuleMajor = d3.geoGraticule().step([15,15])();
  const horizon = { type: "Sphere" };

  return { drawHiRes, drawLoRes };

  function drawHiRes(land) {
    renderPrep();
    draw(land, "fill");
    draw(graticuleMinor, "stroke", { strokeStyle: "#aaa", globalAlpha: 0.4 });
    draw(graticuleMajor, "stroke", { strokeStyle: "#aaa", globalAlpha: 0.8 });
    draw(horizon, "stroke", { strokeStyle: "#000" });
  }

  function drawLoRes(land) {
    renderPrep();
    draw(land, "fill");
    draw(graticuleMajor, "stroke", { strokeStyle: "#aaa", globalAlpha: 0.4 });
    draw(horizon, "stroke", { strokeStyle: "#000" });
  }

  function renderPrep() {
    context.restore();
    context.save();
    context.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);
  }

  function draw(data, method, styles = {}) {
    context.beginPath();
    path(data);
    Object.assign(context, styles);
    context[method]();
  }
}
