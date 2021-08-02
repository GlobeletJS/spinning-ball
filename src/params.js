import { initView } from "yawgl";
import { initEllipsoid } from "./ellipsoid.js";

export function setParams(params) {
  const { PI } = Math;
  const degrees = 180.0 / PI;

  // TODO: Get user-supplied semiMinor & semiMajor axes?
  const ellipsoid = initEllipsoid();

  const {
    display,
    units = "degrees",
    center = [0.0, 0.0],
    altitude = ellipsoid.meanRadius() * 4.0,
    minHeight = ellipsoid.meanRadius() * 0.00001,
    maxHeight = ellipsoid.meanRadius() * 8.0,
  } = params;

  if (!(display instanceof Element)) fail("missing display element");

  if (!["degrees", "radians"].includes(units)) fail("invalid units");
  const unitConversion = (units === "degrees")
    ? (c) => ([c[0] / degrees, c[1] / degrees, c[2]])
    : (c) => c;

  // Center must be a valid coordinate in the given units
  if (!checkCoords(center, 2)) fail("invalid center array");
  const [cx, cy] = (units === "degrees")
    ? center.slice(0, 2).map(c => c / degrees)
    : center;
  if (cx < -PI || cx > PI || cy < -PI / 2 || cy > PI / 2) {
    fail("center coordinates out of range");
  }

  // Altitude, minHeight, maxHeight must be Numbers, positive and not too big
  const heights = [altitude, minHeight, maxHeight];
  if (!heights.every(h => Number.isFinite(h) && h > 0)) {
    fail("altitude, minHeight, maxHeight must be Numbers > 0");
  } else if (heights.some(h => h > ellipsoid.meanRadius() * 100000.0)) {
    fail("altitude, minHeight, maxHeight must be somewhere below Jupiter");
  }

  return {
    ellipsoid, display, units, unitConversion,
    initialPosition: [cx, cy, altitude],
    minHeight, maxHeight,
    // view object computes ray parameters at points on the display
    view: initView(display, 25.0),
  };
}

function checkCoords(p, n) {
  const isArray = Array.isArray(p) ||
    (ArrayBuffer.isView(p) && !(p instanceof DataView));
  return isArray && p.length >= n &&
    p.slice(0, n).every(Number.isFinite);
}

function fail(message) {
  // TODO: Should some errors be RangeErrors or TypeErrors instead?
  throw Error("spinning-ball: " + message);
}
