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
    altitude = 4.0 * ellipsoid.meanRadius(),
  } = params;

  if (!(display instanceof Element)) fail("missing display element");

  if (!["degrees", "radians"].includes(units)) fail("invalid units");

  // Center must be a valid coordinate in the given units
  if (!checkCoords(center, 2)) fail("invalid center array");
  const [cx, cy] = (units === "degrees")
    ? center.slice(0, 2).map(c => c / degrees)
    : center;
  if (cx < -PI || cx > PI || cy < -PI / 2 || cy > PI / 2) {
    fail("center coordinates out of range");
  }

  // Altitude must be a Number, positive and not too big
  if (!Number.isFinite(altitude)) fail("altitude must be a number");
  if (altitude < 0) fail("altitude out of range");

  return {
    ellipsoid, display, units,
    initialPosition: [cx, cy, altitude],
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
