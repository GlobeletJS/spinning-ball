import { initView } from "yawgl";
import { initEllipsoid } from "./ellipsoid.js";
import { checkCoords, getUnitConversion } from "./coords.js";

export function setParams(params) {
  const { PI } = Math;

  // TODO: Get user-supplied semiMinor & semiMajor axes?
  const ellipsoid = initEllipsoid();

  const {
    display,
    units: userUnits = "degrees",
    position = [0.0, 0.0, ellipsoid.meanRadius * 4.0],
    minHeight = ellipsoid.meanRadius() * 0.00001,
    maxHeight = ellipsoid.meanRadius() * 8.0,
  } = params;

  if (!(display instanceof Element)) fail("missing display element");

  if (!["degrees", "radians"].includes(userUnits)) fail("invalid units");
  const units = getUnitConversion(userUnits);

  // minHeight, maxHeight must be Numbers, positive and not too big
  const heights = [minHeight, maxHeight];
  if (!heights.every(h => Number.isFinite(h) && h > 0)) {
    fail("minHeight, maxHeight must be Numbers > 0");
  } else if (heights.some(h => h > ellipsoid.meanRadius() * 100000.0)) {
    fail("minHeight, maxHeight must be somewhere below Jupiter");
  }

  // initialPosition must be a valid coordinate in the given units
  if (!checkCoords(position, 3)) fail("invalid center array");
  const initialPosition = units.convert(position);
  const [lon, lat, alt] = initialPosition;
  const outOfRange =
    lon < -PI || lon > PI ||
    lat < -PI / 2 || lat > PI / 2 ||
    alt < minHeight || alt > maxHeight;
  if (outOfRange) fail ("initial position out of range");

  return {
    ellipsoid, display, units, initialPosition, minHeight, maxHeight,
    view: initView(display, 25.0), // Computes ray params at point on display
  };
}

function fail(message) {
  // TODO: Should some errors be RangeErrors or TypeErrors instead?
  throw Error("spinning-ball: " + message);
}
