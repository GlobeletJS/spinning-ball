import { initView } from "yawgl";
import { initEllipsoid } from "./ellipsoid.js";
import { getUnitConversion, checkCoords } from "./coords.js";
import { initBounds } from "./bounds.js";

export function setParams(params) {
  // TODO: Get user-supplied semiMinor & semiMajor axes?
  const ellipsoid = initEllipsoid();

  const { units: userUnits = "degrees" } = params;
  if (!["degrees", "radians"].includes(userUnits)) {
    fail("units must be either degrees or radians");
  }
  const units = getUnitConversion(userUnits);

  const {
    display,
    position = [0.0, 0.0, ellipsoid.meanRadius * 4.0],
    minAltitude: minAlt = ellipsoid.meanRadius() * 0.00001,
    maxAltitude: maxAlt = ellipsoid.meanRadius() * 8.0,
    minLongitude: minLon = -units.maxLon,
    maxLongitude: maxLon = units.maxLon,
    minLatitude: minLat = -units.maxLat,
    maxLatitude: maxLat = units.maxLat,
  } = params;

  if (!(display instanceof Element)) fail("missing display element");

  check(minAlt, 0, ellipsoid.meanRadius() * 100000.0, "minAltitude");
  check(maxAlt, 0, ellipsoid.meanRadius() * 100000.0, "maxAltitude");
  if (minAlt > maxAlt) fail("minAltitude must be <= maxAltitude");

  check(minLon, -units.maxLon, units.maxLon, "minLongitude");
  check(maxLon, -units.maxLon, units.maxLon, "maxLongitude");
  check(minLat, -units.maxLat, units.maxLat, "minLatitude");
  check(maxLat, -units.maxLat, units.maxLat, "maxLatitude");
  if (minLat > maxLat) fail("minLatitude must be <= maxLatitude");

  const b1 = units.convert([minLon, minLat, minAlt]);
  const b2 = units.convert([maxLon, maxLat, maxAlt]);
  const bounds = initBounds(b1, b2);

  if (!checkCoords(position, 3)) fail("position must be an Array of 3 numbers");
  const initialPosition = units.convert(position);
  if (!bounds.check(initialPosition)) fail ("initial position out of range");

  return {
    ellipsoid, display, units, initialPosition, bounds, minAlt, maxAlt,
    view: initView(display, 25.0), // Computes ray params at point on display
  };
}

function check(c, lb, ub, name) {
  if (Number.isFinite(c) && (lb <= c) && (c <= ub)) return true;
  fail(name + " must be a Number between " + lb + " and " + ub);
}

function fail(message) {
  // TODO: Should some errors be RangeErrors or TypeErrors instead?
  throw Error("spinning-ball parameters check: " + message);
}
