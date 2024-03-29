export function getUnitConversion(units) {
  // Internally, spinning-ball assumes geodetic coordinates in these units:
  //   [longitude (radians), latitude (radians), altitude (kilometers)]
  // Externally, the user may want longitude and latitude in degrees.
  // Construct the functions that convert user inputs to internal coordinates,
  // and invert internal coordinates to the user's units
  const maxLon = (units === "degrees") ? 180.0 : Math.PI;
  const maxLat = maxLon / 2;
  const uPerRad = maxLon / Math.PI;

  return {
    maxLon, maxLat,
    convert: (c) => new Float64Array([c[0] / uPerRad, c[1] / uPerRad, c[2]]),
    invert: (c) => new Float64Array([c[0] * uPerRad, c[1] * uPerRad, c[2]]),
  };
}

export function checkCoords(p, n) {
  const isArray = Array.isArray(p) ||
    (ArrayBuffer.isView(p) && !(p instanceof DataView));
  return isArray && p.length >= n &&
    p.slice(0, n).every(Number.isFinite);
}
