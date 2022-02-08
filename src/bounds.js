export function wrapLongitude(lon) {
  const { floor, PI } = Math;
  const period = floor((lon + PI) / (2 * PI));
  return lon - period * 2 * PI;
}

export function initBounds([minLon, minLat, minAlt], [maxLon, maxLat, maxAlt]) {
  const { min, max, PI } = Math;

  const hWidth = (minLon < maxLon)
    ? (maxLon - minLon) / 2
    : (maxLon - minLon) / 2 + PI;

  const centerLon = minLon + hWidth;

  return { check, apply };

  function check([lon, lat, alt]) {
    const dLon = wrapLongitude(lon - centerLon);
    if (dLon < -hWidth || hWidth < dLon) return false;
    if (lat < minLat || maxLat < lat) return false;
    if (alt < minAlt || maxAlt < alt) return false;
    return true;
  }

  function apply([lon, lat, alt]) {
    const dLon = wrapLongitude(lon - centerLon);
    const limdlon = min(max(-hWidth, dLon), hWidth);
    const clipLon = wrapLongitude(centerLon + limdlon);

    const clipLat = min(max(minLat, lat), maxLat);
    const clipAlt = min(max(minAlt, alt), maxAlt);

    return [clipLon, clipLat, clipAlt];
  }
}
