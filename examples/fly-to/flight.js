export function initFlightButton(ball) {
  const lon = document.getElementById("longitude");
  const lat = document.getElementById("latitude");
  const alt = document.getElementById("altitude");

  document.getElementById("fly").addEventListener("click", startFlight);

  function startFlight() {
    ball.flyTo([lon.value, lat.value, alt.value]);
  }
}
