#include "OpenMVCamera.h"

bool OpenMVCamera::ping() {
  _wire.beginTransmission(_addr);
  uint8_t e = _wire.endTransmission();
  return (e == 0);
}

bool OpenMVCamera::readOnce(CameraData& out, uint32_t timeout_ms) {
  // Escreve o comando 0x00 (buffer atual) com repeated-start
  _wire.beginTransmission(_addr);
  _wire.write((uint8_t)0x00);
  if (_wire.endTransmission(false) != 0) { // false = sem STOP (repeated start)
    return false;
  }

  const uint8_t want = 2;
#if ARDUINO_VERSION >= 10610
  _wire.requestFrom((int)_addr, (int)want, (int)true); // STOP = true
#else
  _wire.requestFrom(_addr, want);
#endif

  uint32_t t0 = millis();
  while (_wire.available() < want && (millis() - t0) < timeout_ms) {
    delay(1);
  }
  if (_wire.available() < want) {
    while (_wire.available()) (void)_wire.read(); // flush
    return false;
  }

  out.vitima    = _wire.read();
  out.confianca = _wire.read();
  return true;
}

// ===== API compatível =====
CameraData OpenMVCamera::ler() {
  CameraData data;
  (void)readOnce(data, 20); // tenta uma vez, timeout 20ms
  return data;
}

// ===== API nova =====
bool OpenMVCamera::read(CameraData& out, uint32_t timeout_ms, uint8_t retries) {
  for (uint8_t attempt = 0; attempt <= retries; ++attempt) {
    if (readOnce(out, timeout_ms)) return true;
    delay(1);
  }
  return false;
}
