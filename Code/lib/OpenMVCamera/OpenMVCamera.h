#pragma once
#include <Arduino.h>
#include <Wire.h>

struct CameraData {
  uint8_t vitima = 0;
  uint8_t confianca = 0;
};

class OpenMVCamera {
public:
  // Construtor compatível (usa Wire por padrão)
  explicit OpenMVCamera(uint8_t i2c_address)
  : _wire(Wire), _addr(i2c_address) {}

  // Construtor opcional se quiser usar Wire1, etc.
  OpenMVCamera(TwoWire& bus, uint8_t i2c_address)
  : _wire(bus), _addr(i2c_address) {}

  // Opcional: iniciar barramento e ajustar clock (chame no setup, se quiser)
  bool begin(uint32_t i2c_clock_hz = 400000) {
    _wire.begin();           // se já estiver iniciado, não tem problema
    _wire.setClock(i2c_clock_hz);
    return ping();
  }

  // ===== API antiga (compatível) =====
  // Lê 2 bytes [vitima, confianca]; se falhar, retorna zeros.
  CameraData ler();

  // ===== API nova (opcional) =====
  // Lê com timeout e tenta 'retries' vezes (default 20ms/1 tentativa)
  bool read(CameraData& out, uint32_t timeout_ms = 20, uint8_t retries = 1);
  CameraData read(uint32_t timeout_ms, uint8_t retries = 1) {
    CameraData d; (void)read(d, timeout_ms, retries); return d;
  }

  // Checa ACK do dispositivo
  bool ping();

  // Utilidades
  uint8_t address() const { return _addr; }
  void setAddress(uint8_t new_addr) { _addr = new_addr; }

private:
  TwoWire& _wire;
  uint8_t  _addr;

  // leitura interna
  bool readOnce(CameraData& out, uint32_t timeout_ms);
};
