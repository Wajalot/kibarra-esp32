#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp.hpp"

// Driver para la familia Kiprim/OWON sobre USB Host (via driver CH34x).
// Mismo protocolo que bridge/instrument.py: comandos de texto terminados
// en \n, respuestas ASCII, 115200 8N1.
// CdcAcmDevice vive en el namespace global (no en esp_usb), a pesar de que
// vcp_ch34x.hpp lo usa desde dentro de namespace esp_usb.
class KiprimDriver {
public:
    explicit KiprimDriver(CdcAcmDevice *dev) : dev_(dev)
    {
        respuesta_lista_ = xSemaphoreCreateBinary();
    }

    // Se llama desde el callback de datos del CDC-ACM (ver main.cpp).
    void on_data(const uint8_t *data, size_t len)
    {
        size_t n = len < sizeof(buffer_) - 1 ? len : sizeof(buffer_) - 1;
        memcpy(buffer_, data, n);
        buffer_[n] = '\0';
        // Quita \r\n final si lo hay
        while (n > 0 && (buffer_[n - 1] == '\n' || buffer_[n - 1] == '\r')) {
            buffer_[--n] = '\0';
        }
        xSemaphoreGive(respuesta_lista_);
    }

    std::string query(const char *cmd, uint32_t timeout_ms = 1000)
    {
        xSemaphoreTake(respuesta_lista_, 0);  // limpia por si quedó algo pendiente
        std::string linea = std::string(cmd) + "\n";
        dev_->tx_blocking((uint8_t *)linea.data(), linea.size());
        if (xSemaphoreTake(respuesta_lista_, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            return std::string(buffer_);
        }
        return "";
    }

    void write_cmd(const char *cmd)
    {
        std::string linea = std::string(cmd) + "\n";
        dev_->tx_blocking((uint8_t *)linea.data(), linea.size());
    }

    bool output_on() { return query("output?") == "ON"; }
    float voltage_set() { return atof(query("voltage?").c_str()); }
    float current_set() { return atof(query("current?").c_str()); }
    float current_limit() { return atof(query("current:limit?").c_str()); }
    float voltage_limit() { return atof(query("voltage:limit?").c_str()); }
    float measure_voltage() { return atof(query("measure:voltage?").c_str()); }
    float measure_current() { return atof(query("measure:current?").c_str()); }

    void set_voltage(float v)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "voltage %.3f", v);
        write_cmd(buf);
    }
    void set_current(float i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "current %.3f", i);
        write_cmd(buf);
    }
    void set_current_limit(float i)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "current:limit %.3f", i);
        write_cmd(buf);
    }
    void set_voltage_limit(float v)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "voltage:limit %.3f", v);
        write_cmd(buf);
    }
    void set_output(bool on) { write_cmd(on ? "output 1" : "output 0"); }

private:
    CdcAcmDevice *dev_;
    SemaphoreHandle_t respuesta_lista_;
    char buffer_[128];
};
