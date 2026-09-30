#pragma once
#include <optional>
#include <vector>
#include "esp_timer.h"

// Máquina de estados de perfiles, equivalente a bridge/profile.py.
// Sin YAML en el micro: los perfiles se definen como datos C++ aquí mismo.

struct CondicionPaso {
    enum Tipo { NINGUNA, TIEMPO, CORRIENTE_BAJA, CORRIENTE_ALTA, VOLTAJE_BAJO, VOLTAJE_ALTO } tipo = NINGUNA;
    float valor = 0;
};

struct PasoPerfil {
    std::optional<float> voltage;
    std::optional<float> current;
    std::optional<float> current_limit;
    std::optional<float> voltage_limit;
    std::optional<bool> output;
    CondicionPaso hasta;
};

// Réplica exacta de profiles/carga_csb_agm.yaml (bridge Python).
static const std::vector<PasoPerfil> PERFIL_CARGA_CSB_AGM = {
    {14.5f, 0.8f, 1.2f, 14.9f, true, {CondicionPaso::TIEMPO, 15.0f}},
    {13.9f, 0.6f, 0.9f, 14.7f, true, {CondicionPaso::TIEMPO, 15.0f}},
    {std::nullopt, std::nullopt, std::nullopt, std::nullopt, false, {CondicionPaso::NINGUNA, 0}},
};

// Réplica exacta de profiles/Carga_Batería_SAI.yaml (bridge Python).
static const std::vector<PasoPerfil> PERFIL_CARGA_SAI = {
    {14.4f, 2.0f, 2.5f, 15.0f, true, {CondicionPaso::VOLTAJE_BAJO, 14.0f}},
    {13.6f, 1.0f, 1.5f, 15.0f, true, {CondicionPaso::TIEMPO, 1800.0f}},
    {std::nullopt, std::nullopt, std::nullopt, std::nullopt, false, {CondicionPaso::NINGUNA, 0}},
};

struct RegistroPerfil { const char *nombre; const std::vector<PasoPerfil> *pasos; };
static const RegistroPerfil PERFILES_DISPONIBLES[] = {
    {"carga_csb_agm", &PERFIL_CARGA_CSB_AGM},
    {"carga_sai", &PERFIL_CARGA_SAI},
};
static const int NUM_PERFILES_DISPONIBLES = sizeof(PERFILES_DISPONIBLES) / sizeof(PERFILES_DISPONIBLES[0]);

class MotorPerfil {
public:
    void arrancar(const std::vector<PasoPerfil> *perfil, const char *nombre)
    {
        perfil_ = perfil;
        nombre_ = nombre;
        indice_ = 0;
        inicio_paso_ms_ = esp_timer_get_time() / 1000;
    }

    void parar()
    {
        perfil_ = nullptr;
        nombre_ = "ninguno";
    }

    bool activo() const { return perfil_ != nullptr && indice_ < perfil_->size(); }
    const char *nombre() const { return nombre_; }
    int indice() const { return activo() ? (int)indice_ : -1; }

    const PasoPerfil *paso_actual() const
    {
        if (!activo()) return nullptr;
        return &(*perfil_)[indice_];
    }

    bool condicion_cumplida(float v_medida, float i_medida) const
    {
        const PasoPerfil *p = paso_actual();
        if (!p) return false;
        const CondicionPaso &c = p->hasta;
        int64_t ahora_ms = esp_timer_get_time() / 1000;
        switch (c.tipo) {
        case CondicionPaso::NINGUNA:         return true;
        case CondicionPaso::TIEMPO:          return (ahora_ms - inicio_paso_ms_) >= (int64_t)(c.valor * 1000);
        case CondicionPaso::CORRIENTE_BAJA:  return i_medida < c.valor;
        case CondicionPaso::CORRIENTE_ALTA:  return i_medida > c.valor;
        case CondicionPaso::VOLTAJE_BAJO:    return v_medida < c.valor;
        case CondicionPaso::VOLTAJE_ALTO:    return v_medida > c.valor;
        }
        return false;
    }

    void avanzar()
    {
        indice_++;
        inicio_paso_ms_ = esp_timer_get_time() / 1000;
    }

private:
    const std::vector<PasoPerfil> *perfil_ = nullptr;
    const char *nombre_ = "ninguno";
    size_t indice_ = 0;
    int64_t inicio_paso_ms_ = 0;
};
