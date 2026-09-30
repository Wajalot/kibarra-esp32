/* KiBarra-ESP32: bridge Kiprim/OWON <-> MQTT corriendo directamente en el
 * ESP32-S3, sin PC de por medio. Mismo esquema de topics que bridge/service.py
 * (lab/<alias>/state/... y lab/<alias>/cmd/...), mismo alias "kiprim_dc605s",
 * para que sea intercambiable con el bridge de Python desde el punto de vista
 * de la GUI, Grafana, etc.
 */
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "led_strip.h"
#include "cJSON.h"

#include "kiprim_driver.hpp"
#include "perfil.hpp"
#include "secrets.h"

using namespace esp_usb;

static const char *TAG = "kibarra_esp32";

// --- Configuración fija (equivalente a config.yaml) ---
// WIFI_SSID, WIFI_PASS y MQTT_URI viven en secrets.h (no se sube a git).
#define ALIAS     "kiprim_dc605s"
#define LED_GPIO  48  // RGB WS2812 integrado en el ESP32-S3-DevKitC-1

// Límites de seguridad por fuente conectada (no son el máximo físico del
// equipo, son el tope que dejamos mandar por software). Seleccionables desde
// la app vía cmd/config/perfil_fuente, persisten en NVS entre reinicios.
struct PerfilFuente { const char *nombre; float voltage_max; float current_max; };
static const PerfilFuente PERFILES_FUENTE[] = {
    {"dc605s", 15.0f, 5.0f},
    {"dc310s", 30.0f, 10.0f},
};
static const int NUM_PERFILES_FUENTE = sizeof(PERFILES_FUENTE) / sizeof(PERFILES_FUENTE[0]);
static int g_perfil_fuente_idx = 0;

static KiprimDriver *g_kiprim = nullptr;
static esp_mqtt_client_handle_t g_mqtt = nullptr;
static MotorPerfil g_perfil;
static SemaphoreHandle_t g_lock;  // protege el acceso concurrente al puerto USB
static led_strip_handle_t g_led = nullptr;

// Rojo = salida OFF, verde = ON sin perfil, azul = ON con perfil corriendo.
static void actualizar_led(bool output_on, bool perfil_activo)
{
    if (!g_led) return;
    if (!output_on) {
        led_strip_set_pixel(g_led, 0, 20, 0, 0);
    } else if (perfil_activo) {
        led_strip_set_pixel(g_led, 0, 0, 0, 20);
    } else {
        led_strip_set_pixel(g_led, 0, 0, 20, 0);
    }
    led_strip_refresh(g_led);
}

static void iniciar_led(void)
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = LED_GPIO;
    strip_config.max_leds = 1;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &g_led));
    actualizar_led(false, false);
}

static void cargar_perfil_fuente_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("kibarra", NVS_READWRITE, &h) == ESP_OK) {
        int32_t idx = 0;
        if (nvs_get_i32(h, "perfil_idx", &idx) == ESP_OK && idx >= 0 && idx < NUM_PERFILES_FUENTE) {
            g_perfil_fuente_idx = idx;
        }
        nvs_close(h);
    }
}

static void guardar_perfil_fuente_nvs(int idx)
{
    nvs_handle_t h;
    if (nvs_open("kibarra", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "perfil_idx", idx);
        nvs_commit(h);
        nvs_close(h);
    }
}

// --- Perfil personalizado: un único hueco, creado y enviado desde la app en
// JSON (cmd/profile/definir), persiste en NVS entre reinicios. Formato:
// {"pasos":[{"voltage":13.6,"current":0.5,"hasta":{"tipo":"tiempo","valor":60}}, ...]}
static std::vector<PasoPerfil> g_perfil_personalizado;
static bool g_tiene_perfil_personalizado = false;
static const char *NOMBRE_PERSONALIZADO = "personalizado";

static CondicionPaso::Tipo tipo_condicion_desde_str(const char *s)
{
    if (!s) return CondicionPaso::NINGUNA;
    if (strcmp(s, "tiempo") == 0) return CondicionPaso::TIEMPO;
    if (strcmp(s, "corriente_baja") == 0) return CondicionPaso::CORRIENTE_BAJA;
    if (strcmp(s, "corriente_alta") == 0) return CondicionPaso::CORRIENTE_ALTA;
    if (strcmp(s, "voltaje_bajo") == 0) return CondicionPaso::VOLTAJE_BAJO;
    if (strcmp(s, "voltaje_alto") == 0) return CondicionPaso::VOLTAJE_ALTO;
    return CondicionPaso::NINGUNA;
}

static bool cargar_perfil_personalizado_json(const std::string &json)
{
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) return false;
    cJSON *pasos_json = cJSON_GetObjectItem(root, "pasos");
    if (!cJSON_IsArray(pasos_json)) {
        cJSON_Delete(root);
        return false;
    }

    std::vector<PasoPerfil> resultado;
    cJSON *paso_json;
    cJSON_ArrayForEach(paso_json, pasos_json) {
        PasoPerfil p;
        cJSON *v = cJSON_GetObjectItem(paso_json, "voltage");
        if (cJSON_IsNumber(v)) p.voltage = (float)v->valuedouble;
        cJSON *c = cJSON_GetObjectItem(paso_json, "current");
        if (cJSON_IsNumber(c)) p.current = (float)c->valuedouble;
        cJSON *cl = cJSON_GetObjectItem(paso_json, "current_limit");
        if (cJSON_IsNumber(cl)) p.current_limit = (float)cl->valuedouble;
        cJSON *vl = cJSON_GetObjectItem(paso_json, "voltage_limit");
        if (cJSON_IsNumber(vl)) p.voltage_limit = (float)vl->valuedouble;
        cJSON *o = cJSON_GetObjectItem(paso_json, "output");
        if (cJSON_IsBool(o)) p.output = cJSON_IsTrue(o);
        cJSON *hasta = cJSON_GetObjectItem(paso_json, "hasta");
        if (cJSON_IsObject(hasta)) {
            cJSON *tipo = cJSON_GetObjectItem(hasta, "tipo");
            cJSON *valor = cJSON_GetObjectItem(hasta, "valor");
            p.hasta.tipo = tipo_condicion_desde_str(cJSON_IsString(tipo) ? tipo->valuestring : nullptr);
            p.hasta.valor = cJSON_IsNumber(valor) ? (float)valor->valuedouble : 0.0f;
        }
        resultado.push_back(p);
    }
    cJSON_Delete(root);
    if (resultado.empty()) return false;

    g_perfil_personalizado = resultado;
    g_tiene_perfil_personalizado = true;
    return true;
}

static void guardar_perfil_personalizado_nvs(const std::string &json)
{
    nvs_handle_t h;
    if (nvs_open("kibarra", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "perfil_json", json.c_str());
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cargar_perfil_personalizado_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("kibarra", NVS_READWRITE, &h) != ESP_OK) return;
    size_t len = 0;
    if (nvs_get_str(h, "perfil_json", nullptr, &len) == ESP_OK && len > 0) {
        std::string buf(len, '\0');
        if (nvs_get_str(h, "perfil_json", buf.data(), &len) == ESP_OK) {
            cargar_perfil_personalizado_json(buf);
        }
    }
    nvs_close(h);
}

static std::string topic_state(const char *sufijo)
{
    return std::string("lab/") + ALIAS + "/state/" + sufijo;
}

// --- Cambio seguro de consigna: OFF -> fijar -> ON si la salida ya estaba
// encendida. Mismo motivo que en el bridge de Python: en esta familia de
// instrumentos, OCP y current:limit comparten registro, y cambiarlo en
// caliente con la salida encendida puede disparar la protección.
static void cambiar_setpoints(float *voltage, float *current, float *current_limit, float *voltage_limit)
{
    xSemaphoreTake(g_lock, portMAX_DELAY);
    bool estaba_encendida = g_kiprim->output_on();
    if (estaba_encendida) {
        g_kiprim->set_output(false);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    float voltage_max = PERFILES_FUENTE[g_perfil_fuente_idx].voltage_max;
    float current_max = PERFILES_FUENTE[g_perfil_fuente_idx].current_max;
    if (voltage) g_kiprim->set_voltage(fminf(*voltage, voltage_max));
    if (current) g_kiprim->set_current(fminf(*current, current_max));
    if (current_limit) g_kiprim->set_current_limit(fminf(*current_limit, current_max));
    if (voltage_limit) g_kiprim->set_voltage_limit(fminf(*voltage_limit, voltage_max));
    if (estaba_encendida) {
        vTaskDelay(pdMS_TO_TICKS(200));
        g_kiprim->set_output(true);
    }
    xSemaphoreGive(g_lock);
}

static void aplicar_paso(const PasoPerfil &paso)
{
    float v, c, cl, vl;
    float *pv = nullptr, *pc = nullptr, *pcl = nullptr, *pvl = nullptr;
    if (paso.voltage) { v = *paso.voltage; pv = &v; }
    if (paso.current) { c = *paso.current; pc = &c; }
    if (paso.current_limit) { cl = *paso.current_limit; pcl = &cl; }
    if (paso.voltage_limit) { vl = *paso.voltage_limit; pvl = &vl; }
    if (pv || pc || pcl || pvl) {
        cambiar_setpoints(pv, pc, pcl, pvl);
    }
    if (paso.output) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_kiprim->set_output(*paso.output);
        xSemaphoreGive(g_lock);
    }
}

// --- Comandos MQTT entrantes ---
static void manejar_comando(const std::string &sufijo, const std::string &payload)
{
    if (sufijo == "voltage") {
        float v = atof(payload.c_str());
        cambiar_setpoints(&v, nullptr, nullptr, nullptr);
    } else if (sufijo == "current") {
        float c = atof(payload.c_str());
        cambiar_setpoints(nullptr, &c, nullptr, nullptr);
    } else if (sufijo == "current_limit") {
        float cl = atof(payload.c_str());
        cambiar_setpoints(nullptr, nullptr, &cl, nullptr);
    } else if (sufijo == "voltage_limit") {
        float vl = atof(payload.c_str());
        cambiar_setpoints(nullptr, nullptr, nullptr, &vl);
    } else if (sufijo == "output") {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        g_kiprim->set_output(payload == "ON" || payload == "1");
        xSemaphoreGive(g_lock);
    } else if (sufijo == "profile/start") {
        // El payload es el nombre del perfil (ver PERFILES_DISPONIBLES en perfil.hpp,
        // o "personalizado" para el que se haya definido desde la app).
        // Si no coincide con ninguno, se usa el primero como valor por defecto.
        const std::vector<PasoPerfil> *pasos = nullptr;
        const char *nombre = nullptr;
        for (int i = 0; i < NUM_PERFILES_DISPONIBLES; i++) {
            if (payload == PERFILES_DISPONIBLES[i].nombre) {
                pasos = PERFILES_DISPONIBLES[i].pasos;
                nombre = PERFILES_DISPONIBLES[i].nombre;
                break;
            }
        }
        if (!pasos && g_tiene_perfil_personalizado && payload == NOMBRE_PERSONALIZADO) {
            pasos = &g_perfil_personalizado;
            nombre = NOMBRE_PERSONALIZADO;
        }
        if (!pasos) {
            pasos = PERFILES_DISPONIBLES[0].pasos;
            nombre = PERFILES_DISPONIBLES[0].nombre;
        }
        g_perfil.arrancar(pasos, nombre);
        aplicar_paso(*g_perfil.paso_actual());
        ESP_LOGI(TAG, "Perfil arrancado: %s", nombre);
    } else if (sufijo == "profile/stop") {
        g_perfil.parar();
        ESP_LOGI(TAG, "Perfil detenido");
    } else if (sufijo == "profile/definir") {
        if (cargar_perfil_personalizado_json(payload)) {
            guardar_perfil_personalizado_nvs(payload);
            ESP_LOGI(TAG, "Perfil personalizado guardado (%d pasos)", (int)g_perfil_personalizado.size());
        } else {
            ESP_LOGW(TAG, "JSON de perfil personalizado inválido");
        }
    } else if (sufijo == "config/perfil_fuente") {
        for (int i = 0; i < NUM_PERFILES_FUENTE; i++) {
            if (payload == PERFILES_FUENTE[i].nombre) {
                g_perfil_fuente_idx = i;
                guardar_perfil_fuente_nvs(i);
                ESP_LOGI(TAG, "Perfil de fuente -> %s (V_max=%.1f I_max=%.1f)",
                         PERFILES_FUENTE[i].nombre, PERFILES_FUENTE[i].voltage_max, PERFILES_FUENTE[i].current_max);
                break;
            }
        }
    } else {
        ESP_LOGW(TAG, "Comando desconocido: %s", sufijo.c_str());
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED: {
        ESP_LOGI(TAG, "MQTT conectado");
        std::string filtro = std::string("lab/") + ALIAS + "/cmd/#";
        esp_mqtt_client_subscribe(g_mqtt, filtro.c_str(), 0);
        break;
    }
    case MQTT_EVENT_DATA: {
        std::string topic(event->topic, event->topic_len);
        std::string payload(event->data, event->data_len);
        // lab/<alias>/cmd/<sufijo...>
        std::string prefijo = std::string("lab/") + ALIAS + "/cmd/";
        if (topic.rfind(prefijo, 0) == 0) {
            std::string sufijo = topic.substr(prefijo.size());
            manejar_comando(sufijo, payload);
        }
        break;
    }
    default:
        break;
    }
}

static EventGroupHandle_t g_wifi_eventos;
#define WIFI_CONECTADO_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, reintentando...");
        xEventGroupClearBits(g_wifi_eventos, WIFI_CONECTADO_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi conectado, IP obtenida");
        xEventGroupSetBits(g_wifi_eventos, WIFI_CONECTADO_BIT);
    }
}

static void iniciar_wifi(void)
{
    g_wifi_eventos = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void iniciar_mqtt(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_URI;
    g_mqtt = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(g_mqtt, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_mqtt);
}

// --- USB Host: descubrir e inicializar la Kiprim ---
static void manejar_datos_recibidos_cdc(const uint8_t *data, size_t data_len, void *user_arg)
{
    if (g_kiprim) g_kiprim->on_data(data, data_len);
}

static bool manejar_datos_recibidos(const uint8_t *data, size_t data_len, void *user_arg)
{
    manejar_datos_recibidos_cdc(data, data_len, user_arg);
    return true;
}

static void manejar_evento_cdc(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "Error CDC-ACM: %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Kiprim desconectada");
        break;
    default:
        break;
    }
}

static void tarea_eventos_usb(void *arg)
{
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

static KiprimDriver *iniciar_kiprim(void)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskCreate(tarea_eventos_usb, "usb_events", 4096, NULL, 5, NULL);

    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    VCP::register_driver<CH34x>();

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 0,
        .out_buffer_size = 256,
        .in_buffer_size = 256,
        .event_cb = manejar_evento_cdc,
        .data_cb = manejar_datos_recibidos,
        .user_arg = NULL,
    };

    ESP_LOGI(TAG, "Buscando la Kiprim...");
    CdcAcmDevice *dev = VCP::open(&dev_config);
    ESP_LOGI(TAG, "¡Kiprim encontrada!");

    cdc_acm_line_coding_t line_coding = {};
    line_coding.dwDTERate = 115200;
    line_coding.bCharFormat = 0;
    line_coding.bParityType = 0;
    line_coding.bDataBits = 8;
    ESP_ERROR_CHECK(dev->line_coding_set(&line_coding));
    ESP_ERROR_CHECK(dev->set_control_line_state(true, false));
    vTaskDelay(pdMS_TO_TICKS(200));

    return new KiprimDriver(dev);
}

// --- Bucle principal: lee estado, corre el motor de perfil, publica por MQTT ---
static void tarea_tick(void *arg)
{
    char buf[32];
    while (1) {
        xSemaphoreTake(g_lock, portMAX_DELAY);
        bool output = g_kiprim->output_on();
        float voltage_set = g_kiprim->voltage_set();
        float current_set = g_kiprim->current_set();
        float current_limit = g_kiprim->current_limit();
        float voltage_limit = g_kiprim->voltage_limit();
        float voltage_measured = g_kiprim->measure_voltage();
        float current_measured = g_kiprim->measure_current();
        xSemaphoreGive(g_lock);

        actualizar_led(output, g_perfil.activo());

        if (g_perfil.activo() && g_perfil.condicion_cumplida(voltage_measured, current_measured)) {
            g_perfil.avanzar();
            if (g_perfil.activo()) {
                aplicar_paso(*g_perfil.paso_actual());
                ESP_LOGI(TAG, "Perfil '%s' -> paso %d", g_perfil.nombre(), g_perfil.indice());
            } else {
                ESP_LOGI(TAG, "Perfil '%s' terminado", g_perfil.nombre());
            }
        }

        if (g_mqtt) {
            esp_mqtt_client_publish(g_mqtt, topic_state("output").c_str(), output ? "ON" : "OFF", 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", voltage_set);
            esp_mqtt_client_publish(g_mqtt, topic_state("voltage_set").c_str(), buf, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", current_set);
            esp_mqtt_client_publish(g_mqtt, topic_state("current_set").c_str(), buf, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", current_limit);
            esp_mqtt_client_publish(g_mqtt, topic_state("current_limit").c_str(), buf, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", voltage_limit);
            esp_mqtt_client_publish(g_mqtt, topic_state("voltage_limit").c_str(), buf, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", voltage_measured);
            esp_mqtt_client_publish(g_mqtt, topic_state("voltage_measured").c_str(), buf, 0, 0, 0);
            snprintf(buf, sizeof(buf), "%.3f", current_measured);
            esp_mqtt_client_publish(g_mqtt, topic_state("current_measured").c_str(), buf, 0, 0, 0);
            snprintf(buf, sizeof(buf), "%.4f", voltage_measured * current_measured);
            esp_mqtt_client_publish(g_mqtt, topic_state("power_measured").c_str(), buf, 0, 0, 0);
            esp_mqtt_client_publish(g_mqtt, topic_state("profile").c_str(), g_perfil.nombre(), 0, 0, 1);
            snprintf(buf, sizeof(buf), "%d", g_perfil.indice());
            esp_mqtt_client_publish(g_mqtt, topic_state("profile_step").c_str(), buf, 0, 0, 1);

            esp_mqtt_client_publish(g_mqtt, topic_state("perfil_fuente").c_str(),
                                     PERFILES_FUENTE[g_perfil_fuente_idx].nombre, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", PERFILES_FUENTE[g_perfil_fuente_idx].voltage_max);
            esp_mqtt_client_publish(g_mqtt, topic_state("voltage_max").c_str(), buf, 0, 0, 1);
            snprintf(buf, sizeof(buf), "%.3f", PERFILES_FUENTE[g_perfil_fuente_idx].current_max);
            esp_mqtt_client_publish(g_mqtt, topic_state("current_max").c_str(), buf, 0, 0, 1);

            esp_mqtt_client_publish(g_mqtt, topic_state("perfil_personalizado").c_str(),
                                     g_tiene_perfil_personalizado ? "disponible" : "no_definido", 0, 0, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    g_lock = xSemaphoreCreateMutex();
    cargar_perfil_fuente_nvs();
    cargar_perfil_personalizado_nvs();
    iniciar_led();

    g_kiprim = iniciar_kiprim();

    ESP_LOGI(TAG, "Conectando WiFi...");
    iniciar_wifi();
    xEventGroupWaitBits(g_wifi_eventos, WIFI_CONECTADO_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    ESP_LOGI(TAG, "Conectando MQTT...");
    iniciar_mqtt();
    vTaskDelay(pdMS_TO_TICKS(2000));

    xTaskCreate(tarea_tick, "tick", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "KiBarra-ESP32 en marcha, alias=%s", ALIAS);
}
