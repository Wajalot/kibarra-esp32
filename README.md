# KiBarra-ESP32

Bridge Kiprim/OWON ↔ MQTT corriendo **directamente en un ESP32-S3**, sin PC de por medio. Hermano del bridge Python de [KiBarra](https://github.com/Wajalot/lab_Kiprim): habla el mismo protocolo serie con el instrumento y el mismo esquema de topics MQTT, así que la [GUI de escritorio](https://github.com/Wajalot/lab_Kiprim) y la [app Android](https://github.com/Wajalot/kibarra-android) funcionan contra cualquiera de los dos sin cambiar nada.

## Por qué existe

El bridge original necesita un PC encendido con el puerto serie del instrumento conectado. Este firmware hace exactamente lo mismo pero en un ESP32-S3 usando su **USB OTG nativo en modo Host**: el propio microcontrolador es quien habla por USB con la Kiprim/OWON, se conecta a tu WiFi y publica/escucha MQTT. El ESP32 se alimenta del puerto auxiliar "5V 1A" de la propia fuente — el conjunto queda totalmente autónomo, sin ordenador.

## Hardware

- **ESP32-S3-DevKitC-1** (o clon compatible, N8R8/N16R8). Los clones necesitan a veces soldar un puente/diodo "USB-OTG" en la parte trasera para que el VBUS funcione correctamente en modo Host — los originales de Espressif ya lo traen habilitado.
- Cable USB del puerto **OTG** (no el de flasheo/COM) hacia el puerto de control de la Kiprim/OWON (chip CH340 internamente).
- Alimentación del ESP32 por su puerto COM/UART, o por cualquier salida 5V auxiliar del propio instrumento si la tiene (independiente del canal programable, para que no se corte al cambiar consignas).

## Qué hace

- **Driver CH34x sobre USB Host** (`usb_host_vcp` + `usb_host_ch34x_vcp`): mismo protocolo serie que el bridge Python (115200 8N1, comandos tipo SCPI).
- **Mismo esquema MQTT**: `lab/<alias>/state/...` y `lab/<alias>/cmd/...` — intercambiable con el bridge Python desde el punto de vista de cualquier cliente.
- **Cambio seguro de consigna**: ciclo OFF → fijar → ON si la salida ya estaba encendida (en esta familia de instrumentos, OCP y `current:limit` comparten registro).
- **Perfiles de carga embebidos** (carga AGM, carga SAI) más un hueco de **perfil personalizado** definido en JSON desde la app Android, persistido en NVS.
- **Selector de fuente conectada**: límites de V/I según el modelo (DC605S, DC310S), configurable por MQTT y persistente entre reinicios.
- **LED RGB de estado** (el WS2812 integrado en la placa): rojo = salida OFF, verde = ON, azul = ON con un perfil corriendo.

## Compilar y flashear

Requiere [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v5.3+.

```bash
cp main/secrets.h.example main/secrets.h   # rellena WIFI_SSID, WIFI_PASS y MQTT_URI reales
source <ruta-a-esp-idf>/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash   # el puerto puede variar; el ESP32-S3 suele NO ser el CH340 clásico
```

`secrets.h` está en `.gitignore` — nunca se sube al repo.

## Arquitectura

```
Kiprim/OWON (USB-B) ── ESP32-S3 (USB Host + WiFi) ── MQTT ── GUI PySide6 / app Android
```

El broker MQTT es solo un intermediario de mensajes; quien de verdad traduce a comandos serie es el ESP32. Cualquier cliente MQTT (la GUI de escritorio, la app Android, un dashboard de Grafana) puede conectarse sin tocar el firmware.

## Limitaciones conocidas

- Solo un hueco de "perfil personalizado" a la vez (se sobreescribe al definir uno nuevo).
- El selector de fuente conectada es manual — no hay detección automática por `*idn?` como en el bridge Python.
