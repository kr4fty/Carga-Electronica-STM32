# SIMULACIÓN

<img align="center" src="https://i.ibb.co/wKb7L5S/PICSIMLAB.png" alt="PICSIMLAB" width="720">


### PASOS para realizar la simulación

1. Instalar [PICSIMLAB](https://github.com/lcgamboa/picsimlab)
2. Abre el archivo `Carga.pzw` con PICSIMLAB; este se ejecutará automáticamente.
3. Si deseas cargar un archivo BIN, asegúrate de seleccionar el entorno **generic_qemu** dentro de `platformio.ini` y luego compilarlo:


```
[platformio]
;default_envs = nanoatmega328
;default_envs = wroom
;default_envs = generic_dfu
;default_envs = generic_hid_dapboot
;default_envs = generic_hid
;default_envs = generic_serial
;default_envs = generic_stlinkv2
default_envs = generic_qemu         ; <----- DESCOMENTAR!!!

...

[env:generic_qemu] ; Usando qemu, https://lcgamboa.github.io/picsimlab_docs/0.9.2/armgdbDebug.html#x50-490006.3.1
platform = ststm32
board = bluepill_f103c8_128k
board_build.mcu = stm32f103c8t6
;board_build.core = maple
framework = arduino

upload_protocol = custom
...
```

Por ultimo, vamos a importar el BIN generado (`firmware.bin`)

```
File -> Load Bin
```

Y nos dirigimos a la dirección:

```
.pio/build/generic_qemu/firmware.bin
```
