# Instalacja driverów OnRobot HEX + 3D Systems Touch
# Ubuntu 24.04 + Docker (ROS Noetic)

Ubuntu 24.04 nie obsługuje ROS1 natywnie. Używamy Dockera z ROS Noetic.
Komunikacja z robotem (ROS Kinetic) działa po sieci — ROS1 jest kompatybilny
między wersjami (Kinetic ↔ Noetic).

---

## 1. Przygotowanie systemu

```bash
# Aktualizacja
sudo apt update && sudo apt upgrade -y

# Docker
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
# WYLOGUJ SIĘ I ZALOGUJ PONOWNIE (żeby grupa docker zadziałała)

# Narzędzia
sudo apt install -y git curl net-tools
```

## 2. Uprawnienia do urządzeń USB (Touch)

Touch tworzy port /dev/ttyACM0 (lub /dev/hidraw* dla wersji HID).

```bash
# Reguła udev dla Touch — automatyczne uprawnienia przy podłączeniu
sudo tee /etc/udev/rules.d/99-touch-haptic.rules << 'EOF'
# 3D Systems Touch (USB serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", MODE="0666"
# 3D Systems Touch (HID)
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", MODE="0666"
# Fallback — dowolny ttyACM
KERNEL=="ttyACM*", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 3. Pobranie repozytoriów

```bash
mkdir -p ~/ros_drivers && cd ~/ros_drivers

git clone https://github.com/lgruszka/onrobot_ft_driver.git
git clone https://github.com/lgruszka/touch_driver.git
```

## 4. Pobranie OpenHaptics SDK

Skopiuj plik `openhaptics_3.4-0-developer-edition-amd64.tar` do `~/ros_drivers/`.
Jeśli masz go na innym komputerze:

```bash
# Z komputera źródłowego:
scp openhaptics_3.4-0-developer-edition-amd64.tar user@VM_IP:~/ros_drivers/
```

## 5. Dockerfile

```bash
cat > ~/ros_drivers/Dockerfile << 'DOCKERFILE'
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# ============================================================
# Zależności systemowe
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ros-noetic-diagnostic-updater \
    ros-noetic-diagnostic-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-std-srvs \
    ros-noetic-tf \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    libboost-all-dev \
    freeglut3-dev \
    libncurses5-dev \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# OpenHaptics SDK
# ============================================================
COPY openhaptics_3.4-0-developer-edition-amd64.tar /tmp/openhaptics.tar

RUN cd /tmp && tar xf openhaptics.tar \
    && mkdir -p /usr/include /usr/lib \
    # Headers
    && cp -r usr/include/HD  /usr/include/ \
    && cp -r usr/include/HDU /usr/include/ \
    && cp -r usr/include/HL  /usr/include/ \
    && cp -r usr/include/HLU /usr/include/ \
    # Libraries
    && cp usr/lib/libHD.so.3.4.0 /usr/lib/ \
    && cp usr/lib/libHL.so.3.4.0 /usr/lib/ \
    && cp usr/lib/libHDU.a       /usr/lib/ \
    && cp usr/lib/libHLU.a       /usr/lib/ \
    # Symlinks
    && ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4 \
    && ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3 \
    && ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so \
    && ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4 \
    && ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3 \
    && ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so \
    && ldconfig \
    && rm -rf /tmp/openhaptics* /tmp/usr /tmp/opt

# ============================================================
# Catkin workspace
# ============================================================
RUN mkdir -p /catkin_ws/src
COPY onrobot_ft_driver /catkin_ws/src/onrobot_ft_driver
COPY touch_driver       /catkin_ws/src/touch_driver

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# ============================================================
# Entrypoint
# ============================================================
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
source /catkin_ws/devel/setup.bash\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
DOCKERFILE
```

## 6. Budowanie obrazu Docker

```bash
cd ~/ros_drivers
docker build -t ros-haptic-drivers .
```

Budowanie trwa ~5-10 minut za pierwszym razem.

## 7. docker-compose.yml (opcjonalnie, ułatwia uruchamianie)

```bash
cat > ~/ros_drivers/docker-compose.yml << 'YAMLEOF'
services:
  # OnRobot HEX F/T sensor
  onrobot:
    image: ros-haptic-drivers
    container_name: onrobot_ft
    network_mode: host
    environment:
      - ROS_MASTER_URI=http://${ROBOT_IP:-192.168.1.100}:11311
      - ROS_IP=${HOST_IP:-192.168.1.50}
    command: >
      roslaunch onrobot_ft_driver ft_sensor.launch
        address:=${SENSOR_IP:-192.168.1.1}
        rate:=100
        filter:=4
    restart: unless-stopped

  # 3D Systems Touch
  touch:
    image: ros-haptic-drivers
    container_name: touch_haptic
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev
    environment:
      - ROS_MASTER_URI=http://${ROBOT_IP:-192.168.1.100}:11311
      - ROS_IP=${HOST_IP:-192.168.1.50}
    command: >
      roslaunch touch_driver touch.launch
        device_name:="Default Device"
        omni_name:=phantom
        units:=m
    restart: unless-stopped
YAMLEOF
```

## 8. Konfiguracja sieci ROS

Na robocie (ROS Kinetic master):

```bash
# Na robocie — upewnij się, że roscore działa
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=ROBOT_IP
roscore
```

Na VM — ustaw zmienne przed uruchomieniem:

```bash
# Plik .env obok docker-compose.yml
cat > ~/ros_drivers/.env << 'EOF'
ROBOT_IP=192.168.1.100
HOST_IP=192.168.1.50
SENSOR_IP=192.168.1.1
EOF
```

Zmień IP na swoje rzeczywiste adresy.

---

## 9. Uruchamianie

### Opcja A: docker-compose (oba naraz)

```bash
cd ~/ros_drivers
docker compose up -d
```

### Opcja B: ręcznie jeden po drugim

**OnRobot HEX (sam czujnik siły):**

```bash
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://192.168.1.100:11311 \
  -e ROS_IP=192.168.1.50 \
  ros-haptic-drivers \
  roslaunch onrobot_ft_driver ft_sensor.launch address:=192.168.1.1 rate:=100
```

**Touch (joystick haptyczny):**

```bash
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e ROS_MASTER_URI=http://192.168.1.100:11311 \
  -e ROS_IP=192.168.1.50 \
  ros-haptic-drivers \
  roslaunch touch_driver touch.launch
```

### Opcja C: interaktywna (debug)

```bash
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e ROS_MASTER_URI=http://192.168.1.100:11311 \
  -e ROS_IP=192.168.1.50 \
  ros-haptic-drivers \
  bash

# Wewnątrz kontenera:
rostopic list
rostopic echo /ft_sensor/wrench
rostopic echo /phantom/pose
rosservice call /ft_sensor/zero
```

---

## 10. Weryfikacja

Z robota lub z kontenera:

```bash
# Sprawdź czy topic'i są widoczne
rostopic list | grep -E "ft_sensor|phantom"

# Odczyt czujnika siły
rostopic echo /ft_sensor/wrench

# Odczyt pozycji joysticka
rostopic echo /phantom/pose

# Odczyt przycisków
rostopic echo /phantom/button

# Tara czujnika siły
rosservice call /ft_sensor/zero

# Test force feedback (lekka siła w dół)
rostopic pub -1 /phantom/force_feedback touch_driver/TouchFeedback \
  "{force: {x: 0.0, y: 0.0, z: -0.3}, position: {x: 0, y: 0, z: 0}}"
```

---

## 11. Touch — konfiguracja urządzenia (pierwszy raz)

Touch wymaga jednorazowej konfiguracji nazwy urządzenia.
Wewnątrz kontenera:

```bash
docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  ros-haptic-drivers \
  bash

# Sprawdź czy urządzenie jest widoczne
ls -la /dev/ttyACM*
# lub dla HID:
ls -la /dev/hidraw*

# Jeśli masz zainstalowany Touch_Setup/Geomagic_Touch_Setup:
# /opt/geomagic_touch_device_driver/Geomagic_Touch_Setup
# Dodaj urządzenie z nazwą "Default Device"
```

UWAGA: Touch Device Driver (Geomagic) nie jest częścią OpenHaptics SDK.
Trzeba go pobrać osobno ze strony 3D Systems:
https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34

Jeśli driver nie jest dostępny dla Ubuntu 20.04 (Focal), może być konieczne
użycie symlinków do bibliotek — patrz sekcja Troubleshooting.

---

## Troubleshooting

### Touch nie inicjalizuje się: "Failed to initialize haptic device"
- Sprawdź `ls /dev/ttyACM*` — czy urządzenie jest widoczne
- Sprawdź uprawnienia: `chmod 666 /dev/ttyACM0`
- Upewnij się, że kontener ma `--privileged` i `-v /dev:/dev`
- Sprawdź czy Geomagic Touch Device Driver jest zainstalowany

### OnRobot nie odbiera danych
- Sprawdź czy sensor jest w tej samej podsieci: `ping 192.168.1.1`
- Sprawdź firewall: `sudo ufw allow 49152/udp`
- Kontener musi mieć `--network host`

### ROS topics nie są widoczne z robota
- Sprawdź `ROS_MASTER_URI` i `ROS_IP` na obu maszynach
- Ping w obu kierunkach musi działać
- `--network host` jest wymagane w kontenerze

### Brak libHD.so
```bash
# Wewnątrz kontenera
ldconfig -p | grep HD
# Powinno pokazać libHD.so.3.4.0
```
