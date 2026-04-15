# Instalacja driverów OnRobot HEX + 3D Systems Touch
# Ubuntu 22.04 + ROS Noetic (natywnie)

---

## 1. Zależności systemowe

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libncurses5-dev \
    freeglut3-dev \
    libmotif-dev \
    libusb-1.0-0-dev \
    libx11-dev \
    libxext-dev \
    libxt-dev
```

## 2. Zależności ROS

```bash
sudo apt install -y \
    ros-noetic-diagnostic-updater \
    ros-noetic-diagnostic-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-std-srvs \
    ros-noetic-tf \
    ros-noetic-message-generation \
    ros-noetic-message-runtime
```

## 3. OpenHaptics SDK 3.4

```bash
# Skopiuj openhaptics_3.4-0-developer-edition-amd64.tar na maszynę
cd ~
tar xf openhaptics_3.4-0-developer-edition-amd64.tar

# Headery
sudo cp -r usr/include/HD  /usr/include/
sudo cp -r usr/include/HDU /usr/include/
sudo cp -r usr/include/HL  /usr/include/
sudo cp -r usr/include/HLU /usr/include/

# Biblioteki
sudo cp usr/lib/libHD.so.3.4.0 /usr/lib/
sudo cp usr/lib/libHL.so.3.4.0 /usr/lib/
sudo cp usr/lib/libHDU.a       /usr/lib/
sudo cp usr/lib/libHLU.a       /usr/lib/

# Symlinki
sudo ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3.4
sudo ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so.3
sudo ln -sf /usr/lib/libHD.so.3.4.0 /usr/lib/libHD.so
sudo ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3.4
sudo ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so.3
sudo ln -sf /usr/lib/libHL.so.3.4.0 /usr/lib/libHL.so
sudo ldconfig

# Weryfikacja
ldconfig -p | grep libHD
```

## 4. Geomagic Touch Device Driver (tylko dla Touch)

Na Ubuntu 22.04 oficjalny driver z 2015 roku może nie działać bezpośrednio.
Są dwie opcje:

### Opcja A: Driver 2022 (zalecana)

3D Systems wydał nowszą wersję driverów. Pobierz ze strony:
https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34

Szukaj wersji oznaczonej jako kompatybilna z Ubuntu 20.04/22.04.

```bash
# Po pobraniu:
cd ~/
tar xzf TouchDeviceDriver_*.tar.gz  # nazwa pliku może się różnić
cd TouchDeviceDriver_*/
sudo ./install
```

### Opcja B: Skrypt instalacyjny jhu-cisst

Repozytorium JHU ma skrypt automatyzujący instalację na nowszych Ubuntu:

```bash
cd ~/
git clone https://github.com/jhu-cisst-external/3ds-touch-openhaptics.git
cd 3ds-touch-openhaptics
# Przeczytaj README — skrypt pobiera i instaluje odpowiednie drivery
```

### Konfiguracja urządzenia

```bash
# Uprawnienia do portu USB
sudo chmod 666 /dev/ttyACM0

# Uruchom konfigurator (nazwa zależy od wersji drivera)
# Driver 2015:
sudo /opt/geomagic_touch_device_driver/Geomagic_Touch_Setup
# Driver 2022:
sudo Touch_Setup

# Dodaj urządzenie z nazwą "Default Device"
# Wybierz port, kliknij Apply
```

### Reguła udev (automatyczne uprawnienia)

```bash
sudo tee /etc/udev/rules.d/99-touch-haptic.rules << 'EOF'
KERNEL=="ttyACM*", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 5. Budowanie paczek ROS

```bash
# Utwórz workspace (jeśli nie masz)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Pobierz paczki
git clone https://github.com/lgruszka/onrobot_ft_driver.git
git clone https://github.com/lgruszka/touch_driver.git

# Zbuduj
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make

# Source (dodaj do .bashrc)
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

## 6. Konfiguracja sieci ROS (komunikacja z robotem)

```bash
# Dodaj do ~/.bashrc
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=THIS_PC_IP
```

Zamień `ROBOT_IP` i `THIS_PC_IP` na rzeczywiste adresy.
Oba komputery muszą się nawzajem pingować.

## 7. Uruchamianie

### OnRobot HEX F/T sensor

```bash
# Domyślne (192.168.1.1, 100 Hz, filtr 15 Hz)
roslaunch onrobot_ft_driver ft_sensor.launch

# Custom
roslaunch onrobot_ft_driver ft_sensor.launch address:=192.168.100.12 rate:=500 filter:=3
```

### 3D Systems Touch

```bash
roslaunch touch_driver touch.launch

# Custom
roslaunch touch_driver touch.launch device_name:="My Touch" units:=mm publish_rate:=500
```

## 8. Weryfikacja

```bash
# Topiki
rostopic list | grep -E "ft_sensor|phantom"

# Odczyt czujnika siły
rostopic echo /ft_sensor/wrench

# Pozycja Touch
rostopic echo /phantom/pose

# Jointy Touch
rostopic echo /phantom/joint_states

# Przyciski Touch
rostopic echo /phantom/button

# Tara czujnika
rosservice call /ft_sensor/zero

# Test force feedback
rostopic pub -1 /phantom/force_feedback touch_driver/TouchFeedback \
  "{force: {x: 0.0, y: 0.0, z: -0.3}, position: {x: 0, y: 0, z: 0}}"
```

---

## Troubleshooting

### catkin_make: "Could not find HD/hd.h"
```bash
# Sprawdź czy headery są zainstalowane
ls /usr/include/HD/hd.h
# Jeśli brak — powtórz krok 3
```

### catkin_make: "cannot find -lHD"
```bash
# Sprawdź symlinki
ls -la /usr/lib/libHD*
# Jeśli brak — powtórz symlinki z kroku 3
```

### catkin_make: błędy z libHDU (undefined reference)
Na Ubuntu 22.04 libHDU.a mogła być skompilowana ze starszym GCC.
Rozwiązanie — przebuduj HDU ze źródeł (są w tarze):

```bash
cd ~
tar xf openhaptics_3.4-0-developer-edition-amd64.tar
cd opt/OpenHaptics/Developer/3.4-0/libsrc/HDU

# Edytuj Makefile — zmień kompilator jeśli trzeba
make -j$(nproc)

# Nadpisz starą bibliotekę
sudo cp libHDU.a /usr/lib/
```

### Touch: "Failed to initialize haptic device"
1. `ls /dev/ttyACM*` — czy urządzenie widoczne?
2. `sudo chmod 666 /dev/ttyACM0`
3. Czy Geomagic Touch Device Driver zainstalowany?
4. Czy urządzenie skonfigurowane w Touch_Setup?

### Touch: libPHANToMIO.so not found
```bash
# Znajdź gdzie jest zainstalowana
sudo find / -name "libPHANToMIO*" 2>/dev/null
# Zrób symlink do /usr/lib/
sudo ln -sf /scieżka/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
sudo ldconfig
```

### OnRobot: brak danych / timeout
```bash
# Ping czujnika
ping 192.168.1.1

# Sprawdź firewall
sudo ufw status
sudo ufw allow 49152/udp

# Sprawdź czy interfejs sieciowy jest w odpowiedniej podsieci
ip addr show
# Jeśli trzeba dodać adres:
sudo ip addr add 192.168.1.50/24 dev eno1
```

### OpenHaptics: GLIBC version mismatch
Biblioteki z tara 3.4 są skompilowane pod starsze GLIBC. Na Ubuntu 22.04
zazwyczaj jest kompatybilność wsteczna i działa. Jeśli nie — użyj
nowszych bibliotek z driverów 2022 od 3D Systems.
