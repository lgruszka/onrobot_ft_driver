# Instalacja driverów OnRobot HEX + 3D Systems Touch
# Ubuntu 16.04 + ROS Kinetic (natywnie)

---

## 1. ROS Kinetic (jeśli jeszcze nie zainstalowany)

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Narzędzia do budowania
sudo apt-get install -y python-rosinstall python-catkin-tools python-wstool build-essential git
```

## 2. Zależności systemowe

```bash
sudo apt-get install -y \
    libboost-all-dev \
    libncurses5-dev \
    freeglut3-dev \
    libdrm-dev \
    libexpat1-dev \
    libglw1-mesa \
    libglw1-mesa-dev \
    libmotif-dev \
    libraw1394-dev \
    libx11-dev \
    libxdamage-dev \
    libxext-dev \
    libxt-dev \
    libxxf86vm-dev \
    libusb-1.0-0-dev
```

## 3. OpenHaptics SDK 3.4

```bash
# Skopiuj openhaptics_3.4-0-developer-edition-amd64.tar na maszynę
cd ~
tar xf openhaptics_3.4-0-developer-edition-amd64.tar

# Zainstaluj headery
sudo cp -r usr/include/HD  /usr/include/
sudo cp -r usr/include/HDU /usr/include/
sudo cp -r usr/include/HL  /usr/include/
sudo cp -r usr/include/HLU /usr/include/

# Zainstaluj biblioteki
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
# Powinno pokazać: libHD.so.3.4.0, libHD.so.3.4, libHD.so.3, libHD.so
```

## 4. Geomagic Touch Device Driver (tylko dla Touch)

Driver systemowy jest osobnym pakietem od 3D Systems. Bez niego Touch się nie zainicjalizuje.

### Pobranie

Oficjalnie: https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34

Jeśli link nie działa, alternatywnie:
https://drive.google.com/drive/folders/1WJY6HpdtGh5zeyASfb4FYJFFG-QGItd6

Szukasz pliku: `geomagic_touch_device_driver_2015.5-26-amd64.tar.gz` (lub nowszy).

### Instalacja

```bash
cd ~
tar xzf geomagic_touch_device_driver_*.tar.gz
cd geomagic_touch_device_driver_*/
sudo ./install
```

Driver instaluje się do `/opt/geomagic_touch_device_driver/`.

### Symlinki (64-bit)

```bash
sudo ln -sf /usr/lib/x86_64-linux-gnu/libraw1394.so.11.0.1 /usr/lib/libraw1394.so.8
sudo ln -sf /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4
```

### Konfiguracja urządzenia

```bash
# Podłącz Touch przez USB
# Nadaj uprawnienia do portu
sudo chmod 666 /dev/ttyACM0

# Uruchom konfigurator
cd /opt/geomagic_touch_device_driver/
sudo ./Geomagic_Touch_Setup

# W oknie:
# 1. Kliknij "Add..." i wpisz nazwę np. "Default Device"
# 2. Wybierz odpowiedni port (Port Num)
# 3. Kliknij "Apply"
# 4. Powinien się wyświetlić numer seryjny urządzenia
```

### Diagnostyka (opcjonalnie)

```bash
cd /opt/geomagic_touch_device_driver/
sudo ./Geomagic_Touch_Diagnostic

# Pozwala:
# - Sprawdzić odczyty enkoderów
# - Skalibrować urządzenie
# - Przetestować siłę zwrotną
```

### Automatyczne uprawnienia USB (udev)

```bash
sudo tee /etc/udev/rules.d/99-touch-haptic.rules << 'EOF'
KERNEL=="ttyACM*", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 5. Catkin workspace i budowanie

```bash
# Utwórz workspace (jeśli nie masz)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Pobierz paczki
git clone https://github.com/lgruszka/onrobot_ft_driver.git
git clone https://github.com/lgruszka/touch_driver.git

# Zbuduj
cd ~/catkin_ws
catkin_make

# Source
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

## 6. Konfiguracja sieci (jeśli osobny komputer od robota)

Na komputerze z driverami:

```bash
# Dodaj do ~/.bashrc
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=DRIVER_PC_IP
```

Na robocie:

```bash
export ROS_MASTER_URI=http://ROBOT_IP:11311
export ROS_IP=ROBOT_IP
```

Oba komputery muszą się nawzajem pingować.

## 7. Uruchamianie

### OnRobot HEX F/T sensor

```bash
# Domyślne (192.168.1.1, 100 Hz, filtr 15 Hz)
roslaunch onrobot_ft_driver ft_sensor.launch

# Custom IP i rate
roslaunch onrobot_ft_driver ft_sensor.launch address:=192.168.100.12 rate:=500 filter:=3
```

### 3D Systems Touch

```bash
# Upewnij się, że port ma uprawnienia
sudo chmod 666 /dev/ttyACM0

# Domyślne
roslaunch touch_driver touch.launch

# Custom nazwa urządzenia (jak skonfigurowałeś w Touch_Setup)
roslaunch touch_driver touch.launch device_name:="My Touch" units:=mm
```

### Oba naraz

```bash
# Terminal 1
roslaunch onrobot_ft_driver ft_sensor.launch

# Terminal 2
roslaunch touch_driver touch.launch
```

## 8. Weryfikacja

```bash
# Lista topiców
rostopic list | grep -E "ft_sensor|phantom"

# Odczyt czujnika siły
rostopic echo /ft_sensor/wrench

# Odczyt pozycji Touch
rostopic echo /phantom/pose

# Odczyt jointów Touch
rostopic echo /phantom/joint_states

# Odczyt przycisków Touch
rostopic echo /phantom/button

# Tara czujnika siły
rosservice call /ft_sensor/zero

# Test force feedback (lekka siła w dół, 0.3N)
rostopic pub -1 /phantom/force_feedback touch_driver/TouchFeedback \
  "{force: {x: 0.0, y: 0.0, z: -0.3}, position: {x: 0, y: 0, z: 0}}"
```

---

## Troubleshooting

### Touch: "Failed to initialize haptic device"
1. Sprawdź czy urządzenie jest widoczne: `ls -la /dev/ttyACM*`
2. Sprawdź uprawnienia: `sudo chmod 666 /dev/ttyACM0`
3. Sprawdź czy Geomagic Touch Device Driver jest zainstalowany: `ls /opt/geomagic_touch_device_driver/`
4. Sprawdź czy urządzenie jest skonfigurowane: uruchom `Geomagic_Touch_Setup`
5. Dla wersji HID: użyj `/dev/hidraw*` zamiast `/dev/ttyACM*`

### Touch: "HD_COMM_CONFIG_ERROR"
- Zły port w konfiguracji. Uruchom `Geomagic_Touch_Setup` i zmień Port Num.

### Touch: buzzing / kicking
- Zmniejsz siłę force feedback
- Sprawdź czy servo loop działa stabilnie: `rostopic hz /phantom/pose` (powinno ~100 Hz)
- Dokument OpenHaptics: rozdział 12 "Troubleshooting"

### OnRobot: brak danych
1. Ping czujnika: `ping 192.168.1.1`
2. Sprawdź port UDP: `sudo ufw allow 49152/udp` (lub wyłącz firewall: `sudo ufw disable`)
3. Sprawdź czy interfejs sieciowy jest w podsieci czujnika:
   `sudo ip addr add 192.168.1.50/24 dev eth0` (jeśli trzeba)

### catkin_make: "Could not find HD/hd.h"
- OpenHaptics nie jest zainstalowany. Powtórz krok 3.
- Sprawdź: `ls /usr/include/HD/hd.h`

### catkin_make: "cannot find -lHD"
- Brak symlinków. Powtórz krok z symlinkami w sekcji 3.
- Sprawdź: `ls -la /usr/lib/libHD*`
