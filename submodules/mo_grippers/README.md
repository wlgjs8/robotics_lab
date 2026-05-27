# mo_grippers

## Build

Dependencies

```bash
sudo apt install libmodbus-dev
```

Installation

```bash
git clone https://github.com/PLAIF-dev/mo_grippers.git
cd mo_grippers
mkdir build
cd build
cmake ..
make -j
sudo make install
```

Uninstallation

```bash
cd mo_grippers/build
sudo make uninstall
```
