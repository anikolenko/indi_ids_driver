# INDI Driver for IDS uEye Cameras (CMOS)

This is an open-source INDI driver for **IDS Imaging uEye industrial cameras**, allowing them to be used as imaging cameras in astronomical software via the INDI protocol.

The driver integrates IDS cameras into the INDI ecosystem and supports basic imaging operations.

---

## 📷 Supported Hardware

- IDS Imaging **uEye USB cameras**
- Tested with:
  - IDS UI-3000SE (12 MP Color)
- Other uEye models may work but are **not guaranteed**.

---

## ✨ Features

- Single exposure capture (via Software Trigger mode)
- Long Exposure mode (if available)
- Frame size control (ROI) - buggy and not all dimensions available, 2000x2000, 1000x1000 - seems ok)
- Gain / exposure handling
- Sensor temperature readout

- Binning control - not yet imlemented 
---

## 🧰 Requirements

- Linux system
- INDI Library (libindi)
- IDS uEye SDK for Linux (ueye.h, libueye_api.so)

Tested and working on:

Linux Ubuntu 22.04
IDS Software Suite 4.96

Install dependencies:

```bash
sudo apt install libindi-dev cmake build-essential
sudo dpkg -i ueye-api_4.96.1.2054_amd64.deb
sudo dpkg -i ueye-dev_4.96.1.2054_amd64.deb

