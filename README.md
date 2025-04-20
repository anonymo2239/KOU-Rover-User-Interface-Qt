# KOU Rover User Interface System / KOU Rover Kullanıcı Arayüz Sistemi
---
Bu proje, KOU Rover takımı için geliştirilmiş görsel arayüz tabanlı bir kontrol panelidir. Arayüz, ROS2 altyapısı ile entegre şekilde çalışır ve robotun görev süreci boyunca durumu, yönlendirme bilgisi ve çeşitli sensör verilerini kullanıcıya sunar. Sistem ayrıca QR kod okuma, yük algılama ve acil durum yönetimi gibi gelişmiş özelliklere de sahiptir.

## 🚀 Özellikler

- **ROS 2 Entegrasyonu:** `/diff_cont/odom`, `scene_gui`, `gui_start`, `qr_code`, `agirlik_data` gibi ROS topic’leri üzerinden haberleşme.
- **Tam Otomasyon Senaryoları:** 6 farklı senaryo üzerinden görev başlatma ve takip.
- **Gerçek Zamanlı Görselleştirme:**
  - QR kod haritası güncellenmesi
  - Lidar haritalama görüntüsü (SLAM benzeri yapı)
- **Uzaktan Kontrol (Turtle):** WASD tuşları ya da butonlar ile yönlendirme.
- **Acil Durum Yönetimi:** Arayüz üzerinden acil durdurma ve iptal sistemi.
- **Görev Takibi:** Görev süresi, sıcaklık, hız, yük ve batarya seviyesi gibi değerlerin canlı takibi.

## 📷 Arayüz Görselleri

Ana ekran:
![Ana Panel](./images/sample_ui_1.png)

Yönlendirme ekranı:
![Kontrolcü Paneli](./images/sample_ui_2.png)

QR kod haritası:
![QR Harita](./images/sample_ui_3.png)

## 🛠️ Kurulum

### Gereksinimler

- Ubuntu 20.04+
- Python 3.8+
- ROS 2 (Foxy ya da Humble önerilir)
- `PyQt6`
- `fontconfig`

### Yükleme

```bash
sudo apt-get install python3-pyqt6 fontconfig
sudo fc-cache -f -v
```

Ardından proje klasöründe gerekli Python bağımlılıklarını yükleyin (örneğin):

```bash
pip install -r requirements.txt
```

> **QR Kod Okuyucu için:**  
> `device = evdev.InputDevice("/dev/input/event24")` satırında uygun `event` ID'si girilmeli ve gerekli izinler (`chmod`, `udev` kuralı) verilmelidir.

## ▶️ Çalıştırma

```bash
python3 main_rovergui_2_0.py
```

Ayrıca ROS 2 düğümlerinizin çalışıyor olması gerekmektedir.

## 📂 Klasör Yapısı

- `main_rovergui_2_0.py`: Ana GUI çalıştırıcısı
- `ros2_nodes/`: ROS publisher/subscriber düğümleri
- `images/`: Arayüz ikonları ve QR kod harita görselleri
- `web_map.py`: Haritalama modülü
- `guiros.py`: Alternatif arayüz (test amaçlı)
- `old_versions/`: Önceki sürümler

## 👨‍💻 Geliştirici Notları

- `rovergui_2_0.py` dosyası içinde QR kod eşleşmeleri, buton işlevleri, ROS publisher'lar ve bağlantı kontrol mekanizması tanımlıdır.
- PyQt sinyalleri thread-safe çalışacak şekilde `pyqtSignal(..., Qt.ConnectionType.QueuedConnection)` şeklinde bağlanmıştır.

## ✨ Ekran Renkleri

- Yeşil: Bağlantı aktif, görevde
- Kırmızı: Acil durum aktif
- Gri: Bekleme modu

## 📜 Lisans

Bu proje özel bir yarışma için geliştirilmiştir. Açık kaynak değildir.
```

---

Görsellerin klasörde `images/` altında `sample_ui_1.png`, `sample_ui_2.png`, `sample_ui_3.png` gibi isimlerle yer aldığını varsaydım. İstersen görsel adlarını seninkilerle güncellerim. Dilersen `.md` dosyasını direkt verebilirim.
