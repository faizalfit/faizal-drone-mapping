# Drone LiDAR 3D Mapping

UTS Sistem Robot Otonom - Faizal Fitrohni (5022241019), Teknik Elektro ITS.

github: https://github.com/faizalfit/faizal-drone-mapping

Video proses mapping: https://youtu.be/LP9T2kGMEIU?si=_66DVsWCc8ckNJEV

Video hasil mapping: https://youtu.be/frZXkwX_bZA?si=8PU8pDzxdcZ2fjby

## Persiapan

- Gunakan CoppeliaSim 4.10 dan Python 3.12.14
- Letakkan ketiga file Python dalam satu folder.
- Sertakan **`mapping_viewer_template.html`** di folder yang sama. File ini wajib untuk membuat viewer HTML.
- Jalankan scene Coppelia  `faizal_fitrohni_5022241019.ttt`

Jalankan satu per satu file python atau buka terminal di folder proyek, kemudian jalankan:

## Urutan menjalankan

### 1. Mapping

Buka scene proyek di CoppeliaSim dan pastikan simulasi dalam kondisi **Stop**. Biarkan CoppeliaSim terbuka, lalu jalankan:

```powershell
python 01_mapping.py
```

Program memulai simulasi, menerbangkan drone, dan menyimpan hasil pemetaan beserta jalurnya. Tunggu hingga muncul **Lintasan selesai**, lalu tutup jendela grafik untuk melanjutkan.

Hasil utama: `hasil_mapping/point_cloud_3d_selesai.npz`.

### 2. Voxel downsampling

```powershell
python 02_voxel_downsample.py
```

Program mengurangi jumlah titik menggunakan voxel 5 cm dan menampilkan perbandingan sampel titik. Setelah selesai, tutup jendela grafik.

Hasil utama: `hasil_mapping/point_cloud_voxel_5cm.npz`.

### 3. Viewer HTML

```powershell
python 03_html_viewer.py
```

Program mengelompokkan titik secara geometris dan membuat visualisasi interaktif dengan warna berdasarkan ketinggian. Browser akan terbuka otomatis.

Hasil utama: **`hasil_mapping/mapping_3d_interaktif.html`**. File ini juga bisa dibuka langsung di browser tanpa menjalankan ulang Python.

## Catatan

- Jalankan berurutan: **01 → 02 → 03**. Folder `hasil_mapping` dibuat otomatis.
- Tahap 02 dan 03 tidak memerlukan CoppeliaSim berjalan.
- Jika data voxel sudah tersedia, cukup jalankan tahap 03 untuk membuat ulang HTML.
- Jika scene diubah dan ingin memperbarui peta, ulangi ketiga tahap.
