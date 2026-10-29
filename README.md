# Drone LiDAR 3D Mapping

Proyek UTS Sistem Robot Otonom — Faizal Fitrohni (5022241019), Teknik Elektro ITS.

Pemetaan 3D dengan drone dan LiDAR di CoppeliaSim, dilanjutkan pengurangan titik menjadi voxel, pewarnaan berdasarkan ketinggian, dan pengelompokan objek tanpa machine learning.

## Persiapan

- Gunakan CoppeliaSim 4.10 dan Python (proyek ini menggunakan environment Conda `sro_env`).
- Letakkan ketiga file Python dalam satu folder.
- Sertakan **`mapping_viewer_template.html`** di folder yang sama. File ini wajib untuk membuat viewer HTML.
- Gunakan scene `.ttt` proyek yang sudah memiliki drone, LiDAR, dan script Lua pengirim data LiDAR.

Buka terminal di folder proyek, kemudian jalankan:

```powershell
conda activate sro_env
python -m pip install numpy matplotlib coppeliasim-zmqremoteapi-client
```

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
- Jika tahap 01 hanya menghasilkan file `parsial`, misi belum selesai. Ulangi mapping hingga berhasil sebelum melanjutkan alur standar.
- Pengelompokan objek tidak mengenali nama benda; objek yang saling menempel dapat tergabung dalam satu kelompok.
