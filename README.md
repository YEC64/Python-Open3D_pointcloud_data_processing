# **Python-Open3D_pointcloud_data_processing**


>"Lidar, son yılların en dikkat çeken teknolojilerinden olan otonom araçlarda sıkça kullanılanılan sensörlerden birisidir. Bulunduğu ortamın belirli bölgesine veya 360 derece olacak şekilde lazer ışınları göndererek uzaklık ölçümü yapılmasını sağlar."

>"Bir bölgenin, bir odanın haritalanmasında veya yeterince veri toplanırsa yapay zeka modelleri eğitilerek nesnelerin tanınmasında kullanılabilir."

>"Bu github reposunda python ile pointcloud şeklinde gelen lidar verilerinin işlenmesi üzerine çalışmalar yaparken karşılaştığım ve faydalı bulduğum bir kütüphaneyi sizlerle paylaşmak ve dökümantasyonda verilen küçük bir örneği sunmak istiyorum."


## **Open3D Kütüphanesinin Kurulumu**

```bash
pip install open3d
```

veya 

```bash
pip install --user open3d 
```

>"Kütüphane dökümantasyonunda bu şekilde pip ile kuulabildiği yazıyor fakat benim denemelerimde kütüphane düzgün bir şekilde kurulmadı. Ben kütüphanenin github reposunu indirip build ederek kurlumunu gerçekleştirdim. Aşağıdaki linkden adımları takip ederek aynı şekilde kurulumları yapabilirsiniz."

[Build from source](http://www.open3d.org/docs/release/compilation.html)


## **ÖRNEK ÇALIŞMA** 
>"Pointcloud verisindeki bir nesneyi ayırt etme. Kod bloklarının çıktılarını altlarına görsel olarak ekleyeceğim"

İlk adım olarak numpy ve open3d kütphanesini import etme
```python
import open3d as o3d 
import numpy as np 
```

Open3d kütüphanesinde test için sunulmuş pointcloud datasını içe aktarma

```python
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
print(pcd)
print(np.asarray(pcd.points))

o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
```

![Screenshot from 2022-02-22 17-02-12](https://user-images.githubusercontent.com/81256525/155185411-a6cc4b98-7e58-4275-8b51-4bfd335f1159.png)



Veriyi daha kolay işlenebilir hale getirmek için noktaların homojen bir alt kümesini alma
```python
downpcd = pcd.voxel_down_sample(voxel_size=0.05)

o3d.visualization.draw_geometries([downpcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
```


![Screenshot from 2022-02-22 17-02-27](https://user-images.githubusercontent.com/81256525/155186138-db8930ae-ef89-4ecb-84f0-236c45741168.png)

Noktaların normal doğrularını çizdirme
```python
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

o3d.visualization.draw_geometries([downpcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
```

![Screenshot from 2022-02-22 17-02-34](https://user-images.githubusercontent.com/81256525/155186612-d534eafa-804a-469e-afe1-1044608db141.png)

Nesnenin nokta bulutundan kırpılması
```python
demo_crop_data = o3d.data.DemoCropPointCloud()
pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
chair = vol.crop_point_cloud(pcd)

o3d.visualization.draw_geometries([chair],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
```

![Screenshot from 2022-02-22 17-02-42](https://user-images.githubusercontent.com/81256525/155187674-8a6844f1-cc5e-445e-9fdc-b514178f0dc7.png)

Nesne üzerinde kalan noktaların ayırt edici olması için renklendirilmesi

```python
chair.paint_uniform_color([1, 0.706, 0])


o3d.visualization.draw_geometries([chair],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
```

![Screenshot from 2022-02-22 17-02-49](https://user-images.githubusercontent.com/81256525/155189784-47c62052-5890-477e-840e-b67b91e82cb6.png)


Nesnedeki noktaların uzaklıklarını bulma
```python
dists = pcd.compute_point_cloud_distance(chair)
dists = np.asarray(dists)
ind = np.where(dists > 0.01)[0]
```

Nesnenin sınırlarının iki eksenli boundingbox içerisine alınması

```python
aabb = chair.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0)
obb = chair.get_oriented_bounding_box()
obb.color = (0, 1, 0)

o3d.visualization.draw_geometries([chair, aabb, obb],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
```

![Screenshot from 2022-02-22 17-02-54](https://user-images.githubusercontent.com/81256525/155189893-1ff59cce-7391-4e65-8979-a399ee30ebf2.png)


Nesnenin noktalarını barındıran en küçük dışbükey yüzeyi tepit etme

```python
pcl = chair
hull, _ = pcl.compute_convex_hull()
hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))


o3d.visualization.draw_geometries([pcl, hull_ls])
```

![Screenshot from 2022-02-22 17-03-19](https://user-images.githubusercontent.com/81256525/155190144-623ee256-3c50-414f-93d3-2ca6fdc1b58c.png)


## SON

>"Merhaba ben YEC, umarım yardımcı olmuştur. Daha fazla incelemek için Open3d kütüphanesi dökümantasyonunu inceleyebilirsiniz. Linkedin hesabımdan takip edebilir ve konu ile ilgili sorularını sunabilirsiniz."

[Linkedin](https://www.linkedin.com/in/yunus-emre-co%C5%9Fkun-84a330202/)
