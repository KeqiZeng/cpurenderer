A simple CPU-based rasterization renderer implemented from scratch as a part of my computer graphics learning journey,

# Requirements

[`stb_image` and `stb_image_write`](https://github.com/nothings/stb) for image read and write.

[`eigen`](https://eigen.tuxfamily.org/index.php?title=Main_Page) for vector and matrix computation.

[`argparse`](https://github.com/p-ranav/argparse) for command-line arguments parse.

All of them are located in `external` already.

# Build

1. Clone this repository `git clone https://github.com/KeqiZeng/cpurenderer`.
2. Change directory to `cd cpurenderer`.
3. `make release`. The executable file `./build/cpurenderer` is generated.

# Gallery

`cpurenderer assets/african_head --camera_position 0.5 0.6 1.6`
[<p align="center"><img src="images/african_head.png" width=400></p>](images/african_head.png)

`cpurenderer assets/boggie --camera_position 0.5 0.8 1.6`
[<p align="center"><img src="images/boggie.png" width=400></p>](images/boggie.png)

`cpurenderer assets/boggie_on_floor --camera_position 0.8 1.2 2.4`
[<p align="center"><img src="images/boggie_on_floor.png" width=400></p>](images/boggie_on_floor.png)

`cpurenderer assets/diablo3_pose --camera_position 0.5 0.6 1.6`
[<p align="center"><img src="images/diablo3_pose.png" width=400></p>](images/diablo3_pose.png)

`cpurenderer assets/diablo3_pose_on_floor --camera_position 0.8 1.2 2.4`
[<p align="center"><img src="images/diablo3_pose_on_floor.png" width=400></p>](images/diablo3_pose_on_floor.png)

`cpurenderer assets/floor --camera_position 0.8 1.2 2.4`
[<p align="center"><img src="images/floor.png" width=400></p>](images/floor.png)

# Key Features

✅ Complete rendering pipeline

✅ View frustum culling for model

✅ Back face culling for primitives

✅ Blinn-Phong reflection model

✅ Phong shader

✅ Texture mapping with bilinear interpolation and perspective correction

✅ Shadow map

# Limitations

❌ Assume the OBJ models are triangulated

❌ Assume vertex order for back face culling

❌ No triangle clipping in clip space. If any vertex of an triangle is outside the clip space, the triangle will be discard. (to avoid annoying wrap-around issue). That's why the floor looks incomplete in some pictures.

# License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
