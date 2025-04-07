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
![african_head](images/afican_head.png)

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
