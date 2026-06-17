---
icon: lucide/lightbulb
---

# Tips

## Viewing the point clouds

To view the point clouds in 3D, you can use [CloudCompare](https://www.danielgm.net/cc/).
To view them in 2D together with the roofprints and footprints, you can use [QGIS](https://www.qgis.org/en/site/).
However, QGIS does not support custom fields in LAZ files, so the solution I found to open them is to first convert them to EPT.
For that, you can use [Entwine](https://entwine.io/), which is installed with pixi and that you can run like this:

```bash
pixi run entwine build -i <input_laz> -o <output_ept_folder> --deep --srs EPSG:2154
```
