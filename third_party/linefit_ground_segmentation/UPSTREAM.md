# LineFit ground segmentation core

The `Bin` and `Segment` headers and sources come from Lorenz Wellhausen's [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation/tree/e6b9418cc5756e30e4ea87b11d8792f936a2002a/linefit_ground_segmentation) at `e6b9418cc5756e30e4ea87b11d8792f936a2002a`. The BSD-3-Clause license is included alongside the code.

The core implements the ground-line fitting method described by Himmelsbach, von Hundelshausen and Wünsche in [Fast segmentation of 3D point clouds for ground vehicles](https://doi.org/10.1109/IVS.2010.5548059). The Studio adapter fills polar bins and classifies points in source order, with one writer per scan and without the upstream ROS or visualization code.
