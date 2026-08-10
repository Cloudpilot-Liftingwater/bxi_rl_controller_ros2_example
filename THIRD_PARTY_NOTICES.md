# Third-party notices for the SONIC integration

This branch adds a modified subset of NVIDIA GEAR-SONIC for ELF3 SONIC
teleoperation. The subset is located under
`src/bxi_example_py_elf3/bxi_example_py_elf3/sonic_pico/vendor/gear_sonic`.
It has been package-scoped to the ELF3 headless runtime and includes local XRT
shutdown, process-cleanup, native calibration, and ELF3 robot-model fixes.
The complete vendored-file inventory and the files modified by BXI Robotics are
listed in `third_party/GEAR_SONIC_FILES.md`. Modified source files also carry
file-level change notices.

GEAR-SONIC source code is Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES
and is provided under Apache License 2.0. See
`third_party/GEAR_SONIC_LICENSE.txt` and `third_party/APACHE-2.0.txt`.

The SONIC ONNX weights are governed by the NVIDIA Open Model License contained
in `third_party/GEAR_SONIC_LICENSE.txt`.

Licensed by NVIDIA Corporation under the NVIDIA Open Model License.

The checked-in ONNX files differ from their original exports only by removal
of ONNX node `doc_string` fields that contained export-machine stack traces;
graph parameters and inference contracts are unchanged. Their SHA256 values
are:

```text
8b30e2f9afc24e081662ea8daf47724753d16918e633688c9561ef4cc54d71e0  step28800 modular encoder, (1,1751) -> (1,64)
8f60f1aae1191ac22a8bb1b0eaf98110529adaf09d2a13bb6d9d61c873bbf69f  step28800 g1_dyn decoder, (1,994) -> (1,29)
26dc3e96adfb894850b409e43f06178c79b74167c719f626eadbb9df3fcacd06  step28800 SMPL policy, (1,1770) -> (1,29)
```

`rotation_conversion.py` contains code derived from PyTorch3D, Copyright (c)
Meta Platforms, Inc. and affiliates, under the BSD 3-Clause License. See
`third_party/PYTORCH3D_LICENSE.txt`.

`kornia_transform.py` contains transformations derived from Kornia, which is
distributed under Apache License 2.0.

The XRoboToolkit Python binding and RoboticsService are external runtime
dependencies. No XRT SDK, shared object, DEB package, service executable, or
other vendor binary is distributed in this repository.

The ELF3 URDF and STL geometry, including the optional Sim2Sim gripper assets,
are BXI Robotics project assets and are included in this public repository with
authorization from their owner. They are not covered by the NVIDIA GEAR-SONIC
attribution or license above; their use remains subject to the licensing terms
applicable to the main project.
