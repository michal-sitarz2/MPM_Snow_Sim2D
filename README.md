# Material Point Method for Snow Simulation
A 2D pipeline for rendering physically accurate snow dynamics based on the "Material Point Method for Snow Simulation" (Stomakhin et al., 2013).

![snow_sim_2d](outputs/mpm_2d_sim.gif)
![snow_sim_3d](outputs/mpm_3d_sim.gif)

### Build
The following commands can be used to build the soluation, and generate the executable:
``` bash
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DUSE_3D=ON
cmake --build . --target MPM --config Release
```

* *Note*: `-DUSE_3D=[ON/OFF]` flag can be used to change between 3D and 2D implementations. However, 3D requires GPU. So if the GPU is off in `src/Parameters.h`, and the `-DUSE_3D=ON`, the program will fail to build.

### Simulation
All of the simulation parameters (including GPU and CPU simulation) are located in `src/Parameters.h`.
The frames of the simulation will be rendered into the `outputs/` folder.



