* Learn about gpu scheduling . 

* A CUDA context is analogous to a CPU process. All resources and actions performed within the driver API are encapsulated inside a CUDA context, and the system automatically cleans up these resources when the context is destroyed

* Modules are dynamically loadable packages of device code and data, akin to DLLs in Windows, that are output by nvcc (see Compilation with NVCC). The names for all symbols, including functions, global variables, and texture or surface references, are maintained at module scope so that modules written by independent third parties may interoperate in the same CUDA context.

* kernels, that, when called, are executed N times in parallel by N different CUDA threads, as opposed to only once like regular C functions.( Thats why <<<N,N>>> in CUDA host functions.


