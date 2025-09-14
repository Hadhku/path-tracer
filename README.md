### Introduction

A simple C++ path tracer, with and without next event estimator (NEE).

### Dependencies

- C++20
- OpenMP (for parallel processing)

### Renders

Diffuse (Lambertian) tall block, image rendered without NEE.
![Diffuse tall block](image/lambert_no_nee.png)
Diffuse (Lambertian) tall block, image rendered with NEE.
![Diffuse tall block](image/lambert_with_nee.png)
Tall block is a dirac reflector, image rendered with NEE.
![Mirror tall block](image/mirror.png)

##### TODO:

Add multi importance sampling (MIS) to NEE.
