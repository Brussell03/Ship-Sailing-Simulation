# Ship Sailing Simulation
This project aims to develop a realistic, physics-based ship sailing experience, focusing on ships from the Age of Sail.

![Unity_2025-07-20_17-47-39](https://github.com/user-attachments/assets/1fe374a9-3f60-47f7-9348-ceb04de02124)

## Buoyancy Simulation
A ship's buoyancy simulation uses a custom-built system. A ship can be subdivided into several simplified sections, each with its own defined geometries. A buoyant force is found for each section, depending on the ship's orientation and water height at the section. Forces are applied at each section's center of mass.

## Drag Simulation
Drag is calculated on all faces of the ship in both the normal and transverse directions. The drag on each face is calculated separately for parts that are underwater and above water. For each one, the respective fluid density and relative motion are considered.

## Cloth Simulation
The project features a GPU-driven cloth simulation utilizing the Extended Position Based Dynamics (XPBD) method. The implementation supports aerodynamic lift and drag forces, allowing for a semi-accurate force to be applied on the cloth or ship. Cloths can be triangular, rectangular, or trapezoidal. Cloth vertices can easily be pinned or connected to other cloth vertices. The cloth simulation also includes a Level-of-Detail (LOD) system that allows cloths to decrease in quality as the distance from the player increases. This allows for the maximum number of simulated vertices to be limited, ensuring the performance impact from the simulation is within bounds.
![Unity_2025-07-16_16-34-07](https://github.com/user-attachments/assets/204f41be-4e90-468f-8e8a-80663442ba53)
