# Ship Sailing Simulation
This project aims to develop a realistic, physics-based ship sailing experience, focusing on ships from the Age of Sail.


https://github.com/user-attachments/assets/4d310e0d-3fa8-4999-a892-d6d8aaf612b4

https://github.com/user-attachments/assets/d8a603db-d63d-4344-b4e3-c3e928971cc5

## Buoyancy Simulation
The buoyancy system employs a custom-built computational approach to accurately simulate a ship's interaction with water. The ship's hull is divided into multiple simplified geometric sections, each with a defined volume. The buoyancy force is calculated for every section based on its submerged volume, determined by the ship's current orientation and the height of the waterline at that point. These forces are applied to each section's calculated center of mass, resulting in realistic hull displacement, pitching, and rolling.

## Drag Simulation
The simulation calculates fluid drag on all relevant surfaces of the ship, distinguishing between underwater and above-water forces.

<b>Underwater Drag:</b> The system applies drag forces on the hull's submerged faces, considering water density and the ship's velocity relative to the water. This accounts for resistance in both normal and transverse directions, influencing the ship's top speed and turning radius.

<b>Aerodynamic Drag:</b> Above the waterline, drag is calculated on a per-face basis for all sails and rigging. It models the resistance of air, factoring in air density and the ship's relative motion, to simulate the wind's influence on the vessel's movement.

## Cloth Simulation
The cloth system is a core feature, utilizing a high-performance GPU-driven simulation to manage the sails.

<b>Extended Position Based Dynamics (XPBD):</b> The simulation is built on the XPBD method, chosen for its stability and efficiency in handling a large number of constraints.

<b>Aerodynamic Forces:</b> The system supports a semi-accurate model for aerodynamic lift and drag, calculated on each vertex to simulate how wind deforms the sails. The force applied is crucial for translating wind into ship movement.

<b>Flexible Cloth Topologies:</b> The implementation is designed to be highly flexible, supporting sails with various topologies, including triangular, rectangular, and trapezoidal shapes.

<b>Dynamic Vertex Management:</b> Key vertices can be interactively pinned to the mast and rigging, or connected to other cloth vertices for more complex rigging setups.

<b>Performance Optimization with LOD:</b> To ensure the simulation remains performant with many ships, the cloth system includes a Level-of-Detail (LOD) system. This automatically reduces the resolution of a cloth's mesh as its distance from the player increases, limiting the total number of simulated vertices to stay within performance bounds.

![Unity_2025-07-16_16-34-07](https://github.com/user-attachments/assets/204f41be-4e90-468f-8e8a-80663442ba53)
