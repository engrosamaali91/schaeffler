# Creating Occupancy Map of Autonomous Production Hub

## Table of Contents
- [Creating Occupancy Map of Autonomous Production Hub](#creating-occupancy-map-of-autonomous-production-hub)
  - [Table of Contents](#table-of-contents)
  - [Issue](#issue)
  - [Solution](#solution)
  - [Before](#before)
  - [occupancy](#occupancy)

## Issue
Black box artifacts covered the robotic cell area in the occupancy map.

## Solution
```GIMP``` application is used to clean up the map and improve occupancy around the robotic cell.

![Map with black box](./media/RIICO/issues.png)
Refer to 

## Before
<table>
  <tr>
    <th>Before</th>
    <th>After</th>
  </tr>
  <tr>
    <td><img src="media/RIICO/riico_right.png" alt="Before map" width="100%"></td>
    <td><img src="src/nav_bringup/maps/riico_right.png" alt="After map" width="100%"></td>
  </tr>
</table>


## occupancy

Autonomous production hub loaded in isaac sim. Occupancy map being visualized in rivz2.

![IsaacSim With occupancy Map](./media/RIICO/Screenshot%20from%202026-05-07%2010-50-41.png)




