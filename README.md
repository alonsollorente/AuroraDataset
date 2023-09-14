# AuroraDataset
A comprehensive Sensor Data Collection for Mapping and Localization in Inland Waterway Scenarios

## Abstract

With the volume of freight traffic increasing over the past few years, road- and rail-borne transportation tend to reach their capacity limits. Inland waterway transport (IWT) constitutes an appealing alternative to land-based transportation. Being a mode of carriage with a long history, IWT offers the advantages of high safety, low energy consumption and cost- efficiency. With the currently used large inland vessels and comparably long transportation time, IWT is most suitable for the transportation of bulky and heavy freight. In addition to the efficiency of mass transportation, the reduced effect on habitat damage of inland waterways emphasises the environmental safety of this mode of transport. To expand the operational capacity of Inland Waterway Transport (IWT), it is imperative to guarantee the safety of its associated infrastucture. One contribution to this goal, is to ensure accurate and well updated chart data. In this context, mapping and localization technologies becomes essential. Unfortunately, current open-source datasets either not address this challenge or do not provide enough sensor data. With the purpose to fix this, we present the AuroraDataset: a collection of sensory data recorded in Indland Waterway Scenarios, specially designed to output HD mapping and localization solutions. The dataset counts with XX trajectories in Berlin, featuring a wide array of scenarios. These scenarios deliberately encompass different challenges, such as navigating around islands, passing under multiple bridges, and maneuvering through sinuous curves in waterways. By doing so, we aim to provide researchers and practitioners with a robust dataset that reflects the complexity and diversity of real-world IWT environments. Furthermore, our dataset includes data from multiple LiDAR, stereocamera, IMU (Inertial Measurement Unit), and GNSS (Global Navigation Satellite System) antennas.

Keywords: HD mapping, localization, GNSS, Berlin

### Contact Authors:
- [Daniel Medina](https://scholar.google.com/citations?user=8Yd99BcAAAAJ&hl=es&oi=ao)
- [Alonso Llorente](https://github.com/alonsollorente)
- [Lukas Hösch]()
- [Iulian Filip]()

_All authors belong to the [Navigation Systems](https://www.dlr.de/kn/desktopdefault.aspx/tabid-2204/admin-1/) department from the [Insitute of Communication and Navigation](https://www.dlr.de/kn/desktopdefault.aspx/tabid-17684/) at the German Aerospace Center ([DLR](https://www.dlr.de/de))_
### Related papers:
- ITSC paper?

## 1. Berlin dataset
### 1.1 Sensor setup
The data collection platform used for the Berlin dataset, is equipped with the following sensors:
- [Velodyne VLP-32c](https://icave2.cse.buffalo.edu/resources/sensor-modeling/VLP32CManual.pdf) LiDAR: 200m measurement range, 360º HFOV, 40º VFOV, 10Hz recording rate
- [Velodyne VLP-16](https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf) LiDAR: 100m measurement range, 360º HFOV, 30º VFOV, 10 Hz recording rate
- Sick SCAN 
