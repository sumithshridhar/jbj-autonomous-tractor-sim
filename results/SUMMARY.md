# Results

Planter: Kverneland Optima V, 6 rows. Ground truth comes from Gazebo, not from the tractor's own GNSS. Accuracy = cross-track error while planting, in cm: mean / 95th percentile / max.

| Scenario | Tractor | What happens | Safety stops | Outcome | Accuracy (cm) | |
|---|---|---|---|---|---|---|
| cow | MF 5M.145 Dyna-4 | A cow is standing in the path 25 m ahead, later walks away | OBSTACLE x1 | closest the bumper got: 3.57 m (contact: no); stopped for obstacle at t=47.7 s; resumed by itself at t=72.0 s | 1.07 / 2.09 / 2.23 | [report](cow_mf_5m_145/report.png) |
| person | MF 5M.145 Dyna-4 | A person walks across the path 22 m ahead | none | closest the bumper got: 5.71 m (contact: no); slowed for obstacle at t=33.8 s; resumed by itself at t=40.2 s | 1.39 / 2.13 / 2.2 | [report](person_mf_5m_145/report.png) |
| remote_estop | MF 5M.145 Dyna-4 | Operator presses the handheld e-stop at full planting speed | ESTOP x1 | standstill 1.5 s / 1.6 m after the event | 1.3 / 2.1 / 2.26 | [report](remote_estop_mf_5m_145/report.png) |
| remote_lost | MF 5M.145 Dyna-4 | The handheld remote's signal drops for 6 s | REMOTE_LOST x1 | standstill 1.9 s / 2.44 m after the event | 1.27 / 2.02 / 2.23 | [report](remote_lost_mf_5m_145/report.png) |
| rtk_loss | MF 5M.145 Dyna-4 | RTK corrections lost for 15 s (fix drops to FLOAT) | RTK x1 | standstill 2.5 s / 3.39 m after the event | 1.1 / 2.08 / 2.43 | [report](rtk_loss_mf_5m_145/report.png) |
| steering_fault | MF 5M.145 Dyna-4 | Steering motor jams at 8 deg left while planting (fault injection) | OFFTRACK x1 | standstill 4.3 s / 7.05 m after the event; max off-line 1.01 m, max outside field edge 0.0 m | 2.63 / 2.23 / 60.42 | [report](steering_fault_mf_5m_145/report.png) |
