# Build plan: from this sim to a real autonomous tractor

The sim is the first step. This page lists what goes on the tractor, how it's wired, and how to test it in the field
without anyone getting hurt. Prices are public list prices (Oct 2026); get quotes before buying.

## 1. What goes on the tractor

```
  ROOF: 2 x GNSS antennas (1 m apart) . camera . amber beacon . 4G antenna
  CAB:  Sveaverken F200 steering motor + wheel-angle sensor . F200 terminal . control box (computer + I/O board) . red e-stop
  FRONT: Livox Mid-360 lidar, upside down at 1.3 m on a short mast . red e-stop on the bumper
  SIDES: red e-stop on each mudguard (reachable from the ground)
  POCKET: handheld radio remote with e-stop (the person supervising)
```

| Job | Part | Notes |
|---|---|---|
| Steer | Sveaverken F200 steering motor + wheel-angle sensor | Use it as-is if the maker gives an interface to command steering. If not, drive the same kind of motor from our own controller |
| Know where it is | Dual-antenna RTK GNSS (the F200 receiver, or an open kit such as ArduSimple simpleRTK2B Heading) | Corrections from a base station or an NTRIP network over 4G |
| See obstacles | Livox Mid-360 lidar (USD 979) | Stops for anything 0.3 m+ high in the path; slows at 15 m |
| Think | Jetson Orin Nano Super (USD 249) or rugged mini PC, ROS 2 | Runs the same nodes as this sim |
| Talk to the tractor | Teensy 4.1 / ESP32 + CAN transceiver + relays | Speed, direction, hitch, PTO, engine stop |
| Stay safe | Hard-wired e-stop chain + safety relay + radio remote | Works even if the computer crashes (section 3) |

## 2. How each tractor gets controlled

| Tractor | Direction / speed | Hitch | What to check on JBJ's unit |
|---|---|---|---|
| **MF 5M.145 Dyna-4** | Dyna-4 is clutch-free with a Power Control shuttle lever, so direction and gear changes are electrical signals | Electronic Linkage Control (ELC) | Which signals the Power Control lever sends; whether the factory guidance (MF Guide) / ISOBUS options are fitted |
| **MF 7615** | Dyna-4, Dyna-6 or Dyna-VT: all electro-hydraulic, no clutch pedal needed | Electronic rear linkage | Which transmission JBJ's unit has. Dyna-VT is the easiest (stepless speed) |
| **MF 8732 Dyna-VT** | Stepless Dyna-VT; speed and direction fully electronic; ISOBUS | Electronic, 12 t rear lift | Steering angle (the sim assumes 50 deg); a 6-row planter is small for it (wider planters need a wider headland) |

The rule: **control the tractor through its own electronics (shuttle lever, hand throttle, hitch switches), never with
robot arms on the pedals.** That's why tractors with an electronic shuttle are the right first choice.

## 3. Safety chain (independent of the computer)

```
  battery + --[cab e-stop]--[bumper e-stop]--[left e-stop]--[right e-stop]--[radio remote relay]--[safety relay coil]
                                                                                     |
  safety relay contacts --> shuttle to neutral + brakes on + engine stop solenoid    |
  computer watchdog ------> opens the chain if the software stops sending a heartbeat
```

- Any button, a lost radio signal, or a frozen computer opens the chain, and the tractor stops whatever the software is doing.
- The software stops are on top of this: obstacle, RTK lost, off-track, field edge (geofence). They're all tested in the sim.

## 4. Test plan on the real tractor (pass criteria from the sim)

| Step | Where | Pass when |
|---|---|---|
| 1. Sim | This repo | All scenarios pass (results/SUMMARY.md) |
| 2. Bench | Control box on a table, motor + relays connected | E-stop chain cuts outputs in < 0.1 s; watchdog trips when the computer is unplugged |
| 3. Wheels off the ground | Tractor on stands, engine running | Steering follows commands; measure the real steering speed (sim assumes 25 deg/s) |
| 4. Open field, walking speed | Flat land, person on the remote e-stop, driver in the seat | Stops within 1 m at 3 km/h on remote e-stop; follows a straight line within 5 cm |
| 5. Planting speed | Driver in the seat | Line within 2.5 cm (the F200's spec, the sim gets ~2 cm); remote e-stop stops it within ~2 m at 7 km/h (sim: 1.6 m) |
| 6. Supervised autonomy | Driver out, supervisor with the remote, geofence on | Full test field planted; every stop type triggered once on purpose |

## 5. Numbers to measure first (they're assumptions in the sim)

Steering speed and lag through the motor, braking distance on JBJ's soil at 3 and 7 km/h, the remote's signal-loss
timeout, the MF 8732's steering angle, the MF 7615 / 8732 ground clearance, the planter's mass and length.
