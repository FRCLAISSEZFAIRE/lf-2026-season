# FRC Team Laissez Faire 2026 Season Robot Code

This repository contains the official robot code for the **LF 2026 Season** robot, built using the WPILib framework in Java. It features advanced autonomous navigation, a custom 2D/3D tunable FieldConstants system, and heavily optimized subsystems for Intake, Feeder, Shooter, and Swerve Drive.

---

## 🚀 Key Features

### 🎮 Single Driver Setup (Unified Control)
The operator role has been entirely eliminated, simplifying communication and logic. Both driving and subsystem operations are routed exclusively through the **Driver Controller** (Port 0).
- **Triggers**: Intelligent Shooting (Right) & Roller Toggle (Left)
- **Bumpers**: Auto TrenchPass (Right Bumper)
- **B Button**: Auto FeedPass Pathing
- **X Button**: Reverse Intake + Feeder (Kusma)
- **Y Button**: Auto Drive to Shooting Pose
- **POV**: Live Hub Target Offset tuning (±0.1m per press)

### 🎯 Shooter Subsystem (Auto-Calculated Interpolation)
The shooter is designed to always track the "Hub Center". 
- **MegaTag 2 Integration**: Reads standard deviations from AdvantageKit/Limelight to maintain extreme accuracy.
- **Physics-Based Aiming**: Turret RPMs and Hood angles are mathematically interpolated purely based on the robot's current 2D translation distance from the tunable `HubCenter` target, eliminating the need for manual overrides.
- **Hood Soft Limits**: Prevents mechanical damage by clamping hood angles to safe range.

### ⚙️ Feeder Subsystem (Hybrid Voltage/Velocity)
The Feeder utilizes a dual-PID and Voltage approach:
- **Kicker & Indexer Separation**: Two independent SparkMax motors.
- The Feeder uses a robust voltage control scheme designed to bypass encoder sensor dropouts in the middle of matches, ensuring the robot can always spit or feed pieces.

### 📏 Intake Subsystem (Linear Rack & Pinion)
In 2026, the pivot-based intake design was discarded in favor of a **Linear Extension (Rack and Pinion)** mechanism driven by a SparkMax motor.
- Calculations are derived intelligently from `Gear Ratio` and `Pinion Diameter`.
- Tunable `Extension Target Cm` and Live Soft Limits via SmartDashboard. 
- Integrated Kraken roller for powerful material collection.

### 🛣️ Smart Auto Commands (`TrenchPass` & `FeedPass`)
The robot doesn't just drive to poses statically; it knows where it is.
- **`TrenchPassCommand`**: Reads the Alliance and current `Y` position to calculate whether to route through the upper or lower safe transition points (A, B, C, D) dynamically.
- **`FeedPassCommand`**: Automatically moves to the closest `FeedStart`/`FeedStop` point and traverses the collection path while simultaneously deploying the Rack & Pinion intake and spinning up the rollers. Alliance-aware and route-optimized (always travels to the nearest point first).
- **`BumpPassCommand`**: Alternative pass route through inner bump zones.

### 🤖 Autonomous Scenarios
Autonomous routines are managed by `AutonomousScenarios.java` with a `SendableChooser`:
- **Scenario 0**: Do nothing (safety default)
- **Scenario 1**: Collect & Shoot (TrenchPass → Intake → TrenchPass → Shoot)
- **Scenario 2**: Collect, Shoot & Climb (adds Outpost drive)
- **Scenario 3**: Double Collect & Shoot (two full collection cycles)
- **Pass Mode Chooser**: Dashboard-selectable Trench or Bump pass routes via `DeferredCommand`

---

## 🔧 Dashboard Tuning System

All critical parameters are persisted to RoboRIO flash memory via the `Preferences` API:

### TunableNumber & TunableBoolean
- **`TunableNumber`**: Double values editable from dashboard, saved to RIO (`/Preferences/[group]/[key]`)
- **`TunableBoolean`**: Boolean values displayed as Switches/Toggles on the dashboard

### Speed Multipliers

| Dashboard Path | Mode | Default |
|---|---|---|
| `Tuning/Drive/TeleopSpeedMultiplier` | Teleop | 1.0 |
| `Tuning/Auto/SpeedMultiplier` | Autonomous | 1.0 |
| `Tuning/Auto/FeedSpeedMultiplier` | Autonomous (Feed) | 0.5 |

### Motor Inversion Controls (Live Toggle)
Swerve motor inversions are controlled via `TunableBoolean` switches under:
- `Drive/Inverts/Driving/` — FL, FR, RL, RR driving motors (SparkFlex)
- `Drive/Inverts/Turning/` — FL, FR, RL, RR turning motors (SparkMax)

These values are:
1. **Loaded from RIO** on robot startup
2. **Applied immediately** to motor controllers during initialization
3. **Changeable live** from the dashboard during Teleop and Test modes

### Field Coordinates
All field positions (Hub, Pass targets, Feed paths, Trench/Bump waypoints, Outpost, Climb) are `TunableNumber` under `/Preferences/Field/...` and can be adjusted without recompilation.

---

## 💻 Code Architecture

* **`frc.robot.Robot`**: Core lifecycle loops (teleopPeriodic, autonomousInit).
* **`frc.robot.RobotContainer`**: Subsystem instantiation and command mappings.
* **`frc.robot.ControllerBindings`**: Contains all controller button mappings for the Single-Driver setup.
* **`frc.robot.AutonomousScenarios`**: Defines autonomous routines with `SendableChooser` and `DeferredCommand` for runtime pass selection.
* **`frc.robot.constants.FieldConstants`**: Contains every single field coordinate mapped to `TunableNumber` for live adjustment via dashboard.
* **`frc.robot.util.TunableNumber`**: Persistent double tuning via WPILib `Preferences` API.
* **`frc.robot.util.TunableBoolean`**: Persistent boolean tuning (switch/toggle on dashboard).
* **`frc.robot.commands.*`**: Encapsulated actions (ShootCommand, SimpleDriveToPose, TrenchPass, BumpPass, FeedPass).

---

## 🛠️ Usage & Setup

### Requirements
- **WPILib 2026** environment setup (GradleRIO 2026.2.1).
- Vendor libraries: REV Robotics (REVLib 2026), CTRE Phoenix 6, AdvantageKit, Studica NavX.

### Building and Simulating
To compile the code cleanly to verify changes:
```bash
./gradlew build
```

To run a simulation instance on your computer to view the SmartDashboard variables:
```bash
./gradlew simulateJava
```

### ⚠️ Important: NetworkTables Persistence
When changing `TunableNumber`/`TunableBoolean` **default values in code**, delete the saved persistence file before restarting to force fresh defaults:
```bash
rm -f networktables.json networktables.json.bck
```
