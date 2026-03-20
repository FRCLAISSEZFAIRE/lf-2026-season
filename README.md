# FRC Team Laissez Faire 2026 Season Robot Code

This repository contains the official robot code for the **LF 2026 Season** robot, built using the WPILib framework in Java. It features advanced autonomous navigation, a custom 2D/3D tunable FieldConstants system, and heavily optimized subsystems for Intake, Feeder, Shooter, and Swerve Drive.

---

## 🚀 Key Features

### 🎮 Single Driver Setup (Unified Control)
The operator role has been entirely eliminated, simplifying communication and logic. Both driving and subsystem operations are routed exclusively through the **Driver Controller** (Port 0).
- **Triggers**: Intelligent Shooting (Right) & Roller Toggle (Left)
- **Bumpers**: Auto TrenchPass (Right Bumper)
- **B Button**: Auto FeedPass Pathing
- **POV**: Live Turret Target Offset tuning.

### 🎯 Shooter Subsystem (Auto-Calculated Interpolation)
The shooter is designed to always track the "Hub Center". 
- **MegaTag 2 Integration**: Reads standard deviations from AdvantageKit/Limelight to maintain extreme accuracy.
- **Physics-Based Aiming**: Turret RPMs and Hood angles are mathematically interpolated purely based on the robot's current 2D translation distance from the tunable `HubCenter` target, eliminating the need for manual overrides.

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
- **`FeedPassCommand`**: Automatically moves to the closest `FeedStart` point and traverses to the `FeedStop` point while simultaneously deploying the Rack & Pinion intake and spinning up the rollers to safely collect field pieces on the fly. 

---

## 💻 Code Architecture

* **`frc.robot.Robot`**: Core lifecycle loops (teleopPeriodic, autonomousInit).
* **`frc.robot.RobotContainer`**: Subsystem instantiation and command mappings.
* **`frc.robot.ControllerBindings`**: Contains all controller button mappings for the Single-Driver setup.
* **`frc.robot.constants.FieldConstants`**: Contains every single field coordinate (from Hub location, Climbing nodes, to Auto-Feeding points) mapped natively to `TunableNumber` so they can be changed via SmartDashboard without recompiling.
* **`frc.robot.commands.*`**: Encapsulated actions (ShootCommand, SimpleDriveToPose, TrenchPass, AutoIntake).

---

## 🛠️ Usage & Setup

### Requirements
- **WPILib 2024/2026** environment setup.
- Relevant vendor libraries (REV Robotics, CTRE Phoenix 6, AdvantageKit).

### Building and Simulating
To compile the code cleanly to verify changes:
```bash
./gradlew build
```

To run a simulation instance on your computer to view the SmartDashboard variables:
```bash
./gradlew simulateJava
```
