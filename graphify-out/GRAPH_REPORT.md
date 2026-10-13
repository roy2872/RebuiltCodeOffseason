# Graph Report - RebuiltCodeOffseason  (2026-10-13)

## Corpus Check
- Corpus is ~40,622 words - fits in a single context window. You may not need a graph.

## Summary
- 1284 nodes · 3140 edges · 78 communities (22 shown, 56 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 82 edges (avg confidence: 0.85)
- Token cost: 40,794 input · 0 output

## Community Hubs (Navigation)
- MotorIO Interface & Inputs
- TalonSRX Motor IO
- Limelight NT Getters
- SparkMax Motor IO
- Limelight 2D Pose & Hardware
- Servo Motor Subsystem
- Controller Interface & Bindings
- REV & Logging Imports
- Constants & Table Loading
- Motor Simulation IO
- PhotonVision Camera IO
- LED Subsystem
- PathPlanner AD* Pathfinding
- Swerve Module TalonFX IO
- Robot Lifecycle & Container
- TalonFX Motor IO
- Servo Mechanism Sim
- General Util Helpers
- Limelight Pose Estimates
- Motor Subsystem Base
- Robot State Estimation
- Single Xbox Controller
- Feedforward Gains
- Flywheel Shooter Subsystem
- Nested Interpolating Maps
- Alliance Flipping & Hub Aim
- Vision Alerts & Matrices
- Tuner Swerve Config Types
- PID Gains & Hood Config
- AdvantageKit IO Interfaces
- Swerve Module Logic
- SuperStructure Commands
- Phoenix6 Control Requests
- TalonFX Request Getters
- Motor Control Modes
- Dummy Controller
- QX Controller
- Simulation Controller
- Swerve Module Sim
- Periodic Telemetry & Tracing
- Drive Subsystem & Pathplanner
- Drivetrain Sim Config
- Limelight Vision IO
- Distance-Angle Conversion
- Joystick Drive Commands
- CAN Port Map
- Vision IO Interface
- Roller Simulation
- SparkMax Utilities
- NavX Gyro IO
- Tunable Numbers
- Drive Module Aggregation
- Gyro Simulation
- CAN Bus Status Logging
- Feeder Constants
- Shooter Constants
- Continue Local LLM Config
- Robot Main Entry
- Tuner Swerve Drivetrain
- Drive Coast Modes
- Limelight IMU Data
- Limelight IMU Results
- Limelight Barcode Target
- Swerve Angle Utils
- Build Constants
- WPILib License

## God Nodes (most connected - your core abstractions)
1. `LimelightHelpers` - 113 edges
2. `MotorIOSparkMax` - 51 edges
3. `MotorIO` - 46 edges
4. `RobotState` - 46 edges
5. `MotorIOTalonSRX` - 37 edges
6. `Drive` - 37 edges
7. `Setpoint` - 36 edges
8. `Util` - 36 edges
9. `MotorIOTalonFX` - 33 edges
10. `MotorSubsystem` - 28 edges

## Surprising Connections (you probably didn't know these)
- `FlywheelMotorSubsystem` --inherits--> `MotorSubsystem`  [EXTRACTED]
  src/main/java/frc/lib/bases/FlywheelMotorSubsystem.java → src/main/java/frc/lib/bases/MotorSubsystem.java
- `MotorSubsystem` --references--> `TunableNumber`  [EXTRACTED]
  src/main/java/frc/lib/bases/MotorSubsystem.java → src/main/java/frc/lib/util/TunableNumber.java
- `ServoMotorSubsystem` --inherits--> `MotorSubsystem`  [EXTRACTED]
  src/main/java/frc/lib/bases/ServoMotorSubsystem.java → src/main/java/frc/lib/bases/MotorSubsystem.java
- `IntakeRollers` --inherits--> `MotorSubsystem`  [EXTRACTED]
  src/main/java/frc/robot/subsystems/intakerollers/IntakeRollers.java → src/main/java/frc/lib/bases/MotorSubsystem.java
- `MotorIOSparkMax` --inherits--> `MotorIO`  [EXTRACTED]
  src/main/java/frc/lib/io/MotorIOSparkMax.java → src/main/java/frc/lib/io/MotorIO.java

## Import Cycles
- None detected.

## Communities (78 total, 56 thin omitted)

### Community 1 - "TalonSRX Motor IO"
Cohesion: 0.09
Nodes (4): ControlModeSetter, MotorIOTalonSRX, MotorIOTalonSRXConfig, IntakeRollersConstants

### Community 5 - "Limelight 2D Pose & Hardware"
Cohesion: 0.10
Nodes (10): HardwareReport, LimelightResults, LimelightTarget_Classifier, LimelightTarget_Detector, LimelightTarget_Fiducial, LimelightTarget_Retro, RewindStats, ReefPoseEstimate (+2 more)

### Community 6 - "Servo Motor Subsystem"
Cohesion: 0.12
Nodes (5): ServoHomingConfig, ServoMotorSubsystem, DelayedBoolean, Hood, IntakeDeploy

### Community 9 - "Constants & Table Loading"
Cohesion: 0.10
Nodes (7): TableLoader, Constants, Mode, REAL, REPLAY, SIM, RobotState

### Community 10 - "Motor Simulation IO"
Cohesion: 0.15
Nodes (3): MotorIOSparkMaxSim, MotorIOTalonFXSim, MechanismSim

### Community 11 - "PhotonVision Camera IO"
Cohesion: 0.11
Nodes (5): FieldConstants, TxTyObservation, VisionConstants, VisionIOPhoton, VisionIOPhotonSim

### Community 13 - "LED Subsystem"
Cohesion: 0.11
Nodes (10): Leds, ledsStates, AQUA, BLUE, FINISH_SCORE, OFF, PINK, PURPLE (+2 more)

### Community 19 - "Servo Mechanism Sim"
Cohesion: 0.14
Nodes (4): ServoMechanismSim, ServoMechanismSimConstants, Pose2dTimeInterpolable, IntakeRollers

### Community 21 - "Limelight Pose Estimates"
Cohesion: 0.14
Nodes (4): PoseEstimate, RawDetection, RawFiducial, RawTarget

### Community 23 - "Motor Subsystem Base"
Cohesion: 0.16
Nodes (3): MotorSubsystem, Setpoint, Feeder

### Community 24 - "Robot State Estimation"
Cohesion: 0.15
Nodes (4): accelLimitsLib, LimelightYawObservation, OdometryObservation, RobotState

### Community 26 - "Feedforward Gains"
Cohesion: 0.10
Nodes (4): ControlGains, FFConstants, SimpleFFConstants, TrapezoidalProfileGains

### Community 27 - "Flywheel Shooter Subsystem"
Cohesion: 0.22
Nodes (3): FlywheelMotorSubsystem, ForceHomeConstants, Shooter

### Community 28 - "Nested Interpolating Maps"
Cohesion: 0.17
Nodes (4): NestedInterpolatingTreeMap, InterpolatingMeasureMap, MeasureInterpolable, MeasureInverseInterpolable

### Community 32 - "PID Gains & Hood Config"
Cohesion: 0.17
Nodes (3): PidGains, HoodConstants, IntakeDeployConstants

### Community 33 - "AdvantageKit IO Interfaces"
Cohesion: 0.19
Nodes (5): MotorInputs, GyroIO, GyroIOInputs, ModuleIO, ModuleIOInputs

### Community 38 - "Motor Control Modes"
Cohesion: 0.14
Nodes (7): Mode, DUTY_CYCLE, IDLE, MOTIONMAGIC, POSITIONPID, VELOCITY, VOLTAGE

### Community 50 - "CAN Port Map"
Cohesion: 0.17
Nodes (12): Ports, FEEDER_ROLLERS_FOLLOWER1, FEEDER_ROLLERS_FOLLOWER2, FEEDER_ROLLERS_MAIN, HOOD, INTAKE_DEPLOY, INTAKE_ROLLERS_FOLLOWER, INTAKE_ROLLERS_MAIN (+4 more)

### Community 51 - "Vision IO Interface"
Cohesion: 0.27
Nodes (4): PoseObservation, TargetObservation, VisionIO, VisionIOInputs

### Community 56 - "Drive Module Aggregation"
Cohesion: 0.28
Nodes (3): ModuleIO, ModuleIO, ModuleIO

### Community 64 - "Continue Local LLM Config"
Cohesion: 0.50
Nodes (4): Local Config (Continue agent config), Model Roles (chat, edit, apply), Ollama Provider (localhost:11434), Qwen 2.5 Coder (qwen2.5-coder:7b)

### Community 68 - "Drive Coast Modes"
Cohesion: 0.50
Nodes (4): CoastRequest, ALWAYS_BRAKE, ALWAYS_COAST, AUTOMATIC

## Knowledge Gaps
- **35 isolated node(s):** `IDLE`, `VOLTAGE`, `MOTIONMAGIC`, `VELOCITY`, `DUTY_CYCLE` (+30 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 272 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **56 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LimelightHelpers` connect `Limelight NT Getters` to `Limelight 3D Botpose`, `Limelight IMU Data`, `Limelight 2D Pose & Hardware`, `Limelight IMU Results`, `Limelight Barcode Target`, `Limelight JSON Parsing`, `Limelight Pose Estimates`, `Limelight Results Parsing`?**
  _High betweenness centrality (0.140) - this node is a cross-community bridge._
- **What connects `IDLE`, `VOLTAGE`, `MOTIONMAGIC` to the rest of the system?**
  _35 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `MotorIO Interface & Inputs` be split into smaller, more focused modules?**
  _Cohesion score 0.06567992599444958 - nodes in this community are weakly interconnected._
- **Why does `MotorIO` connect `MotorIO Interface & Inputs` to `TalonSRX Motor IO`, `SparkMax Motor IO`, `Servo Motor Subsystem`, `Motor Control Modes`, `Periodic Telemetry & Tracing`, `Distance-Angle Conversion`, `TalonFX Motor IO`, `Vendor Control Enums`, `Motor Subsystem Base`, `Flywheel Shooter Subsystem`?**
  _High betweenness centrality (0.061) - this node is a cross-community bridge._
- **Should `TalonSRX Motor IO` be split into smaller, more focused modules?**
  _Cohesion score 0.09082125603864734 - nodes in this community are weakly interconnected._
- **Why does `MotorIOSparkMax` connect `SparkMax Motor IO` to `MotorIO Interface & Inputs`, `PID Gains & Hood Config`, `TalonSRX Motor IO`, `Servo Motor Subsystem`, `REV & Logging Imports`, `Motor Simulation IO`, `Vendor Control Enums`, `Motor Subsystem Base`, `Flywheel Shooter Subsystem`, `Feeder Constants`, `Shooter Constants`?**
  _High betweenness centrality (0.060) - this node is a cross-community bridge._
- **Should `Limelight NT Getters` be split into smaller, more focused modules?**
  _Cohesion score 0.08309178743961353 - nodes in this community are weakly interconnected._