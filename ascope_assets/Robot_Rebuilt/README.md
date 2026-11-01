# Robot_Rebuilt (AdvantageScope model)

3D model of the robot for AdvantageScope's 3D Field tab. The robot code logs one pose per moving
part (`RobotVisualizer/Components/Measured` and `.../Setpoint`), and AdvantageScope moves the
matching `model_INDEX.glb` with it.

## Files

| File | Contents | Code joint |
| --- | --- | --- |
| `model.glb` | Everything that does not move relative to the drivetrain (no hood, no intake slide) | - |
| `model_0.glb` | Intake slide | `Intake Deploy` (linear) |
| `model_1.glb` | Hood | `Hood` (rotary) |

The index is the registration order in `src/main/java/frc/robot/RobotVisualization.java`. Check the
logged `RobotVisualizer/Components/Names` if unsure. Adding a part means: register a joint there,
export `model_N.glb`, and append one entry to `components` in `config.json`.

## Exporting from CAD

The code rotates parts about their real pivot, so every part must be exported **in place**, from the
full robot assembly, in the zero pose:

- Hood at `HoodConstants.HOOD_STARTING_ANGLE` (stowed).
- Intake fully stowed.

1. In the robot assembly, hide everything except the parts for one file (or suppress the moving parts
   for `model.glb`). Do not move or re-origin anything.
2. Export as STEP (or glTF/GLB directly if your CAD supports it) using the assembly origin. Put the
   assembly origin on the floor at the robot center, +X forward, +Y left, +Z up. If it is elsewhere,
   fix it with `position` (see below).
3. Convert STEP to GLB. AdvantageScope's guide: https://docs.advantagescope.org/more-features/custom-assets/gltf-convert
4. Name the files as in the table above and put them in this folder.

## config.json

- `rotations` turn the exported model so it is Z-up. Many STEP to glTF tools export Y-up, which
  `x: 90` fixes. **TODO:** check in AdvantageScope and adjust if the robot is lying on its side or
  facing backward (add e.g. `{"axis": "z", "degrees": 180}`).
- `position` moves the model if the CAD origin is not on the floor at the robot center.
- Every entry in `components` must use **the same** `rotations` (as `zeroedRotations`) and `position`
  (as `zeroedPosition`) as the base model. Since all parts share the assembly origin, nothing else
  needs calibrating, unlike setups that place each part at its pivot.

## Using it in AdvantageScope

1. Point AdvantageScope at the assets: App (AdvantageScope on macOS) > Use Custom Assets Folder > select the `ascope_assets`
   folder in this repo.
2. 3D Field tab: add the robot pose (`RealOutputs/RobotState/EstimatedPose`) as a **Robot** and pick
   "Rebuilt" as the model.
3. Drag `RobotVisualizer/Components/Measured` onto the robot as **Component**.
4. Optional: add the robot pose again as a **Ghost** and drag `RobotVisualizer/Components/Setpoint`
   onto it, to see where each mechanism is being commanded to.
5. No models yet? In sim, drag `RobotVisualizer/SideView` onto the robot to get a 2D mechanism drawn
   on it, or open it in the Mechanism tab.

On the field (FMS attached) the `RobotVisualizer` keys are not published over NetworkTables to save
bandwidth, but they are still in the log file on the USB stick. Open the log after the match to watch
it. Keep AdvantageScope live mode on "Low Bandwidth" at events.
