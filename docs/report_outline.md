# Report outline — AL5B Arm Project

Use this as the skeleton for your Word document submission. Each section maps
directly to a rubric line from the final project brief.

## 1. Introduction
- Team members and student IDs
- Course, semester, instructor
- One-paragraph summary of the project goal: implementing PC → SSC-32 → AL5B
  arm control with virtual joystick, kinematics, and scripted pick-and-place
- Why you chose the alternative robotics platform (page 14 of brief)

## 2. Hardware setup
- Block diagram: PC ↔ USB ↔ SSC-32 ↔ servos (base, shoulder, elbow, wrist,
  wrist rotate, gripper)
- Photo of the assembled AL5B arm
- Channel mapping table (which SSC-32 channel drives which servo)
- Power: 6V supply to the SSC-32 VS1/VS2 rails (document what you used —
  LiPo, bench PSU, wall wart), logic power via USB
- Explain why Raspberry Pi and IMU were not used in this iteration, and note
  the code is portable to the Pi (only the serial port string changes)

## 3. Software architecture
- Module map
  - `ssc32_driver.py` — serial I/O, thread-safe, simulator fallback
  - `al5b_kinematics.py` — geometry constants, FK, IK, angle↔pulse mapping
  - `trajectory.py` — waypoint follower, pick-and-place sequence
  - `webapp.py` — Flask routes + command worker thread + watchdog
  - `templates/index.html` — joystick + sliders UI
- Thread diagram
  - Main thread: Flask HTTP server (one worker thread per request)
  - Command worker: 20 Hz Cartesian→IK→SSC-32 loop
  - Demo thread (spawned on-demand): pick-and-place sequencer
  - All share an `ArmController` guarded by a lock
- SSC-32 protocol notes — `#<ch> P<us> T<ms>`, `Q`, `QP<ch>`, watchdog stop

## 4. Kinematics
- Schematic with joint frames and link lengths (L1, L2, L3, BASE_H)
- Derivation of forward kinematics
- Derivation of geometric IK
  - Base rotation peel-off: θ₀ = atan2(y, x)
  - Wrist position from tool-pitch: (rw, zw) = (r − L3·cos φ, z − BASE_H − L3·sin φ)
  - 2-link planar IK for shoulder and elbow via the law of cosines
  - Wrist angle: θ_wrist = φ − θ_shoulder − θ_elbow
- Servo-angle ↔ pulse mapping (`zero_pulse + us_per_rad · θ`)
- Round-trip validation results from running `python al5b_kinematics.py`
- Reachable workspace (include a scatter plot of reachable points — optional
  but impressive)

## 5. Joystick UI (Task 3 equivalent)
- Screenshot of the UI on a phone
- Flow: touch event → 20 Hz POST to `/api/cmd` → ArmController updates target
  → worker thread runs IK and sends to SSC-32
- Differential-drive analogue explained: the mobile-phone joystick gives (x, y)
  which we mix into Cartesian target (reach, lateral) rather than left/right
  wheel speeds
- Watchdog behaviour: if no update for 1.5 s, the arm freezes in place
- STOP button: calls `stop_all()` which freezes every channel at its current
  pulse via T=1 re-issue

## 6. Pick-and-place (Task 5 equivalent)
- Coordinate system for pick and place points
- Waypoint list from `trajectory.pick_and_place()`
- Timing breakdown (total ~12 s for a full cycle)
- Gripper open/close pulses measured during calibration

## 7. Calibration procedure followed
- How you measured L1/L2/L3/BASE_H
- Per-joint sign determination with the slider UI
- Zero-pulse offsets per joint
- Workspace safety margins chosen

## 8. Individual contributions
- Student A: hardware assembly, calibration, testing
- Student B: driver + kinematics
- Student C: web UI + threading
- Student D: report + demo video
Replace with your actual roles. **This section is explicitly graded
(Teamwork & individual contribution: 5 points).**

## 9. Generative AI usage
The brief (pages 6, 11, 14) explicitly allows generative AI for code. Disclose
honestly:
- Which prompts you used
- What the AI produced vs. what you wrote
- What you debugged / rewrote
(This is asked for in the rubric: "provide implementation details in the
report".)

## 10. References
- Lynxmotion AL5B wiki page
- SSC-32U user guide
- Course lecture slides
- Any tutorials or GitHub repos you consulted

## Appendix A — How to run
Copy the Run section of the README into the appendix.

## Appendix B — Full code listings
Include each Python file with syntax highlighting, or reference the GitHub repo
if the listing would balloon the page count. GitHub Classroom submission link
goes here.
