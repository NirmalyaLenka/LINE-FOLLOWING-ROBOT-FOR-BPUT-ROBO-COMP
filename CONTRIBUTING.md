# Contributing

This repository documents one specific robot. Contributions should keep the documentation and
the code in agreement.

## Ground rules

- **Do not change the pin map** in `src/main.ino` unless the hardware changes. Update
  `hardware/PINOUT.md`, `docs/WIRING.md` and the README table in the same change.
- Keep variable names. The documentation uses the sketch's names (`g_stopEffMm`, `TURN_RIGHT_MS`, ...).
- Do not present timing values as universal. Say what they were derived from and mark values that
  the builder must measure as `USER MUST MEASURE`.
- Separate measured values, calculated values and tuned values.
- Do not document features the code does not implement.
- If you change an algorithm, update `docs/` and `diagrams/` in the same pull request.

## Reporting issues

Include: the commit, the board package version, the serial log (use `d` for the debug stream), your
robot's dimensions, `DRIVE_MM_PER_SEC`, `TURN_RIGHT_MS`, `TURN_LEFT_MS`, battery state and the floor surface.

## Hardware changes

If you adapt the robot, describe it in `docs/ROBOT_SIZE_ADAPTATION.md` terms: which constants
changed and the measured values.

## Testing

Run the sequence in the README testing procedure on a real robot before submitting changes to
control or obstacle logic. Start at low speed.

## Style

Plain engineering language. No marketing wording.
