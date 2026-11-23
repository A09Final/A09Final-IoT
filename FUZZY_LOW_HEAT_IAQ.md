# Low-temperature IAQ rule revision

## Observed problem

The project owner observed IAQ changing from Good to Severely Polluted or Extremely Polluted soon after the stove started, while the reported stove temperature remained below 150 degrees C. At 32 degrees C, changing presence from detected to not detected changed the condition from Normal to Warning 1.

The sensor position above the stove is a possible contributor. This revision does not confirm that cause or change BSEC, its IAQ scale, calibration, or the sensor position.

## Revised decision

Rules 3, 5, and 7 now support Normal instead of Warning 1. Rules 1 through 7 therefore all support Normal. This prevents low-temperature warnings based only on IAQ and absence, including long absence with Lightly or Moderately Polluted readings. It also avoids making those lower IAQ categories more urgent than the Severe/Extreme categories at the same low temperature.

- At valid stove temperatures from 0 through 150 degrees C, IAQ and absence alone do not raise the fuzzy condition. Presence and its timer must be consistent, as supplied by the firmware.
- The existing overlap from 150 to 160 degrees C remains. Higher-temperature rules can contribute there. This is not a new hard threshold at 150 degrees C.
- Rules 8 through 32, temperature memberships, IAQ memberships, timer memberships, centroid calculation, output score bands, and rule-ID selection remain unchanged.
- Normal refers to this stove-condition assessment. It does not mean that polluted air is clean or that the kitchen is proven safe.
- IAQ telemetry and its displayed category remain unchanged. This revision does not add a separate air-quality notification.

The 32 rule IDs remain stable. IDs 3, 5, and 7 now belong to Normal. The app suppresses Normal CTA; an external rule reference that still lists these three as Warning 1 needs to be updated before it is used as a specification.

## Scope and verification

Run `python3 -B -m unittest discover -s tests -p 'test_fuzzy_*.py' -v` from the repository root. The tests compile the actual `Sensor/FuzzyKompor.cpp` with a desktop C++ compiler. They check the reported presence case, low-temperature IAQ/absence combinations, transition coverage, rule IDs, and retained higher-temperature warning/alarm cases.

The workspace Python simulator in `../uji_fuzzy` uses the same revised rule grouping. Its parity check now reads the IoT C++ module instead of the earlier Arduino learning sketch. Earlier learning copies and handoff archives remain historical snapshots.

This is a prototype rule revision based on the reported behavior. It is not a validated fire-detection boundary. Check it with recorded sensor sessions and the ESP32-S3 before deployment. This change does not alter the physical alarm, backend, app, sensor sampling, or firmware confirmation timing.
