# Cross-host DROID mock result and candidate update — 2026-10-24

Source report: `~/yifan/droid-deployment/from-inference/2026-10-24-crosshost-mock/REPORT.md`.
The copied report, episodes and metrics remain unchanged. Original workstation
location: `/shared/projects/state/runs/2026-10-24-crosshost-mock/`.

The requested RT mock PID 3205047 was stopped with SIGTERM after the test. Port
10.77.0.2:47700 is no longer listening. No robot connection or network setting was
changed by this review. These are mock results, not physical timing qualification.

## Results

| Metric | 10-step episode | 30-step episode |
|---|---:|---:|
| Accepted / applied | 9 / 9 | 28 / 28 |
| Release / fault | clean / none | clean / none |
| Clock uncertainty | 338 µs | 339 µs |
| Capture → receipt p95 | 109 ms | 109 ms |
| Accepted → applied median (mock) | 5.3 ms | 5.2 ms |
| Max row-0 lead | 0.263 rad | 0.289 rad |
| Old candidate row-speed rejection among accepted | 5 / 9 | 8 / 28 |
| New candidate row-speed rejection among accepted (offline replay) | 0 / 9 | 0 / 28 |

Replay checks the seven actual row-to-row joint differences at the contract's
15 Hz rate, on accepted chunks only. Overall maximum accepted row speed was
0.752968 rad/s. Replay receipt:
`~/yifan/droid-deployment/validation/crosshost-speed-audit-2026-10-24.json`.

## Target-speed decision

The pi0.5 **absolute-position action contract has no speed-limit constant**.
The pinned DROID FR3 client configuration declares joint velocity ceilings of
`[2.075,2.075,2.075,2.075,2.51,2.51,2.51]` rad/s. These are DROID robot-client
reference settings, not observed policy velocities or a pi0.5 output normalization.

The non-executable candidate now adopts those values for
`acceptance.max_consecutive_row_speed_rad_s`. Its source ledger, retained
source snapshots and regeneration script were updated accordingly. The old
candidate is retained at `commissioning/2026-10-24-sourced/`; the revised host
copy is `commissioning/2026-10-24-crosshost-reviewed/` under droid-deployment.
The portable repository copy remains `configs/fr3/droid-source-review/`.

**Measured-speed guards remain 0.435 rad/s J1–4 and 0.522 J5–7.** No controller
gains, torque/contact/stop guards, row-0 lead, mock dynamics or live robot settings
were changed. A position-reference derivative is not the measured encoder velocity.
Higher target admission removes these rejection causes; it does not ensure that
physical motion remains under the lower measured-speed guards. A physical trial
may still stop on measured speed and requires its own reviewed operating bounds.
Do not report this replay as a successful physical execution or fix it by silently
raising all guards to upstream maxima.

The mock ran with its existing 1 rad/s row bound and kinematic approach model.
It was not rerun with the new physical candidate settings. Do not rewrite that
configuration after the fact when reproducing these episodes.

## Timing decisions to return to inference workstation

- Keep the existing 2000 µs clock-uncertainty admission bound; 339 µs was measured.
- Recommend testing `--replan-after-rows 5` next. At 15 Hz this launches inference
  around 333 ms into a 533 ms horizon; the recorded ~110 ms capture-to-receipt
  latency leaves more slack than replanning after six rows. This recommendation
  has not been tested in these episodes.
- Keep the 200 ms hold allowance and 700 ms command window pending further data.
  Native callback/consumption/stop margins remain unqualified by the mock's 5 ms
  applied delay. No physical timing fields were tightened on the basis of this run.
- Preserve the recorded revisions and commit the workstation's currently dirty
  robot-learning/sensors changes before the next reproducibility run.
- Replace the shared development token with a privately coordinated random token
  before D7. This review did not change either machine's token.

## Test-count reconciliation at dfeb409

No franka-runtime tests are missing merely because the handoff said 230.
That number combined two repositories. Reproduced on the RT host:

| Scope | Passed | Skipped |
|---|---:|---:|
| `franka-runtime/tests` | 163 | 2 |
| pinned `robot-learning/packages/contracts/tests` | 67 | 0 |
| Combined command used by the RT handoff | 230 | 2 |

The external contracts tests are not stored in the franka-runtime commit:

| External test module | Cases |
|---|---:|
| test_v0_contracts.py | 17 |
| test_droid_contract.py | 15 |
| test_clock_state.py | 11 |
| test_samples_wire.py | 8 |
| test_libero_contract.py | 7 |
| test_sessions.py | 5 |
| test_droid_session.py | 3 |
| test_import.py | 1 |

The 67 tests belong to robot-learning commit
`08d64c65ea06639dda55b3cdcb2459a7784e4c85`. The two runtime skips are the local
M4/PPO contract modules absent from upstream contracts 0.0.2. The handoff should
have stated its combined scope explicitly; a normal franka-runtime-only CI run
should report 163 passed + 2 skipped, not 230.

Receipts: `validation/pytest-dfeb409-runtime-only.xml` and the original combined
`validation/pytest-sourced-review-installed.xml` under `~/yifan/droid-deployment/`.
None of these Python counts includes the separately executed C++ test suites or
establishes hardware qualification.
