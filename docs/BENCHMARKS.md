# Benchmarks & Test Protocol

## Official required targets (from problem statement — must be met, non-negotiable)

**Dead Reckoning drift**: < 10% of total distance travelled during GNSS
blackout.
- Example references given: < 5m drift over 50m in < 1 minute, OR
  < 100m drift over 1km at 60kmph (tunnel/underground/simulated GNSS-denied).

**GNSS+INS Fusion update rate**:
- Mobile app: 10Hz
- Edge engine (FOG-grade IMU input): ~200Hz

## Team stretch target (aspirational, report honestly either way)

1–2% drift. This is the goal to aim the engineering at, but it is NOT the
pass/fail bar — the official 10% target is. Never report the 1–2% figure as
an achieved result unless it was actually measured and reproducible. If real
results land between 2% and 10%, that is still a pass and should be reported
as such, plainly, with the actual number.

## What must be measured, and how, at Phase 13

1. **Drift %**: for each test segment, compute
   `drift_% = final_position_error / total_distance_travelled * 100`
   using the ground-truth track (IO-VNBD ground truth for car segments, the
   chosen bridge dataset's reference track for two-wheeler segments — see
   `docs/NOVELTY_SPEC.md` N1). Report car and two-wheeler results separately,
   never averaged together into one misleading number.

2. **Update rate**: measured wall-clock output rate of the fusion engine
   under realistic load on target hardware (actual phone for mobile, target
   edge hardware or closest available proxy for edge), not just theoretical
   loop timing.

3. **Mode-transition latency**: time from GNSS-loss event (or GNSS
   reacquisition) to the fusion engine fully switching trust mode, measured
   in milliseconds from logged timestamps.

4. **NIS gating behavior**: log of accepted vs rejected GNSS updates and the
   chi-squared statistic per update, over at least one test run with induced
   bad fixes (e.g. simulated multipath/jump), to demonstrate the gate
   actually rejects bad updates rather than just existing in code.

## Reporting rules

- Every number in the final report/demo must trace back to a script in
  `/eval` that produced it. No manually-typed benchmark numbers.
- Car and two-wheeler results are always reported separately and labeled.
- If a target is not met, say so plainly in `docs/OPEN_QUESTIONS.md` and the
  final report — do not omit or obscure a failing number.
- Best-case and worst-case results should both be shown, not just the best run.
