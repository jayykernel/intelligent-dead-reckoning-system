# PROJECT CONSTITUTION — READ BEFORE EVERY ACTION

## What this project is
An Intelligent Dead Reckoning (IDR) system with GNSS+INS fusion, for SIH.
Two deliverables: (1) an Android mobile app, (2) a sensor-agnostic edge-deployable
software engine. Full problem context is in `docs/ARCHITECTURE.md`,
`docs/NOVELTY_SPEC.md`, and `docs/BENCHMARKS.md`. Read all three before writing
any code in a new phase.

## Non-negotiable rules (violating any of these is a failure, not a style choice)

1. **No scope expansion.** Do not add features, algorithms, libraries, screens,
   optimizations, or "nice to haves" that are not written in `docs/NOVELTY_SPEC.md`
   or the current phase's entry in `docs/PHASE_CHECKLIST.md`. If you think
   something is missing or would improve the system, STOP and write it as a
   note in `docs/OPEN_QUESTIONS.md` — do not implement it. The person decides,
   not the model.

2. **No scope substitution.** Do not silently replace a specified algorithm
   with a different one because it seemed easier or more familiar (e.g. do not
   swap UKF for a plain complementary filter, do not swap the specified CNN/GRU
   for a generic model, do not skip chi-squared gating because it's fiddly).
   If a specified approach turns out to be genuinely infeasible, STOP, explain
   why in `docs/OPEN_QUESTIONS.md`, and wait for a decision. Do not proceed on
   your own judgment.

3. **One phase at a time, in order.** Work only on the single current phase in
   `docs/PHASE_CHECKLIST.md`. Do not start a later phase early "while you're at
   it." Do not touch files or modules that belong to a future phase.

4. **No fabricated results.** Never report a benchmark number, accuracy figure,
   or "passing" test that was not actually produced by running real code against
   real data. If something can't be validated yet, say so explicitly.

5. **Ask, don't assume, on ambiguity.** If a requirement in the phase checklist
   is ambiguous, stop and ask a single, specific question rather than picking
   an interpretation and continuing.

6. **Every phase ends with a real, working, committed state.** Never leave the
   repo in a broken or half-finished state at the end of a session. If a phase
   cannot be finished in one session, commit the partial work with a clear
   `WIP:` prefix and note exactly what remains in `docs/PHASE_CHECKLIST.md`.

## Execution protocol (follow exactly, every phase)

1. Open `docs/PHASE_CHECKLIST.md`. Find the first unchecked phase.
2. Read that phase's objective, deliverable, exit criteria, and "DO NOT" list.
3. Read the relevant section(s) of `docs/ARCHITECTURE.md` and, if the phase
   touches a novelty feature, the matching section of `docs/NOVELTY_SPEC.md`.
4. Implement only what that phase specifies. Nothing else.
5. Run the phase's exit-criteria checks yourself (tests, plots, benchmark
   scripts as specified). Do not mark a phase done on the basis of code
   compiling alone — it must meet the stated exit criteria.
6. If exit criteria pass: check the box in `docs/PHASE_CHECKLIST.md`, commit
   with the exact commit convention below, and stop. Report to the person what
   was done and what the next phase is. Do not auto-continue to the next phase.
7. If exit criteria fail: do not check the box. Fix within the current phase's
   scope only. If you cannot fix it without exceeding scope, stop and report why.

## Git commit convention (mandatory, every phase)

```
[Phase N] <short imperative summary>

- what was implemented
- exit criteria met (list each, with the actual number/result)
- files touched
```

One commit per phase minimum (multiple WIP commits inside a phase are fine).
Never combine two phases into one commit. Never commit code for a phase that
hasn't been reached yet.

## Repository structure (fixed — do not reorganize)

```
/docs                     architecture, phase checklist, novelty spec, benchmarks
/data
  /raw                     IO-VNBD + any collected data, untouched
  /processed               cleaned/segmented data used for training
/training                  offline model training code (cloud/desktop side)
  /models                  saved trained models (checkpoints, .tflite exports)
/engine                    the core sensor-agnostic fusion engine (shared library)
  /calibration
  /ai_filters
  /nhc_zupt
  /fusion
  /map_matching
  /outage_prediction
/mobile                    Android app (consumes /engine via TFLite + shared logic)
/edge                      edge-deployable engine packaging (consumes /engine)
/eval                      benchmark scripts, drift measurement, plots
/screening_package         the screening-round deliverable (model + plots + writeup)
```

**New Rule**: No script, test, or debug file may ever be created at the repository root. Debug and ad-hoc investigation scripts belong in the relevant module's `tests/` subfolder (e.g. `engine/fusion/tests/`, `engine/calibration/tests/`) — create that subfolder if it doesn't exist yet, matching the module the code under investigation belongs to. This applies for the rest of the project, all remaining phases, no exceptions.

Do not create alternate top-level folders. Do not restructure this layout
mid-project even if it seems cleaner — raise it in `docs/OPEN_QUESTIONS.md` first.

## Tech stack (locked — do not substitute without explicit approval)

- Language (engine core): Python for training, C++/Kotlin for on-device inference
  glue as needed per phase.
- On-device model format: TensorFlow Lite (TFLite).
- Mobile platform: Android (Kotlin), scoped Android-first per `docs/NOVELTY_SPEC.md`
  (predictive outage detection depends on Android's raw GNSS measurements API).
- Fusion filter base: EKF/UKF strapdown mechanization (classical, not learned)
  as the backbone; AI models correct/augment specific sub-problems as defined
  in `docs/NOVELTY_SPEC.md`. The fusion filter is never fully replaced by a
  black-box neural network.
- Map data: OpenStreetMap (offline extract).
- Dataset: IO-VNBD for cars; see `docs/NOVELTY_SPEC.md` for the two-wheeler
  data plan — do not substitute a different primary dataset.

## What "done" means for the whole project

All phases in `docs/PHASE_CHECKLIST.md` checked, all exit criteria met with
real measured numbers, screening package produced at its checkpoint, and the
final benchmark validation phase passing the official targets in
`docs/BENCHMARKS.md`. Nothing more, nothing less.
