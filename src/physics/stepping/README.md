# Stepping

Controls the overall physics step loop structure.

## Architecture

```
Stepper (trait)             FixedTimestep (reusable component)
├── step()                  ├── accumulate(frame_dt) -> substep count
└── fixed_dt()              └── fixed_dt()

SequentialStepper (implementation)
├── owns FixedTimestep
└── step():
    1. Accumulate frame time
    2. update_contacts() — once
    3. substep() — N times
```

Different solver strategies require different stepping patterns:

| Strategy | Pattern |
|----------|---------|
| PGS / PGS+NGS | Contacts once, substep N times (SequentialStepper) |
| TGS / Soft Step | Regenerate contacts each substep (InterleavedStepper) |
| XPBD | Predict positions, solve constraints, derive velocities |

## Files

| File | Purpose |
|------|---------|
| `stepper.rs` | `Stepper` trait + `StepResult` |
| `fixed_timestep.rs` | `FixedTimestep` — reusable time accumulator with spiral-of-death clamping |
| `sequential.rs` | `SequentialStepper` — contacts once, substep N (PGS/PGS+NGS pattern) |

## Adding a new stepper

1. Create a new file (e.g., `interleaved.rs`).
2. Implement `Stepper`, composing `FixedTimestep` for time management.
3. Re-export from `mod.rs`.
4. Pass to `PhysicsResource::new()`.

```rust
let stepper = Box::new(InterleavedStepper::new(1.0 / 240.0, 8));
let resource = PhysicsResource::new(world, stepper);
```

## FixedTimestep

`FixedTimestep` is a standalone component used by steppers, the bench harness,
and the bench viewer. It handles:
- Accumulating variable-rate frame time
- Draining in fixed-size chunks
- Clamping to a max substep budget (prevents spiral of death)
