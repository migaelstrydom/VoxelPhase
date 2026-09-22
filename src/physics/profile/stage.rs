/// One timed stage of a physics frame.
///
/// ```text
///   update_contacts (once per frame)      substep (× N per frame)
///   ├─ StaticNarrowphase                  ├─ Integrate  (forces, allowances, positions)
///   ├─ DynamicNarrowphase                 ├─ Solve      (solve + write-backs + projection)
///   ├─ ManifoldMerge                      ├─ CcdSetup   (pre-integration snapshot)
///   ├─ Bookkeeping (sleep, ownership)     ├─ Ccd
///   └─ Conditioning (shock, supports,     └─ SleepUpdate
///      traction, solver prepare)
/// ```
///
/// Substep stages accumulate over every substep of the frame, so a frame's
/// stage times sum to the whole frame's physics cost.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PhysicsStage {
    StaticNarrowphase,
    DynamicNarrowphase,
    ManifoldMerge,
    Bookkeeping,
    Conditioning,
    Integrate,
    Solve,
    CcdSetup,
    Ccd,
    SleepUpdate,
}

impl PhysicsStage {
    pub const COUNT: usize = 10;

    /// Every stage, in the order a frame runs them.
    pub const ALL: [PhysicsStage; Self::COUNT] = [
        PhysicsStage::StaticNarrowphase,
        PhysicsStage::DynamicNarrowphase,
        PhysicsStage::ManifoldMerge,
        PhysicsStage::Bookkeeping,
        PhysicsStage::Conditioning,
        PhysicsStage::Integrate,
        PhysicsStage::Solve,
        PhysicsStage::CcdSetup,
        PhysicsStage::Ccd,
        PhysicsStage::SleepUpdate,
    ];

    pub fn label(self) -> &'static str {
        match self {
            PhysicsStage::StaticNarrowphase => "static_narrowphase",
            PhysicsStage::DynamicNarrowphase => "dynamic_narrowphase",
            PhysicsStage::ManifoldMerge => "manifold_merge",
            PhysicsStage::Bookkeeping => "bookkeeping",
            PhysicsStage::Conditioning => "conditioning",
            PhysicsStage::Integrate => "integrate",
            PhysicsStage::Solve => "solve",
            PhysicsStage::CcdSetup => "ccd_setup",
            PhysicsStage::Ccd => "ccd",
            PhysicsStage::SleepUpdate => "sleep_update",
        }
    }

    /// Whether the stage runs once per substep rather than once per frame.
    pub fn per_substep(self) -> bool {
        matches!(
            self,
            PhysicsStage::Integrate
                | PhysicsStage::Solve
                | PhysicsStage::CcdSetup
                | PhysicsStage::Ccd
                | PhysicsStage::SleepUpdate
        )
    }

    /// Position in [`PhysicsStage::ALL`], and in any per-stage array.
    pub(super) fn slot(self) -> usize {
        self as usize
    }
}
