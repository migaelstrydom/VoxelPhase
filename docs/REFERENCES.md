# References

External papers, talks, blog posts, and videos reviewed for relevance to this engine.

## Augmented Vertex Block Descent (AVBD)
- **Authors:** Chris Giles, Elie Diaz, Cem Yuksel (SIGGRAPH 2025)
- **Link:** https://graphics.cs.utah.edu/research/projects/avbd/
- **Summary:** Extends VBD (a primal position-space solver) with augmented
  Lagrangian to handle hard constraints and high stiffness ratios. Hybrid
  primal-dual: local Newton solves per body (parallel via graph coloring),
  then dual variable updates. Unconditionally stable, GPU-oriented.
  110K rigid blocks in 3.5ms on RTX 4090 with 4 iterations.
- **Relevance:** Core innovations solve primal-method problems (stiffness
  ratios, hard constraints) that our dual PGS doesn't have. GPU parallelism
  doesn't apply to our CPU solver. Two portable ideas: (1) static friction
  contact point anchoring to prevent tangential drift, (2) warm-start decay
  factor (γ=0.99/frame) vs our fixed scaling.
