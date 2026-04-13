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

## The Separating Axis Test Between Convex Polyhedra (GDC 2013)
- **Author:** Dirk Gregorius (Valve)
- **Link:** https://media.gdcvault.com/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf
- **Summary:** Practical guide to implementing SAT for convex hull pairs.
  Covers the three SAT axis families (face normals from hull A, face normals
  from hull B, edge-edge cross products), and the Gauss map optimisation that
  reduces the O(E²) edge-edge search: two edges can only form a valid
  separating axis if their adjacent face normals define intersecting arcs on
  the Gauss map (the "Minkowski face" test). Also covers reference/incident
  face selection and Sutherland-Hodgman clipping for multi-point manifold
  generation.
- **Relevance:** Direct basis for `hull_hull_manifold`, `is_minkowski_face`,
  and the Gauss-map-filtered edge-edge SAT in the mesh collision code. The
  boundary-edge semicircle variant (`is_boundary_edge_minkowski_face`) extends
  the Minkowski face test for mesh edges with only one adjacent face normal.
