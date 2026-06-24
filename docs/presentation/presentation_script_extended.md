# Extended Presentation Script & Speaker Notes

This document complements `presentation_script.md`. The shorter script
gives you a tight 7-minute talk; this one gives you a deeper reservoir
to draw from when you have extra time, want to elaborate on a point,
or need to answer questions.

How to use it:

* **Each slide section** has the concise spoken text from the main
  script (in the `>` blockquote), followed by **expanded notes** — extra
  context, deeper reasoning, optional tangents, and key numbers worth
  having in your head.
* The **anticipated Q\&A section** at the end groups likely audience
  questions by topic with model answers.
* The **quick-reference appendix** lists the numbers most likely to come
  up — keep it open during the talk.

---

# Part I — Extended slide-by-slide notes

## Slide 1 — Title

> Hi everyone. I'm Param Chokshi, and this is my CISC 601 project on
> comparing four numerical interpolation methods for robot trajectory
> planning. I'll show you which one to pick, and one surprise that
> flipped my own intuition halfway through.

**Expanded notes**

* Robotics is a natural application for chapters 18, 24, and 28 of
  the textbook because robots move through *waypoints* — discrete
  positions a higher-level planner has decided the robot must pass
  through.
* The problem of going from sparse waypoints to executable motion is
  called *trajectory planning* and is its own active research area
  (see Biagiotti & Melchiorri 2008). My contribution is a
  controlled comparison of the textbook methods on realistic
  scenarios, with a methodological correction at the end.
* If asked why you picked this topic: numerical methods we covered
  in class show up in robotics every time the robot has to follow a
  path. It is one of the more visible engineering applications of
  the textbook material.

## Slide 2 — Agenda

> Five things in seven minutes: motivation, the four methods, the
> experiment, the surprise, and what to actually use.

**Expanded notes**

* If you're running long, you can skip directly from the motivation
  section to the surprise (slides 28–29) and still deliver the full
  message.
* If you have a senior audience or robotics-aware professor, you can
  flag here that the surprise is a methodological one rather than a
  novel mathematical result.

## Slide 3 — Motivation divider

(Visual break — no script. If the audience seems disengaged, this is
a good place for an aside: "I'll try to keep this concrete — every
slide ties back to a real motion you'd want a robot to execute.")

## Slide 4 — Robots move through waypoints

> A path planner gives a robot a sparse list of waypoints — corners
> of an aisle, say. The trajectory generator has to fill in the
> motion in between. The way it interpolates directly affects motor
> wear, payload stability, and whether the trajectory is even
> physically possible.

**Expanded notes**

* Concrete example: a warehouse mobile robot might pick up a tote
  somewhere, navigate around shelving, and drop it at a sorting
  station. The path planner emits 10–20 waypoints; the trajectory
  generator turns them into commanded $(x, y)$ versus time.
* Concrete second example: a robot arm executing a pick-and-place
  cycle. The waypoints are joint angles at "home", "approach",
  "grasp", "lift", and so on. The robot has to find a smooth motion
  between them.
* If asked who decides the waypoints: typically a path planner
  separate from the trajectory generator. RRT, A\*, and PRM are
  common choices in robotics.

## Slide 5 — The smoothness budget

> Three rules. Discontinuous velocity needs infinite acceleration —
> impossible. Discontinuous acceleration causes torque jumps and
> vibration. Discontinuous jerk excites mechanical resonance. Each
> method buys different parts of this budget at different costs.

**Expanded notes**

* The "budget" framing is mine; it's a useful mental model because
  every method has to spend its degrees of freedom *somewhere*.
* If asked what jerk is: jerk is the third time derivative of
  position — the rate of change of acceleration. It's the quantity
  most directly responsible for how an unsecured payload feels in a
  vehicle. Self-driving cars and elevators are designed around jerk
  limits.
* Snap (the fourth derivative) is real but rarely a design
  constraint outside high-end machine-tool work.
* Macfarlane & Croft (2003) is a good citation if anyone wants to go
  deeper on jerk-bounded planning.

## Slide 6 — Methods divider

(Visual break.)

## Slide 7 — Four methods compared

> The four methods, straight from chapters 18 and 24. Linear: C-zero
> continuity, position only. Cubic spline: C-two — continuous
> velocity and acceleration. Quintic spline: C-four — even jerk and
> snap continuous. Cubic B-spline: C-two everywhere, but doesn't
> pass through the interior waypoints. I implemented all four from
> the textbook equations, validated to machine precision against
> SciPy.

**Expanded notes**

* **Linear**: just $\vec{p}(t) = \vec{p}_i + (\vec{p}_{i+1} -
  \vec{p}_i)/(t_{i+1} - t_i) \cdot (t - t_i)$. Equation 18.2 in the
  book.
* **Cubic spline**: each segment is a cubic polynomial with
  continuous first and second derivatives. The textbook formulation
  (Equation 18.36) writes each segment in terms of the second
  derivatives at the knots, which gives a tridiagonal system for
  those second derivatives.
* **Quintic spline**: I used the *Hermite* formulation. Each segment
  is a quintic polynomial parameterised by position, velocity, and
  acceleration at each end. The unknowns are velocity and
  acceleration at every knot; continuity of the third and fourth
  derivatives gives the equations.
* **Cubic B-spline**: built using the Cox–de Boor recursion. Knot
  vector is "clamped" — the first and last knots are repeated, so
  the curve passes through the first and last waypoints exactly,
  but the interior waypoints are only *control points* that the
  curve approximates.
* If asked about validation: I generated test data from a known
  fifth-degree polynomial and confirmed the quintic spline
  reproduces it to about $10^{-13}$. I also compared the cubic and
  B-spline against scipy.interpolate.CubicSpline and
  scipy.interpolate.BSpline on identical inputs — agreement was at
  machine precision (around $10^{-15}$).

## Slide 8 — Methodology summary  *(optional)*

> Two robotics datasets, three sweep axes — density, geometry,
> spacing — gives seven variants and twenty-eight experiments. Six
> metrics each. Plus a separate validation against centered finite
> differences from chapter 28.

**Expanded notes**

* The seven variants are: mixed geometry at $n=5, 12, 20$; sharp
  zigzag at $n=12$; gradual sinusoid at $n=12$; mixed geometry at
  $n=12$ with uniform spacing instead of chord-length; and the robot
  arm baseline.
* Six metrics: peak speed, peak acceleration, root-mean-square jerk,
  total path length, maximum waypoint deviation, and compute time.
  A seventh metric — maximum velocity discontinuity — was added
  later to make the C-one violation visible quantitatively.
* The whole experiment runs end-to-end in about 30 milliseconds.

## Slide 9 — Datasets divider

(Visual break.)

## Slide 10 — Two scenarios

> Two scenarios. On the left, a 12-waypoint mobile robot path
> through a warehouse aisle, with two sharp 90-degree turns and an
> S-curve. On the right, an 8-waypoint pick-and-place for a 2-DOF
> planar arm. The arm is interesting because interpolation happens
> in joint space but motion happens in Cartesian space — and that
> mapping is non-linear.

**Expanded notes**

* The mobile-robot path was hand-designed to mix three classes of
  geometric features: straight runs, sharp turns, and a smooth
  S-curve. Sharp turns stress every smooth interpolant; the S-curve
  is the easy case. Including both lets us compare each method
  under both stress and benign conditions in a single trajectory.
* The robot arm has link lengths $L_1 = 0.5$ m and $L_2 = 0.4$ m, so
  the reachable workspace is a circle of radius 0.9 m. Every
  Cartesian waypoint lies inside that workspace.
* If asked why 2-DOF specifically: it's the simplest non-trivial
  manipulator and forward kinematics is exact in closed form. A
  6-DOF arm would generalise the result but add a lot of
  configuration overhead without changing the substantive
  conclusions.

## Slide 11 — Baseline divider

(Visual break.)

## Slide 12 — Mobile robot trajectory

> All four methods on the same waypoints. Gray is linear — the
> polygon. Cubic and quintic both pass through every waypoint and
> round the corners. Red is the B-spline; you can see it visibly
> cuts the corners — that's the approximation property.

**Expanded notes**

* The B-spline's corner cutting is the *defining* property of an
  approximating spline. The interior waypoints are not interpolation
  conditions, they're *control points* that pull the curve in their
  direction.
* For applications where the waypoints are strict (like picking up
  a part at an exact location), the B-spline's corner cutting is a
  failure mode. For applications where the waypoints are soft
  (like obstacle-avoidance buffers), it's a feature — the robot
  takes a smoother path that still respects the spirit of the
  waypoints.

## Slide 13 — Mobile robot kinematics

> Now the kinematics over time. Linear shows zero analytic
> acceleration and zero jerk — that's mathematically true on the
> open intervals, but it hides infinite spikes at every waypoint.
>
> The cubic spline jerk is the obvious step pattern in blue. That's
> the textbook signature: piecewise-cubic means jerk is constant on
> each segment and jumps at every internal knot.
>
> The quintic jerk is continuous, as theory promises, but you can
> see those big spikes at the start and end. We'll explain those.

**Expanded notes**

* The "infinite spikes" interpretation of linear is the
  distributional second derivative — Dirac impulses at every
  waypoint. Mathematically: if you take centered finite
  differences of the linear-interpolated trajectory, you'll see
  acceleration values that *grow without bound* as you sample more
  finely.
* The cubic jerk pattern is a great visual diagnostic — if you
  ever see a jerk profile shaped like a staircase, you're looking
  at a piecewise-cubic interpolant. Quintic gives you continuous
  ramps; B-spline gives you smooth curves.
* Linear's apparent advantage on the metrics (zero acceleration,
  zero jerk) is the hardest pedagogical point in the talk. Be
  prepared to explain it twice.

## Slide 14 — Robot arm Cartesian

> Same eight joint waypoints, very different Cartesian paths.
> Forward kinematics is non-linear, so small differences in joint
> space get amplified.

**Expanded notes**

* The forward kinematics for a 2-DOF arm:
  $x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)$,
  $y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)$.
  This is a sum of cosines, so it's gently non-linear in
  $\theta_1, \theta_2$.
* The natural-BC quintic produces visible loops near waypoints 2
  and 5–6. Those loops appear because in joint space the velocity
  has to ramp up to nominal speed quickly; in Cartesian space the
  same joint-velocity profile traces a curve.

## Slide 15 — Per-joint derivatives  *(optional)*

> Drilling into one joint at a time makes the cubic's piecewise-
> constant jerk and the quintic's continuous-but-spiky jerk
> obvious side-by-side.

**Expanded notes**

* The figure is a 4-by-2 grid: rows are derivative orders 0, 1, 2,
  3 (position, velocity, acceleration, jerk); columns are
  $\theta_1, \theta_2$.
* If asked about why jerk on $\theta_2$ has bigger spikes than on
  $\theta_1$: the second joint angle has a larger range of motion
  (it goes from 0 to roughly $-\pi/2$ and back, multiple times,
  while $\theta_1$ has a smaller swing).

## Slide 16 — Quantitative summary

> The numbers. Three things to flag. One: linear's zero
> acceleration is a mathematical artifact, not the physics. Two:
> the B-spline gets the smoothest derivatives — but it's 0.68
> metres off from the requested waypoints, which is a real
> failure if you actually need to reach those positions. Three —
> and this is the puzzle — the quintic spline has *higher* peak
> acceleration and RMS jerk than the cubic. Quintic has more
> continuity. So how is it less smooth? Hold that thought.

**Expanded notes**

* The B-spline waypoint error of 0.68 m is roughly 7% of the
  bounding-box diagonal of the path. That's a lot.
* On the robot arm the B-spline error is 0.40 rad, or about 23
  degrees of joint-angle deviation — even worse in relative terms.
* The quintic puzzle is real and not an implementation bug — I
  validated the quintic against a known fifth-degree polynomial
  before drawing this comparison.

## Slide 17 — Variant divider

(Visual break.)

## Slide 18 — Three axes  *(optional)*

> To draw general conclusions I varied three axes one at a time:
> waypoint count, geometric character, and time-spacing rule.

**Expanded notes**

* The three axes are designed to be independent so each effect can
  be isolated. Density holds geometry and spacing fixed; geometry
  holds density and spacing fixed; etc.
* The full sweep is 28 (method, dataset) combinations: 7 datasets
  $\times$ 4 methods. Every metric is computed for every cell.

## Slide 19 — Density sweep

> First surprise. Going from 5 to 12 to 20 waypoints on the same
> geometry, peak acceleration and RMS jerk *increase* monotonically
> for every spline. The intuition that more waypoints means
> smoother motion is just wrong here. Chord-length spacing keeps
> nominal speed fixed, so denser waypoints means smaller segments
> and bigger curvatures. Density buys geometric fidelity at the
> cost of smoothness.

**Expanded notes**

* The mechanism: spline curvature scales like $1/h_i^2$ where
  $h_i$ is the segment width. Denser waypoints means smaller
  $h_i$, so curvature can blow up.
* The effect is not pathological — the splines aren't ill-
  conditioned, they're just satisfying more constraints in less
  space.
* The "right" number of waypoints depends on what you care about.
  If geometric fidelity matters more (the polyline is the truth),
  you want denser. If smooth motion matters more (the polyline is
  a guideline), you want sparser.
* This is a genuinely surprising result — most students arrive
  with the opposite intuition.

## Slide 20 — Density visuals  *(optional)*

> Visually you can see it: at n=5 every method makes obvious
> geometric mistakes; at n=20 they all converge to the polyline.

## Slide 21 — Density numbers  *(optional)*

> The numerical version of the same trade-off.

**Expanded notes**

* The numerical table makes the doubling-of-jerk argument concrete.
  Roughly: $n: 5 \to 12 \to 20$ produces RMS jerk values of
  approximately $0.13 \to 0.34 \to 0.88$ for the cubic, and $0.27
  \to 0.85 \to 1.91$ for the quintic.

## Slide 22 — Geometry sweep

> Geometry has a much bigger effect than density. Sharp 90-degree
> corners roughly double peak acceleration and triple RMS jerk for
> the cubic and quintic. The B-spline absorbs sharp geometry
> gracefully — its sharp-versus-gradual ratio is 1.18, compared to
> 1.78 for the cubic. So if your input has sharp features,
> B-splines buy you the most smoothness.

**Expanded notes**

* The numbers for context: cubic on the smooth sinusoid at $n=12$
  has peak acceleration of about $1.19$ m/s$^2$; on the sharp
  zigzag at the same $n$ it's about $2.12$ — almost double.
* The B-spline ratio of 1.18 vs. cubic's 1.78 is the strongest
  practical case for B-splines.

## Slide 23 — Geometry visuals  *(optional)*

> Visually: cubic and quintic overshoot at every sharp corner;
> B-spline cuts the corner.

## Slide 24 — Spacing sweep  *(optional)*

> Time-spacing has the smallest effect. Chord-length versus uniform
> stays within about 30 percent on every metric.

**Expanded notes**

* Chord-length is the textbook recommendation (Section 24.1).
  Uniform is what you get if you naively sample the waypoints at
  equal time intervals.
* The most visible effect is on peak speed. Under chord-length,
  linear interpolation has an exactly 1 m/s speed envelope. Under
  uniform, it has to traverse longer segments in the same time
  budget as shorter ones, so peak speed jumps to about 1.32 m/s.

## Slide 25 — All variants  *(optional)*

> Pulling it together — the smoothest method depends strongly on
> the variant. There's no universal winner.

## Slide 26 — Validation divider  *(optional)*

(Visual break.)

## Slide 27 — Analytic vs finite-difference

> Quick validation note. To check the analytic derivative formulas
> are right, I also computed derivatives by centered finite
> difference — the schemes from chapter 28. Velocity and
> acceleration overlap perfectly. The cubic-spline jerk gap is
> real, not a bug — it's the finite-difference scheme failing to
> resolve the actual step discontinuities.

**Expanded notes**

* I used the 5-point centered formula (Equation 28.3 in the
  table) for the first derivative — it's $O(h^4)$ accurate. For
  the second derivative I used the 3-point centered formula
  (Equation 28.4), which is $O(h^2)$. For the third derivative I
  used the centered 5-point stencil, also $O(h^2)$.
* The cubic-spline jerk error of about $0.5$ corresponds to the
  height of the worst step discontinuity in the dataset. If you
  zoom into the figure you can see the FD scheme smoothing each
  step into a short ramp of width about $5h$.
* The quintic spline has *continuous* jerk, so the FD scheme
  agrees with the analytic to about $9 \times 10^{-4}$. This is
  the validation that the analytic formula is correctly
  implemented.

## Slide 28 — Why quintic looked worse

> Now back to the quintic puzzle. Both splines used "natural"
> boundary conditions — start and stop at rest. But chord-length
> parameterisation implies a steady one-metre-per-second nominal
> speed everywhere else. So the quintic has to ramp from rest up
> to one metre per second within the very first segment, and back
> down in the last. That ramp is what creates the big jerk spikes
> you saw earlier.
>
> So the hypothesis is: the quintic isn't actually worse — the
> *boundary condition* is. Let's give it the cubic spline's end
> velocities and accelerations as clamped values instead.

**Expanded notes**

* The natural BC for the cubic spline only constrains
  *acceleration* to be zero at the endpoints. The natural BC for
  the quintic constrains both *velocity* and *acceleration* to
  zero. That's two extra constraints — two more degrees of freedom
  the quintic has to spend at the boundaries.
* Equivalently: the cubic chooses its boundary first derivative
  freely, but the quintic must start at rest. Forcing rest plus
  reaching nominal speed is a much harder problem in a tight first
  segment.
* If asked whether this is well-known: it is in some specialised
  trajectory-planning circles (Macfarlane & Croft 2003), but it's
  rarely the first thing taught about quintic splines. Most
  textbooks either don't compare cubic and quintic directly or
  paper over the boundary-condition issue.

## Slide 29 — Quintic boundary-condition fix

> Here's the comparison. Green is the original natural-boundary
> quintic. Purple is the same quintic with cubic-derived clamped
> boundary conditions. Peak acceleration drops 33 percent. RMS
> jerk drops 64 percent. And now the quintic *beats* the cubic on
> both metrics — which is what you'd expect from a higher-
> continuity method. The original result was a boundary-condition
> artefact, not a property of quintic splines.

**Expanded notes**

* Specific numbers: natural quintic max accel = $1.49$, clamped
  quintic max accel = $1.00$ — that's the 33% drop. Natural quintic
  RMS jerk = $0.85$, clamped = $0.31$ — that's the 64% drop.
* For comparison, cubic max accel was $1.18$ and cubic RMS jerk
  was $0.34$. So the clamped quintic beats the cubic on both —
  $1.00$ vs $1.18$ on accel, $0.31$ vs $0.34$ on jerk.
* This is a small but meaningful methodological correction. Anyone
  publishing a comparison of spline interpolation should explicitly
  state the boundary condition.

## Slide 30 — Recommendations divider

(Visual break.)

## Slide 31 — Decision rule

> So here's the practical guidance. Strict waypoints with known end
> velocities — clamped quintic spline. Strict waypoints, start and
> stop at rest — natural cubic spline; *avoid* the natural-boundary
> quintic. Soft waypoints with intentionally sharp features —
> cubic B-spline. Linear is for debugging only.

**Expanded notes**

* "Known end velocities" is more common than people think — any
  time you're stitching trajectory segments together, the previous
  segment provides the start velocity for the next one.
* "Start and stop at rest with no other prior" is the easiest case
  in some ways — robot is paused before and after — but it's also
  where the natural-BC quintic falls down.
* The "soft waypoints" recommendation for B-splines applies to
  collision avoidance buffers, comfort-driven trajectories, and
  similar smoothing tasks where the planner emits indicative rather
  than prescriptive waypoints.

## Slide 32 — Eight findings  *(optional)*

> The eight findings in one minute, if you want them: waypoint
> adherence is binary; linear has hidden C-one violations; more
> waypoints can hurt smoothness; geometry dominates; spacing barely
> matters; the quintic puzzle was a boundary-condition artefact;
> derivatives are validated against finite differences; and compute
> cost is sub-millisecond.

## Slide 33 — Limitations / future work  *(optional)*

> Honest limitations: synthetic data, kinematic metrics only, two
> BC variants explored. Natural extensions for the final paper.

**Expanded notes**

* Synthetic data is fine for a controlled comparison but doesn't
  prove anything about real-world planner outputs. A natural next
  step is to run the same analysis on a corpus of recorded
  trajectories from a real robot.
* Kinematic metrics ignore actuator saturation, mechanical resonance
  frequencies, and payload dynamics. A weighted composite metric
  that includes these would change the rankings — particularly
  pushing the quintic up further because of its smoother torque
  profile.

## Slide 34 — Thank you / Questions

> That's the talk. Full code, data, and notebook are in the
> repository. Happy to take questions.

---

# Part II — Anticipated Q&A

These questions and answers are organised by topic. The student
audience is most likely to ask about the surprising findings;
faculty are more likely to probe the methodology.

## On the methods themselves

**Q: Why didn't you just use scipy.interpolate?**

> The proposal explicitly required implementing each method from the
> textbook formulation. The point isn't reproducing scipy — it's
> demonstrating understanding of the underlying math. That said, I
> *did* use scipy as a reference: my cubic and B-spline
> implementations match scipy's to machine precision, which is the
> validation step.

**Q: What's the Cox--de Boor recursion?**

> It's a recursive formula for evaluating B-spline basis functions.
> The idea is that a B-spline basis function of degree $p$ is
> defined as a weighted combination of two basis functions of
> degree $p-1$, with the recursion bottoming out at degree-zero
> indicator functions on the knot intervals. It's the standard way
> to evaluate B-splines and it's numerically very stable.

**Q: Why a cubic B-spline specifically? Why not higher degree?**

> Cubic B-spline gives $C^2$ continuity, which is sufficient for
> the smoothness comparison. Higher-degree B-splines would give
> more continuity but at the cost of stronger smoothing of the
> waypoint polygon. The point of including a B-spline at all was
> to study the approximation-versus-interpolation trade-off, and
> cubic is the standard choice in robotics literature.

**Q: What about NURBS?**

> NURBS (Non-Uniform Rational B-Splines) generalise B-splines by
> attaching a weight to each control point and dividing through.
> They're standard in CAD/CAM for representing exact conic
> sections. For trajectory planning the rational form doesn't add
> much, so I stuck with plain B-splines. Adding NURBS is a natural
> extension for Deliverable 4.

**Q: What's a Hermite basis?**

> A Hermite basis is a set of polynomials chosen so that each one
> is "active" for a specific boundary condition. For the quintic
> Hermite, $H_0$ is 1 at one end and 0 at the other with all
> derivatives zero; $H_1$ has derivative 1 at one end and zero
> elsewhere; and so on. So writing a polynomial as $H_0 p_i + H_1
> h v_i + H_2 h^2 a_i + \dots$ is just expressing it in terms of
> its boundary values rather than its monomial coefficients.

## On the results

**Q: Why is RMS jerk a meaningful metric? Why not max jerk?**

> Both are useful, and the experiment outputs both. RMS jerk
> captures the *typical* jerk over the trajectory — it's
> insensitive to a single brief spike. Max jerk captures the worst
> case. For applications dominated by mechanical fatigue, RMS is a
> better proxy because fatigue accumulates over time. For
> applications dominated by single-event payload disturbance, max
> jerk is the right metric. I report both in the metrics CSV.

**Q: Could linear interpolation be saved by post-smoothing?**

> Yes, in practice that's exactly what happens — controllers often
> apply a low-pass filter to the commanded position. But that's
> equivalent to introducing a smoothing time constant, which is
> what the spline methods do *natively* and with explicit control.
> So you've reinvented the spline; you might as well use a spline
> directly.

**Q: What's the right number of waypoints?**

> It depends on what you care about. From the density sweep: more
> waypoints buys geometric fidelity (path length converges to the
> polyline) but worsens smoothness (peak acceleration and jerk
> grow). For the canonical mobile-robot scenario, $n=12$ was a
> reasonable balance. If your geometry has features at a particular
> length scale, you want enough waypoints to resolve them but not
> so many that you lose smoothness.

**Q: Why didn't the time spacing matter much?**

> The smooth methods compensate. Cubic and quintic splines have
> enough degrees of freedom that they can adjust their derivatives
> at internal knots to match whatever speed the time-stamping
> implies. Linear can't compensate, so the spacing effect shows up
> there as a peak-speed difference. The lesson is that
> chord-length parameterisation matters more for the linear
> interpolant than for the splines.

## On validation and theory

**Q: How did you know the analytic derivatives were right?**

> Two checks. First, I generated test data from a known fifth-
> degree polynomial and confirmed every method reproduces the
> derivatives to floating-point precision. Second, I compared the
> analytic derivatives against centered finite differences from
> chapter 28. They agree everywhere except where the analytic
> derivative has a step discontinuity (the cubic-spline jerk),
> which is the expected behaviour.

**Q: Why centered finite difference and not something more accurate?**

> The point of the validation was to use *exactly* the textbook
> formulas, not the most accurate available. The 5-point centered
> formula for the first derivative is $O(h^4)$ which is plenty for
> validation purposes — the resulting agreement is at the $10^{-5}$
> level, far below any of the kinematic differences between
> methods.

**Q: What's the convergence rate for cubic splines?**

> Standard result: cubic splines converge as $O(h^4)$ in the
> $L^\infty$ norm for smooth functions. The error in the second
> derivative converges as $O(h^2)$. None of this is in tension with
> the density-sweep result, because the density sweep isn't
> measuring approximation error — it's measuring the *magnitude*
> of the second derivative on a fixed underlying geometry, which
> grows as $h$ shrinks.

**Q: What's the Runge phenomenon and is it relevant here?**

> The Runge phenomenon is the overshoot you get when you fit a
> *single high-degree polynomial* to many data points. Splines
> avoid it by using piecewise low-degree polynomials, so it's not
> a concern here. What we *do* see in the sharp-zigzag results is
> *spline overshoot* near corners, which is a related but distinct
> phenomenon — it's the spline's smoothness fighting against the
> sharpness of the input.

## On implementation and scope

**Q: How long did this take?**

> About a week of focused work, spread over the semester. The
> implementation took two days; the experiment design and sweep
> took two more; writing up took the rest. The Jupyter notebook
> that regenerates every figure is in the repository.

**Q: Why only 2-DOF?**

> Pedagogical clarity. A 2-DOF arm has closed-form forward
> kinematics that fits on a slide; a 6-DOF arm needs a Denavit-
> Hartenberg table that distracts from the interpolation question.
> The substantive conclusions don't change in higher dimensions,
> just the visualisation gets harder.

**Q: Why didn't you use a real robot dataset?**

> I considered it but the synthetic data has two advantages: every
> reader can reproduce it, and I can vary the geometry and density
> independently. A real-robot follow-up is on the limitations
> slide for Deliverable 4.

**Q: Did you compare against deep-learning trajectory generators?**

> No. Learning-based trajectory generators are a different kind of
> thing — they require training data and produce policies, not
> closed-form interpolants. The textbook methods compared here are
> the right baselines for any future learning-based approach.

## On the surprise

**Q: Is the natural-quintic-jerk-spike result novel?**

> The mechanism is well-known in the trajectory-planning
> literature, but it's rarely emphasised in introductory treatments
> of splines. So it's not novel research, but it's a useful
> pedagogical point.

**Q: Why does the cubic spline not have the same problem?**

> The cubic's natural BC only constrains *acceleration* at the
> endpoints, not velocity. So the cubic can pick its boundary
> velocity to match the chord-length-implied speed, while the
> natural quintic is forced to start at rest *in addition to*
> reaching the nominal speed.

**Q: Could you fix the quintic with a different boundary condition?**

> Yes — anything other than "start at rest in velocity" would
> remove the spike. The cubic-derived clamped BC I used is one
> reasonable choice; another is "match the average speed of the
> first segment". The point is just that the natural BC is a
> *very* aggressive constraint and not always the right default.

---

# Part III — Quick reference

Numbers worth memorising for fluent answers:

* **Mobile robot baseline**: 12 waypoints, total time 24.93 s,
  nominal 1 m/s.
* **Robot arm baseline**: 8 waypoints, total time 7 s, $L_1=0.5$,
  $L_2=0.4$ m, reach 0.9 m.
* **Cubic spline mobile robot**: peak accel 1.18 m/s$^2$, RMS jerk
  0.34 m/s$^3$.
* **Natural-BC quintic mobile robot**: peak accel 1.49 m/s$^2$,
  RMS jerk 0.85 m/s$^3$.
* **Clamped quintic** (with cubic-derived BCs): peak accel 1.00,
  RMS jerk 0.31 — beats cubic.
* **B-spline mobile robot**: peak accel 0.41, RMS jerk 0.13,
  waypoint error 0.68 m.
* **Linear velocity discontinuity**: ~1 m/s at every waypoint.
* **Validation errors**: cubic $v$ error $2.6 \times 10^{-5}$,
  quintic $j$ error $9 \times 10^{-4}$, cubic $j$ error $0.51$
  (real, not a bug).
* **Total experiment runtime**: ~30 ms, $<$1 ms per method.
* **Density sweep** ($n=5 \to 12 \to 20$): cubic RMS jerk roughly
  $0.13 \to 0.34 \to 0.88$.
* **Geometry sweep**: sharp/gradual ratio for peak accel — cubic
  1.78, B-spline 1.18.

Key textbook references:

* Equation 18.2: linear interpolation.
* Equation 18.36: cubic spline closed form.
* Equation 18.37: cubic spline tridiagonal system.
* Chapter 28: numerical differentiation formulas.
* Section 24.1: chord-length parameterisation recommendation.
