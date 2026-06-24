# Presentation Script

**Target length:** ~7 minutes at conversational pace (~150 words/min,
~1050 spoken words). Trim the slides marked **\[optional]** for a
~5-minute version. Keep them all and slow your pace for ~10 minutes.

Times shown after each slide are spoken-pace estimates. Section
dividers are visual breaks; treat them as 1-second pauses.

---

## Slide 1 — Title (10 s)

> Hi everyone. I'm Param Chokshi, and this is my CISC 601 project on
> comparing four numerical interpolation methods for robot trajectory
> planning. I'll show you which one to pick, and one surprise that
> flipped my own intuition halfway through.

## Slide 2 — Agenda (10 s) **\[optional]**

> Five things in seven minutes: motivation, the four methods, the
> experiment, the surprise, and what to actually use.

## Slide 3 — Motivation divider (1 s)

## Slide 4 — Robots move through waypoints (25 s)

> A path planner gives a robot a sparse list of waypoints — corners
> of an aisle, say. The trajectory generator has to fill in the
> motion in between. The way it interpolates directly affects motor
> wear, payload stability, and whether the trajectory is even
> physically possible.

## Slide 5 — The smoothness budget (20 s)

> Three rules. Discontinuous velocity needs infinite acceleration —
> impossible. Discontinuous acceleration causes torque jumps and
> vibration. Discontinuous jerk excites mechanical resonance. Each
> method buys different parts of this budget at different costs.

## Slide 6 — Methods divider (1 s)

## Slide 7 — Four methods compared (30 s)

> The four methods, straight from chapters 18 and 24. Linear: C-zero
> continuity, position only. Cubic spline: C-two — continuous
> velocity and acceleration. Quintic spline: C-four — even jerk and
> snap continuous. Cubic B-spline: C-two everywhere, but doesn't
> pass through the interior waypoints. I implemented all four from
> the textbook equations, validated to machine precision against
> SciPy.

## Slide 8 — Methodology summary (15 s) **\[optional]**

> Two robotics datasets, three sweep axes — density, geometry,
> spacing — gives seven variants and twenty-eight experiments. Six
> metrics each. Plus a separate validation against centered finite
> differences from chapter 28.

## Slide 9 — Datasets divider (1 s)

## Slide 10 — Two scenarios (25 s)

> Two scenarios. On the left, a 12-waypoint mobile robot path
> through a warehouse aisle, with two sharp 90-degree turns and an
> S-curve. On the right, an 8-waypoint pick-and-place for a 2-DOF
> planar arm. The arm is interesting because interpolation happens
> in joint space but motion happens in Cartesian space — and that
> mapping is non-linear.

## Slide 11 — Baseline divider (1 s)

## Slide 12 — Mobile robot trajectory (20 s)

> All four methods on the same waypoints. Gray is linear — the
> polygon. Cubic and quintic both pass through every waypoint and
> round the corners. Red is the B-spline; you can see it visibly
> cuts the corners — that's the approximation property.

## Slide 13 — Mobile robot kinematics (35 s)

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

## Slide 14 — Robot arm Cartesian (15 s)

> Same eight joint waypoints, very different Cartesian paths.
> Forward kinematics is non-linear, so small differences in joint
> space get amplified.

## Slide 15 — Per-joint derivatives (10 s) **\[optional]**

> Drilling into one joint at a time makes the cubic's piecewise-
> constant jerk and the quintic's continuous-but-spiky jerk
> obvious side-by-side.

## Slide 16 — Quantitative summary (30 s)

> The numbers. Three things to flag. One: linear's zero
> acceleration is a mathematical artifact, not the physics. Two:
> the B-spline gets the smoothest derivatives — but it's 0.68
> metres off from the requested waypoints, which is a real
> failure if you actually need to reach those positions. Three —
> and this is the puzzle — the quintic spline has *higher* peak
> acceleration and RMS jerk than the cubic. Quintic has more
> continuity. So how is it less smooth? Hold that thought.

## Slide 17 — Variant divider (1 s)

## Slide 18 — Three axes of variation (15 s) **\[optional]**

> To draw general conclusions I varied three axes one at a time:
> waypoint count, geometric character, and time-spacing rule.

## Slide 19 — Density sweep (30 s)

> First surprise. Going from 5 to 12 to 20 waypoints on the same
> geometry, peak acceleration and RMS jerk *increase* monotonically
> for every spline. The intuition that more waypoints means
> smoother motion is just wrong here. Chord-length spacing keeps
> nominal speed fixed, so denser waypoints means smaller segments
> and bigger curvatures. Density buys geometric fidelity at the
> cost of smoothness.

## Slide 20 — Density visuals (10 s) **\[optional]**

> Visually you can see it: at n=5 every method makes obvious
> geometric mistakes; at n=20 they all converge to the polyline.

## Slide 21 — Density numbers (5 s) **\[optional]**

> The numerical version of the same trade-off.

## Slide 22 — Geometry sweep (25 s)

> Geometry has a much bigger effect than density. Sharp 90-degree
> corners roughly double peak acceleration and triple RMS jerk for
> the cubic and quintic. The B-spline absorbs sharp geometry
> gracefully — its sharp-versus-gradual ratio is 1.18, compared to
> 1.78 for the cubic. So if your input has sharp features,
> B-splines buy you the most smoothness.

## Slide 23 — Geometry visuals (10 s) **\[optional]**

> Visually: cubic and quintic overshoot at every sharp corner;
> B-spline cuts the corner.

## Slide 24 — Spacing sweep (10 s) **\[optional]**

> Time-spacing has the smallest effect. Chord-length versus uniform
> stays within about 30 percent on every metric.

## Slide 25 — All variants (10 s) **\[optional]**

> Pulling it together — the smoothest method depends strongly on
> the variant. There's no universal winner.

## Slide 26 — Validation divider (1 s) **\[optional]**

## Slide 27 — Analytic vs finite-difference (20 s)

> Quick validation note. To check the analytic derivative formulas
> are right, I also computed derivatives by centered finite
> difference — the schemes from chapter 28. Velocity and
> acceleration overlap perfectly. The cubic-spline jerk gap is
> real, not a bug — it's the finite-difference scheme failing to
> resolve the actual step discontinuities.

## Slide 28 — Why quintic looked worse (40 s)

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

## Slide 29 — Quintic boundary-condition fix (25 s)

> Here's the comparison. Green is the original natural-boundary
> quintic. Purple is the same quintic with cubic-derived clamped
> boundary conditions. Peak acceleration drops 33 percent. RMS
> jerk drops 64 percent. And now the quintic *beats* the cubic on
> both metrics — which is what you'd expect from a higher-
> continuity method. The original result was a boundary-condition
> artefact, not a property of quintic splines.

## Slide 30 — Recommendations divider (1 s)

## Slide 31 — Decision rule (25 s)

> So here's the practical guidance. Strict waypoints with known end
> velocities — clamped quintic spline. Strict waypoints, start and
> stop at rest — natural cubic spline; *avoid* the natural-boundary
> quintic. Soft waypoints with intentionally sharp features —
> cubic B-spline. Linear is for debugging only.

## Slide 32 — Eight findings (20 s) **\[optional]**

> The eight findings in one minute, if you want them: waypoint
> adherence is binary; linear has hidden C-one violations; more
> waypoints can hurt smoothness; geometry dominates; spacing barely
> matters; the quintic puzzle was a boundary-condition artefact;
> derivatives are validated against finite differences; and compute
> cost is sub-millisecond.

## Slide 33 — Limitations / future work (10 s) **\[optional]**

> Honest limitations: synthetic data, kinematic metrics only, two
> BC variants explored. Natural extensions for the final paper.

## Slide 34 — Thank you / Questions (10 s)

> That's the talk. Full code, data, and notebook are in the
> repository. Happy to take questions.

---

## Speaker notes

- **Pace**: aim for 140–150 words per minute. The surprise on slides
  28–29 is the punchline — practice the rhythm there.
- **5-minute version**: drop every **\[optional]** slide and shorten
  the kinematics commentary on slide 13 to two sentences. Brings the
  spoken script to ~750 words.
- **10-minute version**: keep everything, slow your pace, and add
  one audience-question prompt — slide 19 ("any guesses why
  smoothness *worsens*?") works well.
- **Slow down on**: "boundary condition", "chord-length
  parameterisation", "piecewise constant", "Cox–de Boor".
- **If you forget your line**: every slide title is a complete
  thought on its own. Read it aloud and improvise — you have the
  numbers in your head from running the experiment.
