# Gears

An animated two-stage involute spur gear train, written in Curv. Each gear
is a custom signed distance field. Sliders in the viewer change the module,
the tooth counts, the backlash, the speed and the layout. A test suite checks
the geometry numerically through the `curv` command.

## Files

| File | Contents |
|------|----------|
| `gear.curv` | Library: gear geometry, distance fields, placement, meshing, the train builder. |
| `train.curv` | The parametric, animated demo. |
| `tests/support.curv` | Test helpers: checks, a deterministic number sequence, minimum search. |
| `tests/*.curv` | One test program per topic. Each evaluates to `#true` or stops with an error. |
| `run-tests` | Runs every test program and the GPU compile check. |

## Running

Show the train in the viewer:

    curv train.curv

Export a still image at time `t` seconds:

    curv -o train.png -O fstart=2 train.curv

Run the tests. Give the path of the `curv` binary if it is not on `PATH`:

    ./run-tests --curv ../curv/release/curv

## Gear geometry

All lengths are in model units. Angles are in radians. The inputs are the
module `m`, the tooth count `z`, the pressure angle `alpha` (default 20°)
and the circular backlash `j` (default 0).

| Name | Formula | Meaning |
|------|---------|---------|
| `inv a` | `tan a - a` | Involute function. |
| `rp` | `m*z/2` | Pitch radius. |
| `rb` | `rp*cos alpha` | Base radius. |
| `ra` | `rp + m` | Tip radius. |
| `rf` | `rp - 1.25*m` | Root radius. |
| `psi_p` | `(pi*m/2 - j/2) / (2*rp)` | Half angle of a tooth at the pitch circle. |
| `psi_b` | `psi_p + inv alpha` | Half angle of a tooth at the base circle. |
| `lip` | `sqrt(1 + (pi/z)^2)` | Lipschitz divisor of the field (see below). |

Each tooth is thinner than nominal by `j/2` at the pitch circle. Two gears
made with the same `j` mesh with a circular backlash of `j`.

Tooth 0 is centred on the gear's local +X axis. Tooth `k` is centred at
angle `k*tau/z`. Each flank is an involute of the base circle from `rb` to
`ra`. Below `rb`, each flank continues as a radial line at angle `psi_b`.
The tip is an arc of radius `ra`. The root is an arc of radius `rf`.

The profile has no undercut model. Use `z >= 18` with `alpha = 20°`.

## Distance field

The field of a gear is a function `field [r, psi]` of polar coordinates in
the gear's frame. It is negative inside the gear and positive outside.

1. Fold the angle into tooth 0:
   `u = abs(mod[psi + pi/z, tau/z] - pi/z)`.
2. Compute the flank term. Let `R = max[r, rb]` and `phi = acos(rb/R)`:
   `flank = (min[r, rb]*(u - psi_b) + rb*inv(phi)) / lip`.
3. Combine with the tip and root circles:
   `field = min[max[flank, r - ra], r - rf]`.

For `r >= rb`, the level sets of `rb*(u + inv phi)` are involutes of the
base circle. Its gradient has length 1, so it is the exact distance to the
flank's involute. For `r < rb`, the term `r*(u - psi_b)` has a gradient of length
`sqrt(1 + (u - psi_b)^2)`, which is at most `lip`. The division by `lip` makes
the whole field 1-Lipschitz.

The contract of the field:

- The zero set is the gear profile.
- `abs(field p - field q) <= mag(p - q)` for all points `p` and `q`.
- So `abs(field p)` never exceeds the true distance to the profile. Sphere
  tracing is safe without a `lipschitz` correction.
- Near a flank, the field equals the true distance divided by `lip`. For
  `z >= 18`, the field is within 1.6% of the true distance there.

The field has no conditionals that depend on sliders. The shape compiler
accepts it in parametric shapes.

## Library API (`gear.curv`)

`gear.curv` evaluates to a record with these fields.

`inv a`
: The involute function `tan a - a`.

`spur {m, z, alpha = 20*deg, backlash = 0}`
: A gear's geometry. The result has the fields `m`, `z`, `alpha`,
  `backlash`, `rp`, `rb`, `ra`, `rf`, `psi_p`, `psi_b`, `lip` and `field`.

`place {gear, centre, angle, z0, width, bore}`
: A placed gear. `centre` is a 2D point. `angle` is a function from time to
  the rotation of tooth 0 from the world +X axis. The gear occupies
  `z0 <= z <= z0 + width`. `bore` is the shaft hole diameter. The result has
  the input fields and the derived fields `hub`, `rim`, `hole_ring` and
  `hole_d` (see Body).

`mesh {driver, gear, direction, z0, width, bore}`
: A placed gear that meshes with the placed gear `driver`. Its centre is at
  distance `driver.gear.rp + gear.rp` from the driver's centre, in the
  direction `direction` (an angle). Its angle function is set by the phasing
  rule below.

`coaxial {base, gear, z0, width}`
: A placed gear on the same shaft as `base`. It has the same centre, angle
  function and bore as `base`.

`profile pg`
: The 2D shape of the placed gear's teeth, without bore or holes. It turns
  with time. Its bounding box is the square of half-size `ra` around the
  centre.

`body pg`
: The 3D shape of the placed gear: the extruded profile, minus the bore and
  the lightening holes. It turns with time. It uses the exact extrusion
  formula, so it keeps the 1-Lipschitz contract.

`train {m, teeth, backlash, speed, elbow, width}`
: The two-stage train (see Gear train). `teeth` is the list
  `[za, zb, zc, zd]`. The result has the fields `gears` (the list
  `[A, B, C, D]` of placed gears), `meshes` (the list `[[0,1], [2,3]]`),
  `ratio` and `shape`.

### Phasing rule

The driven gear's tooth must sit in the driver's tooth gap at the pitch
point. Let `gamma` be the direction from the driver's centre to the driven
gear's centre. Let `theta_1` be the driver's angle function, and let `z1` and
`z2` be the tooth counts of the driver and the driven gear. The driven
gear's angle is:

    theta_2 t = gamma + pi - pi/z2 + (gamma - theta_1 t) * z1/z2

At `theta_1 = gamma`, a driver tooth points at the driven gear, and a
driven gap points back at the driver. The angular velocity ratio is
`-z1/z2`.

With this phasing and a backlash `j`, both flanks of every tooth pair have
the same normal clearance. For involute teeth this clearance is constant
in time:

    s = j * cos(alpha) / 2

### Body

The body of a placed gear has a bore, a hub, a rim and six circular
lightening holes.

| Name | Formula |
|------|---------|
| `hub` | `bore/2 + 1.5*m` (outer radius of the solid hub) |
| `rim` | `rf - 2*m` (inner radius of the solid rim) |
| `hole_ring` | `(hub + rim)/2` (radius of the hole centres) |
| `hole_d` | `max[0, min[0.85*(rim - hub), 0.7*hole_ring]]` |

Hole `k` is centred at angle `k*tau/6` in the gear's frame. If `hole_d` is
0, the gear has no holes.

## Gear train

`train` builds a compound two-stage reduction:

- Gear A: centre `[0, 0]`, angle `t -> speed*t`, layer 1.
- Gear B: meshes with A in direction 0, layer 1.
- Gear C: coaxial with B, layer 2.
- Gear D: meshes with C in direction `elbow`, layer 2.

Layer 1 is `0 <= z <= width`. Layer 2 starts at `width + m`. Every gear has a
bore of `2*m`. Each shaft is a cylinder of diameter `0.9` times the bore.
It spans only the layers of the gears on it.

The output ratio is `ratio = (za*zc) / (zb*zd)`. This is the angular velocity
of D divided by that of A. The two sign changes cancel, so the ratio is
positive.

Each gear's colour brightens where the surface comes within `m/4` of the gear
it meshes with. This shows the contact moving along the line of action.

`train.curv` has these sliders:

| Slider | Range | Default |
|--------|-------|---------|
| `module` | 0.5 to 3 | 1.5 |
| `teeth_a`, `teeth_c` | 18 to 40 | 18, 20 |
| `teeth_b`, `teeth_d` | 18 to 60 | 36, 30 |
| `backlash` (in modules) | 0 to 0.2 | 0.08 |
| `speed` (radians per second) | -2 to 2 | 0.6 |
| `elbow` (degrees) | 40 to 140 | 75 |

The face width is `4*module`.

## Tests

`run-tests` runs each program in `tests/` except `support.curv`. A program
passes if `curv` exits with status 0 and prints `#true`. `run-tests` then
compiles `train.curv` for the GPU with `curv -o gpu`. It exits with status
0 only if every step passes.

`tests/instruments.curv`
: `minimize` and `clearance` from `support.curv` give the known answers for
  circles. A failure here means that the measurement is wrong, not the gears.

`tests/geometry.curv`
: The radii match the formulas. Points on the involute, computed from the
  textbook parametric form, have `abs(field) < 1e-9*m`. The tooth tip, the
  root and the pitch-circle tooth thickness are on the zero set. The `profile`
  shape gives the same values for every tooth and both flanks.

`tests/lipschitz.curv`
: For many point pairs, `abs(f p - f q) <= mag(p - q)*(1 + 1e-9)`. The pairs
  include points below the base circle, on the fold lines and inside the
  bore and holes. The test covers `field`, `profile` and `body`.

`tests/mesh.curv`
: For pairs of gears built with `mesh`, the minimum of `fA + fB` over the
  mesh zone is the clearance. It must lie between `s/lip_max - eps` and
  `s + eps`, where `eps = 2e-3*m` and `lip_max` is the larger `lip` of the
  pair. The test samples one tooth pitch of the driver's rotation. It
  covers tooth counts from 18 to 60, two modules, and backlash 0, `0.05*m`
  and `0.15*m`. A clearance below zero would mean the teeth overlap.

`tests/train.curv`
: The centre distances, the coaxial angles and the output ratio match the
  spec. The ratio is measured by finite differences of the angle functions.
  Both meshes of the train pass the clearance check of `tests/mesh.curv`.
  The two layers do not overlap in Z.

`tests/body.curv`
: The bore is empty. The hub and the rim are solid. The holes are empty
  when `hole_d > 0`. The body spans `z0` to `z0 + width`. The bounding box
  contains the body.

## Limits

- The tooth form has no root fillet and no undercut. It is valid for
  `z >= 18` at `alpha = 20°`.
- The shafts and the gears are separate solids. There are no bearings and no
  frame.
