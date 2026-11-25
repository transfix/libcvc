# Boat hulls baked from models

`bake_loft.py` turns a boat model (a `.glb`) into the stations of a lofted hull: the table a preset passes to `cvc::physics::loft_hull` (`inc/cvc/physics/hull.h`). The physics then floats, trims and drags the boat as its own hull would. A box would sit too high and steer like a raft.

The bake is fixed in code. The physics is the same whether or not the model's package is installed, and the model is only for drawing.

## How it works

1. The script fits the model as cvcGL's `fit_vehicle_geometry` does (`inc/cvc/gl/ariadne/vehicle_view.h`):
   - up is the axis the file stands on, or else its shortest extent;
   - its length runs along x with its nose at +x, scaled to `--length`;
   - it is centred along and across, with its lowest point at z = 0.
2. It cuts the model at `--stations` stations from `--from` to `--to` (x in that frame).
3. At each station it keeps the half-section from the keel up to the sheer: the widest point at or under `--sheer-max`.
   - It samples `--points` heights, closer together low on the hull, where the water is.
   - A half width never narrows going up, so inner faces (a cockpit's walls) are left out.
4. It moves the heights so the hull's box is centred on z = 0, the hull body's origin.

The boat node draws the model fitted to `dyn_boat_desc::length_overall` (the same `--length`) over that origin, so the drawing and the physics line up without an offset.

Whatever is outside the station range, or above the sheer, is not part of the hull: the outboards and brackets past the transom, a cabin, rails, a gun mount. Put the preset's propeller and rudder where the model's outboards or shaft are, in the same frame (the summary gives the shift in z).

## Re-baking the tactical boat

The vehicle-tactical-boat package's `TacticalBoat.glb` (cy-pca/graphics-assets) was baked with:

```bash
python3 -I tools/boat_hulls/bake_loft.py TacticalBoat.glb --length 7.8 --from -3.05 --to 3.8 --stations 16 --points 8 --mass 2800 --name kTacticalLoft
```

To re-bake, replace `kTacticalLoft` in `src/cvc/physics/dyn_boat_plant.cpp` with the output. Then measure the preset again (`physics_dyn_boat_plant_test`, `PresetsDoWhatTheySay` prints its top speed and turning radius) and update `max_speed`, `turn_radius` and the numbers in `TheTacticalBoatIsItsModelsHull`.

## A new boat

1. Find the hull's range. Print sections first (any station range will do) and read where the transom and stem are. Past the transom, the sections narrow to the outboards' legs.
2. Bake it, giving `--mass` so the summary reports the draft.
3. Make a preset like `dyn_boat_desc::tactical()`:
   - build `loft_hull(stations)`;
   - use `convex_hull{vertices}` as the collision shape;
   - set the mass, propeller, rudder and `length_overall`.
4. Give it grip sideways. The hull's pressure drag pushes back on sideways flow by its normal component squared, so a V bottom gets almost none from it, and an outboard boat on its loft alone skids and, at full lock, spins.
   - Set the hull's `lift_coefficient` (`hull.h`): 0.56, as `tactical()` has, is a planing plate's (Savitsky's dynamic lift at a wetted length of two beams). It gives the V and chines their grip and has the boat rise and trim onto the plane at speed.
   - Give the outboards' legs (or a shaft boat's rudder and skeg) the rudder slot, at the propellers. A hull's side force acts ahead of its centre of mass (its bow's pressure drag), so the hull alone yaws off its course; the legs far aft hold it straight. `tactical()`'s are 0.2 m² swinging 0.3 rad with the motors (`max_angle` equal to `outboard_max_angle`).
   - Measure the turns. Larger legs or a wider swing turn tighter, with more drift; `physics_dyn_boat_plant_test`'s `TheTacticalBoatDoesNotSpinAtFullLock` is the check that it holds a circle.
5. Measure it as above.

For a boat with no model, or many boats of one kind, a parametric hull (a generated V of given length, beam, deadrise and chine) would do as well. None is written yet.
