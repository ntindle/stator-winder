# Static frame hardware source-BREP audit

OpenCascade common-volume booleans on the authored build123d solids; STL/FCL meshes are not used.

## production 15-HBKT plus custom-shoe layout (bracket bodies)

- Brackets: 15
- Broad-phase candidate pairs: 5
- Positive-volume pairs: 0
- Allowed thread/T-slot engagements: 0
- Forbidden collisions: 0


## independently normalized 15-HBKT plus custom-shoe layout (HBKT bodies)

- Brackets: 15
- Broad-phase candidate pairs: 5
- Positive-volume pairs: 0
- Allowed thread/T-slot engagements: 0
- Forbidden collisions: 0


## Custom rear-post left shoe

- Shoe/cross common volume: 0.000000 mm3
- Shoe/post common volume: 0.000000 mm3
- Gap to left base rail: 1.000000 mm
- Nearest repositioned HBKT gap: 1.000000 mm
- Floor bore minimum ligament: 3.300 mm
- Upright bore minimum ligament: 5.300 mm
- Floor head minimum edge margin: 1.640 mm
- Upright head minimum edge margin: 3.640 mm
- Scallop back wall: 3.400 mm
- Scallop-to-upright-bore ligament: 2.500 mm
- Forbidden positive-volume pairs: 0

Allowed positive volumes are limited to the documented T-slot envelope:

- `rear_post_left_shoe_floor_m5x12` / `cross_rear`: 8.329693 mm3, `tslot_passage_envelope`
- `rear_post_left_shoe_floor_tnut` / `cross_rear`: 141.887304 mm3, `tslot_capture`
- `rear_post_left_shoe_upright_m5x12` / `rear_post`: 8.329693 mm3, `tslot_passage_envelope`
- `rear_post_left_shoe_upright_tnut` / `rear_post`: 40.200000 mm3, `tslot_capture`

## Exact disposition

- Replace `frame_bracket_rear_post_left` and its four stack occurrences with the custom 14 mm-floor shoe and two M5x12/HNTA5-5 stacks. A 25 mm HBKT leg cannot fit the 15 mm X corridor.
- The requested shoe fastener centers initially failed: floor screw / upright shoe = 62.827142 mm3 and the two screw envelopes = 12.713730 mm3. Add the OD9.2 scallop y=-219.2..-213.2 and move the upright axis-X stack from y=-211 to y=-208.
- Reposition `rear_base_L`, `mid_base_L/R`, and `front_base_L/R` to their cross front faces/upper-member undersides with `x=(1,0,0), y=(0,0,1), z=(0,-1,0)`.
- Reposition `front_stringer_L/R` the same way at origins `(-45,-225,170)` and `(45,-225,170)`.
- The proposed 16th front-Z rear-post HBKT is rejected: exact common volume with the rear post is 1810.303324 mm3.
- `post_L_front` / T8 has zero common volume. It is a tangent manufacturing-risk contact, not an allowed engagement; add a 2 mm corner relief if tolerance clearance is required.
