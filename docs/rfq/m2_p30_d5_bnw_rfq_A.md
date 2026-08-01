# RFQ M2-P30-D5-BNW-RFQ-A — configured motor pulley

Status: ready to send for supplier quotation and drawing only. This document does not authorize an order, installation, or machine motion.

## Requested NBK configuration

- Base part: `P30-3GT-BLP-6C-5`
- Teeth / pitch / belt width: 30 / 3 mm / 6 mm
- Stock bore: 5.0 mm
- Stock split-clamp bolt: supplied M2, 0.5 N m tightening torque
- Additional machining: `BNW`, two supplied radial set screws at 90 degrees
- Application shaft: Leadshine CS-M21708, 5.0 mm D shaft, 4.5 mm across the flat, 15 mm D-flat length
- Intended orientation: one set screw bears on the D-flat without crossing the split or clamp-bolt spot face

Official product/configurator source: <https://www.nbk1560.com/products/pulley/timingpulley/3GT-BLP-6C/P30-3GT-BLP-6C/>

The immutable stock reference STEP is `cad/models/upgrades/NBK_P30-3GT-BLP-6C-5_AP214.step`, SHA-256 `996449b7d9ec7703e7b38c6f75eff00a1174e3e1f088c05f0f1460b205169df9`. It is the unmodified stock pulley, not a representation of delivered BNW holes.

## Supplier return required

Please return all of the following before an order can be considered:

1. Complete configured order code and supplier drawing.
2. Quote, currency, quantity-one price, and lead time.
3. Bore tolerance and confirmation the configuration is permitted on this base part.
4. Set-screw quantity, thread size, length, material, tip style, tightening torque, and whether both screws are supplied.
5. Axial and azimuthal hole stations plus permitted screw insertion direction.
6. Confirmation that one screw may bear on the stated D-flat while both holes avoid the split and clamp-bolt spot face.
7. Any published bidirectional/reversing retention rating for this exact configuration on the stated shaft, or written confirmation that no such rating is supplied.

## Fail-closed acceptance boundary

The separate M3x12 hole and screw solids in the review CAD are inertia and clearance upper bounds only. They are not delivered-part claims and must not be copied into production CAD. A supplier-returned configuration still does not authorize motion: the received pulley must be modeled from the returned drawing and either carry a supplier-guaranteed bidirectional rating or pass the exact reversing slip coupon at or above `0.471456 N m`, followed by the existing hot-endurance gate.
