# Builder guide

This repository now includes a loadable, project-owned mechanical package in
[`hardware/`](hardware/). It is intended to let another engineer inspect,
quote, print, and mechanically assemble the research design without receiving
supplier CAD that this project cannot redistribute.

> [!CAUTION]
> This is an alpha research build, not a production-ready or safety-qualified
> machine. The current release gates intentionally block powered winding.
> Review [`TODO.md`](TODO.md) and
> [`docs/engineering-status.md`](docs/engineering-status.md) before spending
> money or energizing hardware.

## Start here

1. Open
   [`hardware/assembly/stator_winder_reference_envelope.step`](hardware/assembly/stator_winder_reference_envelope.step)
   in FreeCAD, Fusion, Onshape, or another STEP-capable CAD system. Purchased
   components are represented by project-authored dimensional envelopes, not
   copied vendor geometry.
2. Review [`COSTS.md`](COSTS.md) and [`bom.csv`](bom.csv). The current
   `$1,573.78` subtotal is incomplete and must not be treated as a checkout
   total.
3. Print and measure the fit/bridge coupon under `hardware/coupon/` before the
   complete printed set.
4. Slice the 21 files in `hardware/printables/stl/` using the settings in
   `hardware/orders/print_jobs.csv` and the checked-in profiles under
   `print/profiles/`.
5. Use the editable versions in `hardware/printables/step/` when a printer or
   insert fit needs a controlled adjustment.
6. Send the carriage DXF, custom STEP files, and matching drawings to suitable
   fabricators for quotation. RFQ-ready does not mean order- or
   production-authorized.
7. Source purchased hardware from `hardware/orders/full_order.csv`; verify
   every current price, pack quantity, and substitution before checkout.

## Package map

```text
hardware/
  assembly/             vendor-free loadable reference assembly
  coupon/               proof coupon STEP, STL, and 3MF
  drawings/             project-authored shop/RFQ PDFs
  fabrication/          carriage DXF
  machining/legacy/     spacers, shaft, lead screw, eyelet, and holder STEP
  machining/successor/  PEEK, aluminum, and B777 successor STEP
  manifests/            checksums and external-CAD boundary
  orders/               full order, fastener, and print CSVs
  printables/step/       editable printed-part geometry
  printables/stl/        print-oriented meshes
```

## Source-only assembly generation

Python 3.11 and `requirements-dev.txt` are the verified environment. A clean
checkout can generate the public reference assembly without any supplier CAD:

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.venv\Scripts\python.exe cad\assembly.py `
  --reference-mode envelope `
  --output out\stator_winder_reference_envelope.step
```

`--reference-mode exact` is reserved for local fit and collision work after
the user has separately obtained the external models listed in
[`hardware/manifests/external-cad.json`](hardware/manifests/external-cad.json).
Exact supplier geometry is not required to open or mechanically understand
the public package.

The simpler custom machining set is also source-only:

```powershell
.venv\Scripts\python.exe cad\custom_parts.py
```

The full print and successor-manufacturing regeneration path consumes
hash-bound engineering reports from the internal validation pipeline. The
checked-in artifacts let builders inspect and quote the current geometry,
while their manifests retain the source and checksum boundary.

## Mechanical assembly order

1. Build the 2020 frame from the cut list drawing; square it before fitting
   the MGN12 rails.
2. Install the M0 screw, fixed-end bearing stack, motor, coupling, nut, and
   carriage. Confirm free travel by hand before fitting a belt or energizing a
   motor.
3. Assemble the M1 spindle tower, 608 bearing stack, selected work holder, and
   stator fixture.
4. Assemble the M2 flyer bearing stack, shaft, pulley, belt, arm, guide, and
   balance parts. Installed balance and retention tests are mandatory before
   rotation.
5. Install the spool, felt drag stack, dancer, eyelets, and wire path.
6. Verify all fastener stacks against `hardware/orders/hardware_order.csv`.
7. Stop at mechanical inspection. Controller wiring, mains enclosure,
   protective earth, emergency stop, commissioning, and powered motion remain
   separate safety work and are not authorized by this package.

## External CAD boundary

Supplier and community files were used locally to verify interfaces. They are
not necessary to load the public assembly, and they are not included when the
project does not have a clear redistribution grant. The manifest records the
expected filenames and SHA-256 values so a builder can obtain and verify their
own copies under the source's terms.
