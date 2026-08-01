# Third-party notices

The project licenses apply only to material owned by the project copyright
holder. They do not relicense external software, supplier geometry,
datasheets, photographs, trademarks, or patent rights.

## Separate controller dependency

The motion and serial compatibility contract comes from
[`aotenjo-xyz/winder`](https://github.com/aotenjo-xyz/winder), which is
MIT-licensed by its own contributors. Its source is not vendored here. Users
obtain it as a separate checkout and retain its license and notices.

## Locally cached COTS reference material

Development used local reference files from the following sources. The public
source snapshot deliberately excludes their CAD, PDFs, images, archives, and
converted previews:

| Source class | Examples | Publication treatment |
| --- | --- | --- |
| step.parts catalog | bearings, rails, screws, couplings, profiles | download separately; verify catalog checksum |
| Manufacturer CAD | StepperOnline, Leadshine, HIWIN, Omron, Elesa, Wurth | omitted unless an explicit redistributable license is established |
| Catalog CAD | McMaster-Carr and supplier configurators | omitted; design/reference use is not a redistribution grant |
| Community CAD | GrabCAD mirrors and unlicensed GitHub repositories | omitted; no license means no redistribution permission is assumed |
| CADENAS/NBK | P30 pulley STEP files marked CC BY-ND 4.0 | omitted from the public snapshot to avoid distributing modified or derived forms |
| Product media | datasheets, drawings, torque curves, product photos | omitted unless independently authored by this project |

The reports in `cad/models/upgrades/` preserve source URLs, part identifiers,
measurements, and checksums where available. Those facts and project-authored
verification notes remain in the repository; the external binary files do
not.

## Architecture research

Public product photos and documentation were reviewed to understand ordinary
flyer-winding architecture and failure modes. Patent publications
[US4340186A](https://patents.google.com/patent/US4340186A/en) and
[US20030150951A1](https://patents.google.com/patent/US20030150951A1/en) were
also read for background. No reference photograph, patent drawing, or
third-party machine CAD is included in the public source snapshot.

The mechanism's project source is independently parameterized from the written
requirements, controller motion contract, measured interfaces, and conventional
mechanical constraints. This statement is a provenance record, not a legal
opinion or a freedom-to-operate determination.

No affiliation with or endorsement by Aotenjo, CERN, any named supplier, or
any patent owner is claimed.
