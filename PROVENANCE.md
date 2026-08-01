# Design provenance

This document records what the project was based on and what was independently
authored. It exists so that "open source" does not blur together software
compatibility, functional inspiration, original design work, and third-party
reference assets.

## Original project work

The repository's Python CAD, mechanism layout, simulation, validation logic,
manufacturing studies, and documentation were authored for this project from a
written design envelope. Geometry is driven from explicit parameters and
machine interfaces rather than copied mesh or STEP geometry from another
winder.

The original brief permitted publicly visible photos and documentation for
architecture understanding while requiring the mechanics themselves to be
independently designed. That is the provenance standard applied here.

## Software compatibility reference

[`aotenjo-xyz/winder`](https://github.com/aotenjo-xyz/winder) is the motion
contract. The project studied its settings, `M{id}A{rad}` serial commands,
winding sequence, parking states, and simulation behavior. The upstream
checkout remains separate. Project code imports it at runtime only when the
user points to a local checkout.

## Photos, documents, and published mechanisms

Public photos, product pages, drawings, datasheets, and patent publications
were used to understand the function of flyer winders and purchased
components. Copyright protects those particular images and documents, so they
are not redistributed in the public repository. The project does not claim
ownership of them.

Ordinary functional ideas—such as a rotating flyer, hollow wire-feed shaft,
counterweight, carriage traverse, indexing spindle, and dancer tensioner—were
treated as engineering constraints. Their project-specific dimensions,
arrangement, source code, and generated geometry were independently authored.

## COTS geometry

Supplier and catalog STEP files were cached locally for interface verification
and collision checking. They are not project-owned source. Public Git history
therefore excludes them, including converted GLB previews and assemblies that
embed exact supplier geometry.

Where practical, the repository retains:

- a project-authored parametric envelope or generator;
- the supplier and part number;
- source URL and retrieval notes;
- a checksum for the locally obtained artifact; and
- a report comparing the model with published interface dimensions.

Users who need an exact supplier model must obtain it directly under the
supplier's current terms.

## Patent boundary

An open-source license grants only rights controlled by its licensors. It does
not promise that making or selling a physical machine is free of third-party
patent, regulatory, certification, or safety obligations. No
freedom-to-operate review has been performed.
