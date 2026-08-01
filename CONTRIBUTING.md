# Contributing

Contributions are welcome, especially fixes that make an engineering claim
more reproducible, more conservative, or easier to audit.

## Ground rules

- Do not weaken or bypass a fail-closed release check to make a report pass.
- Keep geometry-driving dimensions in the parametric source, not only in a
  generated STEP or mesh.
- Add a focused regression test for behavioral or geometry changes.
- State what a result proves and what it does not prove.
- Do not commit generated `out/` artifacts or locally downloaded supplier CAD,
  datasheets, product images, or unlicensed community files.
- Record the source URL, part/revision, retrieval date, license status, and
  checksum for every external engineering reference.

## Development setup

```powershell
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

Start with the focused tests for the code you changed. Geometry tests may need
the separately obtained models documented in `cad/models/README.md`.

## Pull requests

A good pull request explains:

- the engineering problem and affected release gate;
- the source of any new requirement or dimension;
- the exact test, audit, or visual review performed;
- any remaining physical, tolerance, fatigue, safety, or procurement gap; and
- whether generated artifacts or supplier references must be refreshed.

By contributing, you agree that your project-authored contribution is
available under the repository's applicable license scope in
`LICENSES/README.md`.
