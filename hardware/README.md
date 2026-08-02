# Hardware build package

Everything under this directory is project-authored output selected for the
public alpha build package. It is covered by the repository's mechanical
design license scope in [`../LICENSES/README.md`](../LICENSES/README.md).

The package deliberately contains no supplier or community CAD. Purchased
components in the reference assembly are project-authored dimensional
envelopes. See [`manifests/external-cad.json`](manifests/external-cad.json) for
the optional exact-model cache boundary and [`../BUILD.md`](../BUILD.md) for
the build workflow.

`manifests/build-package.json` records a SHA-256 checksum, size, category, and
source path for every generated artifact in this package.
