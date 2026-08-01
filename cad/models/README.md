# External CAD cache

This directory is a local cache boundary, not a redistributable model library.

The project uses exact supplier and catalog models for interface checks where
their geometry materially affects fit or collision results. Most of those
files do not carry an open redistribution license. They are therefore ignored
by Git and must be obtained directly by each user under the source's current
terms.

The project-authored Python generators, JSON provenance manifests, and Markdown
verification reports remain versioned. Consult `upgrades/*.report.md`,
`upgrades/*.source.json`, `bom.csv`, and `NOTICE.md` for:

- expected local filename;
- supplier and exact part number;
- source URL;
- known license or terms boundary;
- authoritative interface dimensions; and
- SHA-256 checksum where one was recorded.

Do not substitute a similarly named model without rerunning the dimensional,
placement, collision, and load audits that depend on it.

The absence of a supplier file should fail the exact-model audit or select an
explicit parametric-envelope path. It must never silently create a claim that
the exact purchased component was verified.
