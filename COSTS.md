# Cost status

The published [`bom.csv`](bom.csv) contains 46 physical line items. As of the
project's July 2026 price checks, 36 rows have numeric planning values and 10
remain unpriced.

## Numeric planning subtotal

| Category | Subtotal (USD) |
| --- | ---: |
| Motion | $1,124.76 |
| Required 36 V supply candidate | $199.00 |
| Structure | $97.00 |
| Assembly consumables | $87.79 |
| Tensioner | $26.24 |
| Optional controller integration | $25.00 |
| Printing material | $13.99 |
| **Current numeric subtotal** | **$1,573.78** |

This is a planning lower bound, not an expected delivered total. It includes
a conditional `$199` M2 driver candidate and a `$50` pulley-machining
allowance that is not a supplier quote.

## Unpriced required or job-dependent rows

- MIC6 carriage plate from the release DXF
- counterweight fastener packs
- one-piece machined PEEK flyer guide and bell
- front/rear machined PEEK stator caps
- front/rear machined PEEK active-sector guides
- machined 6061-T6 active-sector yoke
- delivered USD cost for the stock NBK D10 pulley
- six serialized ASTM-B777 tungsten balance trims
- winding wire
- Nomex slot insulation

The subtotal also excludes shipping, tax, import charges, refreshed pack
pricing, machine-shop minimums, inspection charges, mains enclosure and
wiring, protective devices, emergency-stop hardware, and the cost of closing
physical qualification gates.

The stricter machine-readable release catalog currently classifies only
`$420.28` as annotated order-ready known-current cost, `$199.00` as
conditional known-current cost, and `$50.00` as a planning allowance. That
catalog intentionally reports `complete_machine_total_available: false`.
Until the ten unpriced rows are quoted and the manual BOM is reconciled with
the release catalog, the project does not have a defensible complete expected
build cost.

Before purchasing, regenerate or review `hardware/orders/full_order.csv`,
refresh every vendor price, and record quotes separately from allowances.
