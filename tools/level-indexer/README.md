# Level Indexer

Small diagnostic helper for finding test-friendly Hospital levels without
manually reading large ASCII maps.

Run from anywhere inside the repository:

```powershell
python tools/level-indexer/level_indexer.py
```

or, from this directory:

```powershell
python level_indexer.py
```

Default output:

- `target/diagnostics/level-index/level-index.json`
- `target/diagnostics/level-index/level-index.md`

The indexer scans `levels/` and `complevels/`, extracts structural metrics, and
adds tags such as `tiny`, `single-agent`, `multi-agent`, `same-letter-multiple`,
`possible-namo`, `corridor-heavy`, and `deadends`. These tags are meant to help
pick small fixture levels and motif candidates for unit/regression tests.
