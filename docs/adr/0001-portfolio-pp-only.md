# ADR 0001: Portfolio Uses Only PP Variants

## Status
Accepted (2026-05-09)

## Context
Hospital domain is Cooperative Multi-Agent Sokoban with Pull, not vanilla MAPF.
Vanilla CBS assumes point agents + static environment; both fail here
(see claudeopus47.txt §3.1.2). A full CBS port targeting push/pull is
estimated at PhD-thesis effort.

## Decision
PortfolioController offers strictly PP variants differing in subgoal
ORDERING (TOPOLOGICAL / DISTANCE_GREEDY / RANDOM with multi-seed).
CBS / JointAStar are explicitly excluded from the top-level sequence;
CBS is retained only as PP-internal cycle fallback.

## Consequences
- Pro: avoids brittle Sokoban-CBS port; cycle-breaking via multi-seed RANDOM.
- Con: levels with hard cyclic coupling depend on luck of seed.
- Future-proofing: if a real heterogeneous solver is added, expose it as
  a new StrategyConfig entry rather than rewriting the controller.

## References
- claudeopus47.txt §1.2.4, §3
- CLAUDE.md "Strategy Selection Flow"
- deepseek-bug.txt #7 (peer-reviewed as architectural decision, not bug)
