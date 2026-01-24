# MAPF System Architecture

## High-Level Design Philosophy

The MAPF hospital domain requires balancing two competing concerns:
1. **Completeness**: Solve as many levels as possible within constraints
2. **Efficiency**: Minimize actions and computation time

This document outlines architectural patterns and design decisions for building an effective MAPF solver.

## Architectural Overview

```
┌─────────────────────────────────────────────────────────┐
│                   Client Application                    │
├─────────────────────────────────────────────────────────┤
│  Level Parser → Search Controller → Action Executor     │
│       ↓              ↓                    ↓              │
│  State Model    Planning Engine     Server Interface    │
└─────────────────────────────────────────────────────────┘
```

### Core Components

1. **Level Parser**: Reads level files, constructs initial/goal states
2. **State Model**: Efficient representation of world state
3. **Planning Engine**: Search algorithms and heuristics
4. **Action Executor**: Translates plans into joint actions and handles communication

## Critical Architectural Decisions

### 1. State Representation

**Immutable State Objects**
- States should be immutable to enable safe hashing and caching
- Enables efficient closed list management in graph search
- Facilitates parallel exploration (if using multi-threading)

**Suggested Structure**:
```java
class State {
    final Map<Integer, Position> agentPositions;  // agent number → position
    final Map<Position, Character> boxPositions;   // position → box letter
    final int hashCode;  // Pre-computed for performance
    
    // Constructor computes hash once
    // No setters - all modifications create new State instances
}
```

**Performance Consideration**: Use primitive arrays for positions instead of objects where possible (e.g., `int[numAgents][2]` instead of `Map<Integer, Position>`).

### 2. Multi-Agent Coordination Strategy

**Option A: Independent Planning + Conflict Resolution**
- Each agent plans independently to its goal
- Conflicts detected and resolved through priority schemes or replanning
- **Pros**: Scalable to many agents, fast for loosely-coupled problems
- **Cons**: Can miss globally optimal solutions, may fail on tightly-coupled scenarios

**Option B: Coupled Planning**
- Plan for all agents simultaneously as a single composite search problem
- Search space: joint configuration space of all agents
- **Pros**: Guarantees conflict-free plans, better for tightly-coupled problems
- **Cons**: Exponential state space explosion (|positions|^|agents|)

**Recommendation**: Start with Option A (independent planning), upgrade to Option B or hybrid approach if competition levels require it.

### 3. Search Algorithm Framework

**Base Graph Search Structure**:
```
function graphSearch(initialState, goalCondition):
    openList ← priority queue with initialState
    closedList ← empty set
    
    while openList not empty:
        current ← openList.pop()
        
        if current in closedList:
            continue
            
        if goalCondition(current):
            return reconstructPath(current)
            
        closedList.add(current)
        
        for successor in expand(current):
            if successor not in closedList:
                openList.add(successor)
                
    return NO_SOLUTION
```

**Algorithm Choices**:
- **A***: Optimal, requires admissible heuristic, slower for large search spaces
- **Weighted A*** (WA*): Suboptimal but faster, weight parameter tunes optimality vs. speed
- **Greedy Best-First**: Very fast, no optimality guarantee, may fail to find solutions

**Recommendation**: Implement configurable search (can switch algorithms via parameter) to test different strategies on different level types.

### 4. Heuristic Function Design

**Single-Agent Heuristics**:
```
h(state) = max(h_agent(state), h_boxes(state))

where:
  h_agent(state) = Manhattan distance from agent to its goal (if it has one)
  h_boxes(state) = sum of Manhattan distances from each box to its goal
```

**Multi-Agent Heuristics** (more sophisticated):
```
h(state) = h_boxes(state) + h_conflicts(state)

where:
  h_conflicts(state) = estimated cost to resolve agent path conflicts
```

**Heuristic Properties**:
- **Admissibility**: h(state) ≤ true_cost(state) for A* optimality
- **Consistency**: h(current) ≤ cost(current, successor) + h(successor)
- **Informativeness**: Higher h-values prune more of search space

**Performance Optimization**:
- Precompute all-pairs shortest paths in obstacle-free grid
- Cache heuristic values for recurring states
- Use lazy evaluation: only compute expensive components when needed

### 5. Successor State Generation

**Action Applicability Checks** (critical for correctness):
```java
boolean isApplicable(State state, Action action) {
    // Check preconditions based on action type
    // - Move: target cell free?
    // - Push: box present and correct color? box target cell free?
    // - Pull: agent target free? box in opposite direction with correct color?
    
    // For joint actions: also check for conflicts
    return preconditionsSatisfied && noConflicts;
}
```

**Optimization**: Only generate applicable actions (don't generate then filter).

### 6. Memory Management

**Search Space Explosion Mitigation**:
- **Duplicate detection**: Use closed list with efficient hashing (HashMap or HashSet)
- **Memory limits**: Track memory usage, switch to iterative deepening if approaching limits
- **State pruning**: Implement domain-specific pruning rules (e.g., don't push boxes into corners)

**Java-Specific Considerations**:
- Use `HashMap<State, Node>` for closed list (fast lookup)
- Use `PriorityQueue<Node>` for open list (efficient min extraction)
- Consider weak references for large state spaces to allow garbage collection

## Design Patterns

### 1. Strategy Pattern for Search Algorithms
```java
interface SearchStrategy {
    List<Action> search(State initial, GoalCondition goal);
}

class AStarSearch implements SearchStrategy { ... }
class GreedySearch implements SearchStrategy { ... }

class SearchController {
    SearchStrategy strategy;
    
    void setStrategy(SearchStrategy strategy) {
        this.strategy = strategy;
    }
}
```

### 2. Factory Pattern for Heuristics
```java
interface Heuristic {
    double evaluate(State state, GoalState goal);
}

class ManhattanHeuristic implements Heuristic { ... }
class ConflictAwareHeuristic implements Heuristic { ... }

class HeuristicFactory {
    static Heuristic create(String type) { ... }
}
```

### 3. Command Pattern for Actions
```java
abstract class Action {
    abstract State apply(State current);
    abstract boolean isApplicable(State current);
    abstract String toProtocolString();  // e.g., "Move(N)" or "Push(E,S)"
}
```

## Performance Optimization Strategies

### Phase 1: Correctness
1. Implement basic A* with simple heuristic
2. Validate on warmup levels
3. Ensure all action types work correctly

### Phase 2: Efficiency
1. Profile code to identify bottlenecks (likely state hashing or successor generation)
2. Optimize state representation (use primitives, not objects)
3. Improve heuristic informativeness

### Phase 3: Scalability
1. Implement weighted A* for speed-accuracy tradeoff
2. Add conflict-based search for multi-agent scenarios
3. Consider anytime algorithms that return improving solutions

## Testing Strategy

### Unit Testing
- State representation: equality, hashing, immutability
- Action applicability: each action type with various preconditions
- Heuristic functions: admissibility, consistency

### Integration Testing
- Parse warmup levels correctly
- Generate valid action sequences
- Communicate with server correctly (protocol compliance)

### Performance Testing
- Measure states explored per second
- Track memory consumption growth
- Validate within 3-minute timeout on competition levels

## Common Pitfalls to Avoid

1. **Mutable State Objects**: Leads to subtle bugs in closed list (states can be modified after hashing)
2. **Inadmissible Heuristics**: A* finds suboptimal solutions or behaves like weighted A*
3. **Missing Conflict Detection**: Joint actions that violate simultaneous movement rules
4. **Memory Leaks**: Not clearing closed list between level attempts
5. **Inefficient Hashing**: Using default Object.hashCode() instead of custom state-based hash

## Advanced Techniques (Optional)

If basic approaches struggle with competition levels:

### Conflict-Based Search (CBS)
- Plan for each agent independently
- Detect conflicts in joint execution
- Add constraints to avoid conflicts, replan

### Operator Decomposition
- Meta-level planning: "agent A moves box B to region R"
- Low-level planning: concrete action sequences

### Learning-Based Heuristics
- Train neural network to predict solution cost from state features
- Use as heuristic in A* (may not be admissible)

## References

Key papers on MAPF (if implementing advanced techniques):
- Sharon et al. (2015): "Conflict-Based Search for Optimal Multi-Agent Path Finding"
- Silver (2005): "Cooperative Pathfinding" (for basic multi-agent coordination)
- Hart et al. (1968): "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (original A* paper)

## Recommended Development Workflow

1. **Week 1**: Implement state model, basic A* with Manhattan heuristic, test on single-agent warmup levels
2. **Week 2**: Add multi-agent joint action generation, conflict detection, test on simple multi-agent levels
3. **Week 3**: Optimize performance (profiling, state representation, heuristic tuning)
4. **Week 4**: Implement advanced techniques if needed, prepare competition submission
