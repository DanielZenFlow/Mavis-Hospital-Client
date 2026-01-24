# MAPF Hospital Domain - Product Specification

## Problem Overview

The Multi-Agent Path Finding (MAPF) problem in the hospital domain involves coordinating multiple robots to transport items (boxes) to designated locations in a grid-based hospital environment. The system simulates transportation robots similar to real-world applications like Amazon's KIVA robots and hospital TUG robots.

## Domain Elements

### Agents (Robots)
- Represented by numbers: 0-9 (max 10 agents)
- Must be consecutively numbered starting from 0
- Each agent has a color attribute
- Visualized as numbered colored circles
- Can have optional goal positions (yellow circles with matching numbers)

### Boxes (Items)
- Represented by capital letters: A-Z
- Each letter denotes a box type (e.g., B for hospital beds)
- Multiple boxes can share the same letter/type
- Each box has a color attribute
- Visualized as lettered colored squares
- Must reach goal positions (yellow squares with matching letters)

### Environment
- Grid-based layout (max 215-1 rows × 215-1 columns)
- Walls represented by `+` symbols (black squares in visualization)
- Free cells for movement
- All agents and boxes must be enclosed by walls

### Colors
- Available colors: blue, red, cyan, purple, green, orange, pink, grey, lightblue, brown
- **Critical constraint**: Agents can only move boxes of the same color

## Actions

### 1. Move
- Syntax: `Move(N|S|E|W)`
- Agent moves one cell in specified direction
- Precondition: Target cell must be free

### 2. Push
- Syntax: `Push(<agent-dir>, <box-dir>)`
- Agent moves in `<agent-dir>`, box moves in `<box-dir>`
- Can push boxes "around corners" (e.g., `Push(W,S)`)
- Preconditions:
  - Adjacent cell in agent direction contains a box of same color
  - Box's target cell in box direction is free
- Note: Prevents agent-box position swaps

### 3. Pull
- Syntax: `Pull(<agent-dir>, <box-dir>)`
- Agent moves in `<agent-dir>`, box follows in `<box-dir>`
- Preconditions:
  - Agent's target cell in agent direction is free
  - Adjacent cell opposite to box direction contains a box of same color
- Note: Prevents agent-box position swaps

### 4. NoOp
- Syntax: `NoOp`
- Agent does nothing
- Always applicable
- Result of any failed action execution

## Multi-Agent Coordination

### Joint Actions
- Syntax: `<action0>|<action1>|...|<action9>`
- All agent actions are fully synchronized
- Cell occupancy determined at start of joint action (no sequential movement within same timestep)

### Conflicts
Conflicting actions cause both agents to execute NoOp instead:
- Two or more agents move into the same cell
- Two or more agents push/pull boxes into the same cell
- Two agents attempt to move the same box

## Goal State Definition

A level is solved when:
- Every goal cell has an object of the correct type occupying it
- Yellow squares must have boxes with matching letters
- Yellow circles must have agents with matching numbers
- Objects without goal cells can be anywhere

## Competition Constraints

### Hard Limits
- **Time limit**: 3 minutes per level
- **Action limit**: 20,000 joint actions per level
- **Level size**: Max 50×50 grid cells

### Scoring
- **Action score**: Number of joint actions used (fewer = better)
- **Time score**: Computation time (logarithmic to reduce hardware bias)
- Unsolved levels receive score of 0

### Success Priority
1. Solve maximum number of levels (completeness)
2. Minimize action count (efficiency)
3. Minimize computation time (speed)

## Level Format

Levels are defined in ASCII text files with this structure:
```
#domain
hospital
#levelname
<name>
#colors
<color>: <object>, <object>, ...
#initial
<grid with +, 0-9, A-Z, spaces>
#goal
<grid with +, 0-9, A-Z, spaces>
#end
```

Key format rules:
- Walls must match exactly in initial and goal states
- Line termination: LF or CRLF
- No tab characters
- Agents/boxes must be specified with colors in #colors section

## Client-Server Protocol

### Communication Flow
1. Client sends name
2. Server sends level file contents
3. Client sends joint action or comment (starting with #)
4. Server responds with success/failure for each agent: `true|false|...|true`
5. Repeat steps 3-4 until level solved, timeout, or action limit reached

### Success/Failure Indicators
- `true`: Action executed successfully
- `false`: Action failed (inapplicability or conflict)

## Real-World Context

The domain simulates actual hospital automation systems:
- TUG robots (Aethon): Deployed in 100+ US hospitals since 2004
- First European deployment: Sygehus Sønderjylland, Denmark (2012)
- Applications: Medicine delivery, meal transport, waste management
- Related systems: Amazon KIVA warehouse robots

## Key Design Considerations

1. **Color-based capabilities**: Models real-world specialization (e.g., only authorized robots access secure storage)
2. **Conflict resolution**: Simulates physical collision avoidance in real environments
3. **Simultaneous actions**: Reflects distributed multi-agent systems where perfect coordination is impossible
4. **Action limits**: Represents battery life and operational efficiency constraints
