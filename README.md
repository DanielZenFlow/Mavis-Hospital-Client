# MAvis Hospital Client - Multi-Agent Path Finding

A Multi-Agent Path Finding (MAPF) client for the Hospital Domain, developed as part of a DTU course project.

## Overview

This program acts as a client that communicates with the MAvis server to control multiple agents in a Hospital Domain environment. The goal is to coordinate agents to push boxes to their designated target positions.

## Project Structure

```
mavis-hospital-client/
├── src/main/java/mapf/
│   ├── domain/           # Domain models
│   │   ├── Level.java    # Static level information
│   │   ├── State.java    # Dynamic state (agent/box positions)
│   │   ├── Action.java   # Agent actions (Move, Push, Pull, NoOp)
│   │   └── Position.java # Immutable coordinate class
│   ├── planning/         # Planning algorithms
│   │   ├── AStar.java              # A* search implementation
│   │   ├── Heuristic.java          # Heuristic interface
│   │   ├── ManhattanHeuristic.java # Manhattan distance heuristic
│   │   ├── TrueDistanceHeuristic.java # BFS-based true distance
│   │   ├── ConflictDetector.java   # Multi-agent conflict detection
│   │   └── MultiAgentPlanner.java  # Priority-based coordination
│   └── client/           # Server communication
│       ├── Client.java   # Main entry point
│       └── LevelParser.java # Level file parser
├── src/test/java/mapf/   # Unit tests (to be added)
├── levels/               # Level files (.lvl)
├── server.jar            # MAvis server (place here)
├── pom.xml
└── README.md
```

## Requirements

- Java 17 or higher
- Maven 3.6 or higher

## Building

```bash
# Compile the project
mvn compile

# Package into JAR
mvn package

# Run tests
mvn test
```

## Running

The client is designed to be run by the MAvis server:

```bash
java -jar target/mavis-hospital-client-1.0.jar < level_input > action_output
```

Or with the MAvis server (place server.jar in project root):

```bash
java -jar server.jar -c "java -jar target/mavis-hospital-client-1.0.jar" -l levels/example.lvl
```

## Communication Protocol

1. Client sends its name to stdout (e.g., "HospitalClient\n")
2. Client reads level description from stdin
3. Client computes actions and sends them to stdout
4. Server responds with action results
5. Repeat until goal state is reached

### Action Format

- Single agent: `Move(N)` or `Push(E,S)` or `Pull(W,E)` or `NoOp`
- Multiple agents: `action0|action1|action2|...`

### Server Response

- `true|true|false|...` - indicates success/failure for each agent's action

## Level File Format

```
#domain
hospital
#levelname
ExampleLevel
#colors
red: 0, A, B
blue: 1, C
#initial
+++++
+0A +
+ B1+
+++++
#goal
+++++
+  A+
+ B +
+++++
#end
```

## Authors

DTU Course Project Team

## License

Academic use only.
