package mapf.client;

import mapf.domain.*;
import mapf.planning.*;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;

import java.io.*;
import java.util.*;

/**
 * Main client class that communicates with the MAvis server.
 * 
 * Communication protocol:
 * 1. Client sends its name to stdout
 * 2. Client reads level description from stdin
 * 3. Client computes actions and sends them to stdout
 * 4. Server responds with action results
 * 5. Repeat 3-4 until goal state is reached
 * 
 * Debug output should go to stderr to avoid interfering with server
 * communication.
 * 
 * Strategy Pattern: Delegates to PortfolioController for multi-strategy planning.
 */
public class Client {

    /** Client name sent to the server */
    private static final String CLIENT_NAME = "HospitalClient";

    /** Input reader for server communication */
    private final BufferedReader serverIn;

    /** Output writer for server communication */
    private final PrintStream serverOut;

    /** Debug output stream */
    private final PrintStream debugOut;

    /** The parsed level */
    private Level level;

    /** Current state */
    private State currentState;

    /** Search configuration */
    private final SearchConfig config;

    /**
     * Creates a new Client with standard I/O streams.
     */
    public Client() {
        this.serverIn = new BufferedReader(new InputStreamReader(System.in));
        this.serverOut = System.out;
        this.debugOut = System.err;
        this.config = SearchConfig.defaults();
    }

    /**
     * Creates a new Client with custom I/O streams (for testing).
     * 
     * @param in    input stream
     * @param out   output stream
     * @param debug debug output stream
     */
    public Client(BufferedReader in, PrintStream out, PrintStream debug) {
        this.serverIn = in;
        this.serverOut = out;
        this.debugOut = debug;
        this.config = SearchConfig.defaults();
    }

    /**
     * Creates a new Client with custom I/O streams and config (for testing).
     */
    public Client(BufferedReader in, PrintStream out, PrintStream debug, SearchConfig config) {
        this.serverIn = in;
        this.serverOut = out;
        this.debugOut = debug;
        this.config = config;
    }

    /**
     * Main entry point.
     * 
     * @param args command line arguments (not used)
     */
    public static void main(String[] args) {
        Client client = new Client();
        try {
            client.run();
        } catch (Exception e) {
            System.err.println("Client error: " + e.getMessage());
            e.printStackTrace(System.err);
            System.exit(1);
        }
    }

    /**
     * Runs the client's main loop.
     * 
     * @throws IOException if communication fails
     */
    public void run() throws IOException {
        // Step 1: Send client name
        sendClientName();

        // Step 2: Read and parse level
        parseLevel();

        debugOut.println("Level: " + level.getName() + " | " + level.getRows() + "x" + level.getCols() + " | " + level.getNumAgents() + " agents");

        // Step 3: Plan and execute
        planAndExecute();

        if (currentState.isGoalState(level)) {
            debugOut.println("Goal reached!");
        } else {
            debugOut.println("Partial plan executed - goal NOT reached. Check agent/box positions for debugging.");
            debugOut.println("Final state:");
            debugOut.println(currentState.toGridString(level));
        }
    }

    /**
     * Sends the client name to the server.
     */
    private void sendClientName() {
        serverOut.println(CLIENT_NAME);
    }

    /**
     * Reads and parses the level from the server.
     * 
     * @throws IOException if reading fails
     */
    private void parseLevel() throws IOException {
        LevelParser parser = new LevelParser();
        LevelParser.ParseResult result = parser.parse(serverIn);

        this.level = result.level;
        this.currentState = result.initialState;
    }

    /**
     * Main planning and execution loop.
     * Continues until the goal state is reached.
     * Uses strategy pattern with fallback mechanism.
     * 
     * @throws IOException if communication fails
     */
    private void planAndExecute() throws IOException {
        // Search with fallback mechanism
        List<Action[]> plan = searchWithFallback();

        if (plan == null || plan.isEmpty()) {
            debugOut.println("ERROR: No plan found with any strategy!");
            return;
        }

        // Check action limit
        if (plan.size() > SearchConfig.MAX_ACTIONS) {
            debugOut.println("WARNING: Plan exceeds action limit (" + plan.size() +
                    " > " + SearchConfig.MAX_ACTIONS + ")");
        }

        debugOut.println("Plan found with " + plan.size() + " steps");

        // Execute the plan step by step
        int step = 0;
        int totalActions = 0;

        for (Action[] actions : plan) {
            if (currentState.isGoalState(level)) {
                debugOut.println("Goal reached early at step " + step);
                break;
            }

            // Check total action limit
            totalActions++;
            if (totalActions > SearchConfig.MAX_ACTIONS) {
                debugOut.println("WARNING: Exceeded maximum actions limit");
                break;
            }

            // Send actions to server
            sendActions(actions);

            // Read response
            boolean[] results = readResponse();

            // Update state based on successful actions
            updateState(actions, results);

            step++;
        }
    }

    /**
     * Searches for a plan using PortfolioController (which handles multi-strategy fallback internally).
     */
    private List<Action[]> searchWithFallback() {
        debugOut.println("[Client] Using Portfolio Controller");
        PortfolioController portfolio = new PortfolioController(config);
        // Resolve effective wall-clock budget. Server -t is server-side and not
        // exposed via protocol, so allow override via env var MAVIS_TIMEOUT_MS
        // (tests/competition runners can pipe their server -t value through).
        // Reserve 10s safety buffer for partial-plan flush + serialization +
        // transmission so we never get killed mid-plan with bestPartialPlan
        // still cached in PortfolioController memory.
        long effectiveServerTimeoutMs = config.getTimeoutMs();
        String envTimeout = System.getenv("MAVIS_TIMEOUT_MS");
        if (envTimeout != null) {
            try {
                long v = Long.parseLong(envTimeout.trim());
                if (v > 0) effectiveServerTimeoutMs = v;
            } catch (NumberFormatException ignored) {}
        }
        long planningTimeout = Math.max(effectiveServerTimeoutMs - 10_000, effectiveServerTimeoutMs / 2);
        portfolio.setTimeout(planningTimeout);
        debugOut.println("[Client] Planning budget: " + planningTimeout + "ms (server=" + effectiveServerTimeoutMs + "ms)");
        return portfolio.search(currentState, level);
    }

    /**
     * Creates the heuristic to use for planning.
     * Override this method to change the heuristic.
     * 
     * @return the heuristic instance
     */
    protected Heuristic createHeuristic() {
        // Use TrueDistanceHeuristic for better performance
        // Fall back to Manhattan if true distance is too expensive
        try {
            return new TrueDistanceHeuristic(level);
        } catch (Exception e) {
            debugOut.println("Warning: Could not create TrueDistanceHeuristic, using Manhattan");
            return new ManhattanHeuristic();
        }
    }

    /**
     * Sends actions for all agents to the server.
     * 
     * @param actions array of actions, one per agent
     */
    private void sendActions(Action[] actions) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < actions.length; i++) {
            if (i > 0)
                sb.append("|");
            sb.append(actions[i].toServerString());
        }

        String actionString = sb.toString();
        serverOut.println(actionString);
    }

    /**
     * Reads the server's response to the last actions.
     * 
     * @return array of booleans indicating success/failure for each agent
     * @throws IOException if reading fails
     */
    private boolean[] readResponse() throws IOException {
        String response = serverIn.readLine();

        if (response == null) {
            throw new IOException("Server closed connection");
        }

        String[] parts = response.split("\\|");
        boolean[] results = new boolean[parts.length];

        for (int i = 0; i < parts.length; i++) {
            results[i] = parts[i].trim().equalsIgnoreCase("true");
        }

        return results;
    }

    /**
     * Updates the current state based on which actions succeeded.
     * 
     * @param actions the actions that were attempted
     * @param results which actions succeeded
     */
    private void updateState(Action[] actions, boolean[] results) {
        // Mirror server semantics: actions are evaluated synchronously against the
        // state at the START of the timestep. We mask rejected actions to NoOp and
        // hand the whole array to applyJointAction, which (a) reads every agent
        // position from the original state (no order-dependent shadow updates) and
        // (b) defensively rejects out-of-grid effects. Previously we called
        // currentState.apply(actions[i], i) per agent, which happened to produce
        // the same final state when results[] was trustworthy but bypassed those
        // safety checks and diverged in semantics from the planner's own
        // applyJointAction usage.
        boolean logRejected = "1".equals(System.getenv("MAVIS_LOG_REJECTED"));
        Action[] effective = new Action[actions.length];
        for (int i = 0; i < actions.length; i++) {
            boolean ok = i < results.length && results[i];
            if (ok) {
                effective[i] = actions[i];
            } else {
                effective[i] = Action.noOp();
                if (logRejected) {
                    debugOut.println("Warning: Action for agent " + i + " failed: " + actions[i]);
                }
            }
        }
        currentState = currentState.applyJointAction(effective, level);
    }
}
