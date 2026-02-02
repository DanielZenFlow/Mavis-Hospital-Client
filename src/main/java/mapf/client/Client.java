package mapf.client;

import mapf.domain.*;
import mapf.planning.*;
import mapf.planning.heuristic.Heuristic;
import mapf.planning.heuristic.ManhattanHeuristic;
import mapf.planning.heuristic.TrueDistanceHeuristic;
import mapf.planning.strategy.JointAStarStrategy;
import mapf.planning.strategy.SingleAgentStrategy;

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
 * Strategy Pattern: Dynamically selects search algorithm based on level
 * complexity.
 * Fallback Mechanism: If one strategy fails/times out, tries next with
 * increased weight.
 * 
 * Environment Variables:
 * - USE_SIMPLE_STRATEGY=true : Use the new simplified priority strategy (for
 * testing refactored code)
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

    /** Whether to use Portfolio Controller for strategy selection */
    public static boolean USE_PORTFOLIO = true;

    static {
        // Check environment variable to enable new simplified strategy
        String useSimple = System.getenv("USE_SIMPLE_STRATEGY");
        if ("true".equalsIgnoreCase(useSimple)) {
            StrategySelector.USE_SIMPLE_STRATEGY = true;
            System.err.println("[Client] USE_SIMPLE_STRATEGY enabled via environment variable");
        }

        // Check environment variable to enable Portfolio Controller
        String usePortfolio = System.getenv("USE_PORTFOLIO");
        if ("true".equalsIgnoreCase(usePortfolio)) {
            USE_PORTFOLIO = true;
            System.err.println("[Client] USE_PORTFOLIO enabled via environment variable");
        }
    }

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

        debugOut.println("Level: " + level.getName());
        debugOut.println("Dimensions: " + level.getRows() + "x" + level.getCols());
        debugOut.println("Agents: " + level.getNumAgents());
        debugOut.println("Initial state:");
        debugOut.println(currentState.toGridString(level));

        // Step 3: Plan and execute
        planAndExecute();

        debugOut.println("Goal reached!");
    }

    /**
     * Sends the client name to the server.
     */
    private void sendClientName() {
        serverOut.println(CLIENT_NAME);
        debugOut.println("Sent client name: " + CLIENT_NAME);
    }

    /**
     * Reads and parses the level from the server.
     * 
     * @throws IOException if reading fails
     */
    private void parseLevel() throws IOException {
        debugOut.println("Parsing level...");

        LevelParser parser = new LevelParser();
        LevelParser.ParseResult result = parser.parse(serverIn);

        this.level = result.level;
        this.currentState = result.initialState;

        debugOut.println("Level parsed successfully");
    }

    /**
     * Main planning and execution loop.
     * Continues until the goal state is reached.
     * Uses strategy pattern with fallback mechanism.
     * 
     * @throws IOException if communication fails
     */
    private void planAndExecute() throws IOException {
        int numAgents = currentState.getNumAgents();
        debugOut.println("Number of agents: " + numAgents);

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
     * Attempts search with fallback strategies if primary fails.
     * 
     * Fallback order:
     * 1. Primary strategy (A* with weight=1.0)
     * 2. Weighted A* (weight=2.0)
     * 3. More aggressive weighted A* (weight=5.0)
     * 4. Greedy search (infinite weight)
     * 
     * @return the plan, or null if all strategies fail
     */
    private List<Action[]> searchWithFallback() {
        // Use Portfolio Controller if enabled
        if (USE_PORTFOLIO) {
            debugOut.println("[Client] Using Portfolio Controller");
            PortfolioController portfolio = new PortfolioController(config);
            portfolio.setTimeout(config.getTimeoutMs());
            return portfolio.search(currentState, level);
        }

        // Legacy fallback mechanism
        int numAgents = currentState.getNumAgents();
        double[] weights = { config.getAstarWeight(), 2.0, 5.0, Double.POSITIVE_INFINITY };

        long totalStartTime = System.currentTimeMillis();
        long remainingTime = config.getTimeoutMs();

        for (int i = 0; i < weights.length; i++) {
            double weight = weights[i];
            if (remainingTime <= 0) {
                debugOut.println("Total timeout exceeded, stopping search");
                break;
            }

            // Allocate time for this attempt: half of remaining time
            // This ensures fair distribution across fallback strategies
            long attemptTimeout = Math.min(remainingTime / 2, remainingTime);

            SearchConfig currentConfig = new SearchConfig(
                    attemptTimeout,
                    config.getMaxStates(),
                    weight);

            SearchStrategy strategy = createStrategy(numAgents, currentConfig);
            debugOut.println("Trying strategy: " + strategy.getName() + " (weight=" + weight +
                    ", timeout=" + attemptTimeout + "ms)");

            List<Action[]> plan = strategy.search(currentState, level);

            if (plan != null && !plan.isEmpty()) {
                debugOut.println("Success with " + strategy.getName());
                return plan;
            }

            debugOut.println("Strategy " + strategy.getName() + " failed, trying fallback...");

            // Update remaining time
            remainingTime = config.getTimeoutMs() - (System.currentTimeMillis() - totalStartTime);
        }

        return null;
    }

    /**
     * Creates appropriate search strategy based on agent count and config.
     * Uses StrategySelector which can be configured via USE_SIMPLE_STRATEGY.
     */
    private SearchStrategy createStrategy(int numAgents, SearchConfig strategyConfig) {
        Heuristic heuristic = createHeuristic();

        // Use StrategySelector to allow simple strategy testing
        StrategySelector selector = new StrategySelector(heuristic, strategyConfig);
        SearchStrategy strategy = selector.selectStrategy(level, currentState);

        // Set weight if supported
        if (strategy instanceof SingleAgentStrategy) {
            ((SingleAgentStrategy) strategy).setWeight(strategyConfig.getAstarWeight());
        } else if (strategy instanceof JointAStarStrategy) {
            ((JointAStarStrategy) strategy).setWeight(strategyConfig.getAstarWeight());
        }

        return strategy;
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
        debugOut.println("Sent: " + actionString);
    }

    /**
     * Reads the server's response to the last actions.
     * 
     * @return array of booleans indicating success/failure for each agent
     * @throws IOException if reading fails
     */
    private boolean[] readResponse() throws IOException {
        String response = serverIn.readLine();
        debugOut.println("Received: " + response);

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
        // Apply successful actions one by one
        for (int i = 0; i < actions.length && i < results.length; i++) {
            if (results[i]) {
                currentState = currentState.apply(actions[i], i);
            } else {
                debugOut.println("Warning: Action for agent " + i + " failed: " + actions[i]);
            }
        }
    }
}
