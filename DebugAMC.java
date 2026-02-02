import mapf.domain.*;
import mapf.planning.analysis.TaskFilter;
import java.util.*;
import java.io.*;

public class DebugAMC {
    public static void main(String[] args) throws Exception {
        // Parse AMC.lvl
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
        Level level = Level.parse(reader);
        State state = State.parse(reader, level);
        
        // Find Agent 0's position
        Position agent0Pos = state.getAgentPosition(0);
        System.out.println("Agent 0 (cyan) position: " + agent0Pos);
        
        // Find B boxes
        System.out.println("\nB boxes:");
        for (Map.Entry<Position, Character> entry : state.getBoxes().entrySet()) {
            if (entry.getValue() == 'B') {
                System.out.println("  " + entry.getKey());
            }
        }
    }
}
