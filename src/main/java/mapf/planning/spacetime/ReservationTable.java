package mapf.planning.spacetime;

import mapf.domain.Position;
import java.util.*;

/**
 * Space-Time Reservation Table for MAPF.
 * Records which positions are reserved at which time steps.
 */
public class ReservationTable {
    
    // (position, time) -> agentId who reserved it
    private final Map<TimePosition, Integer> reservations = new HashMap<>();
    
    // For permanent reservations (agent stays at goal forever)
    private final Map<Position, Integer> permanentReservations = new HashMap<>();
    
    /** Reserve a position at a specific time for an agent. */
    public void reserve(Position pos, int time, int agentId) {
        reservations.put(new TimePosition(pos, time), agentId);
    }
    
    /** Reserve an entire path starting from a given time. */
    public void reservePath(int agentId, List<Position> path, int startTime) {
        int t = startTime;
        for (Position pos : path) {
            reserve(pos, t, agentId);
            t++;
        }
        // Permanently reserve final position (agent stays at goal)
        if (!path.isEmpty()) {
            Position finalPos = path.get(path.size() - 1);
            permanentReservations.put(finalPos, agentId);
        }
    }
    
    /** Check if a position is reserved at a specific time (excluding a given agent). */
    public boolean isReserved(Position pos, int time, int excludeAgentId) {
        // Check permanent reservations first
        Integer permAgent = permanentReservations.get(pos);
        if (permAgent != null && permAgent != excludeAgentId) {
            return true;
        }
        
        // Check time-specific reservation
        TimePosition tp = new TimePosition(pos, time);
        Integer reservedBy = reservations.get(tp);
        return reservedBy != null && reservedBy != excludeAgentId;
    }
    
    /** Clear all reservations. */
    public void clear() {
        reservations.clear();
        permanentReservations.clear();
    }
    
    /** Get the number of reservations (for debugging). */
    public int size() {
        return reservations.size();
    }
    
    /** Inner class for (position, time) key. */
    private static class TimePosition {
        final Position pos;
        final int time;
        
        TimePosition(Position pos, int time) {
            this.pos = pos;
            this.time = time;
        }
        
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (!(obj instanceof TimePosition)) return false;
            TimePosition other = (TimePosition) obj;
            return time == other.time && pos.equals(other.pos);
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(pos, time);
        }
    }
}
