package mapf.planning.strategy;

/**
 * Classic Hungarian (Kuhn-Munkres) algorithm for optimal assignment in O(n³).
 * 
 * Solves the rectangular assignment problem: given an n×m cost matrix (n ≤ m),
 * find a one-to-one assignment of rows to columns that minimizes total cost.
 * 
 * Handles rectangular matrices (more columns than rows) and Integer.MAX_VALUE
 * as "impossible" costs. If a row cannot be assigned (all costs MAX_VALUE),
 * it is left unassigned (-1 in the result).
 * 
 * Thread-safe: all state is method-local.
 * 
 * Reference: https://en.wikipedia.org/wiki/Hungarian_algorithm
 */
public final class HungarianAlgorithm {

    /** Large sentinel representing "impossible" assignment cost. */
    private static final long INF = (long) Integer.MAX_VALUE * 2;

    private HungarianAlgorithm() {} // Utility class, no instantiation

    /**
     * Computes the optimal (minimum cost) assignment for a rectangular cost matrix.
     *
     * @param cost n×m cost matrix where n ≤ m. cost[i][j] = cost of assigning row i to column j.
     *             Use Integer.MAX_VALUE for impossible assignments.
     * @return int[n] where result[i] = column index assigned to row i, or -1 if unassignable.
     * @throws IllegalArgumentException if cost is null, empty, or n > m.
     */
    public static int[] solve(int[][] cost) {
        if (cost == null || cost.length == 0) {
            throw new IllegalArgumentException("Cost matrix must be non-null and non-empty");
        }

        int n = cost.length;     // number of rows (goals)
        int m = cost[0].length;  // number of columns (boxes)

        if (n > m) {
            throw new IllegalArgumentException("Rows (" + n + ") must not exceed columns (" + m + ")");
        }

        // Pad to square if rectangular (add dummy rows with 0 cost)
        int size = Math.max(n, m);
        long[][] c = new long[size][size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i < n && j < m) {
                    c[i][j] = cost[i][j] == Integer.MAX_VALUE ? INF : cost[i][j];
                } else {
                    c[i][j] = 0; // Dummy entries have 0 cost
                }
            }
        }

        // Kuhn-Munkres with potentials (shortest augmenting path variant)
        // u[i] = potential for row i, v[j] = potential for column j
        long[] u = new long[size + 1];
        long[] v = new long[size + 1];
        int[] colMatch = new int[size + 1]; // colMatch[j] = row assigned to column j (1-indexed, 0 = unassigned)
        int[] way = new int[size + 1];      // way[j] = previous column in augmenting path

        for (int i = 1; i <= size; i++) {
            colMatch[0] = i;
            int j0 = 0; // virtual column
            long[] minv = new long[size + 1]; // min reduced cost to each column
            boolean[] used = new boolean[size + 1];

            for (int j = 0; j <= size; j++) {
                minv[j] = Long.MAX_VALUE;
                used[j] = false;
            }

            do {
                used[j0] = true;
                int i0 = colMatch[j0];
                long delta = Long.MAX_VALUE;
                int j1 = -1;

                for (int j = 1; j <= size; j++) {
                    if (!used[j]) {
                        long cur = c[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                if (j1 == -1) break; // No augmenting path found

                for (int j = 0; j <= size; j++) {
                    if (used[j]) {
                        u[colMatch[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;
            } while (colMatch[j0] != 0);

            // Trace augmenting path back and update matching
            do {
                int j1 = way[j0];
                colMatch[j0] = colMatch[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // Extract result: row → column assignment (0-indexed)
        int[] result = new int[n];
        for (int j = 1; j <= size; j++) {
            if (colMatch[j] > 0 && colMatch[j] <= n) {
                int row = colMatch[j] - 1;
                int col = j - 1;
                // Check if this is a real assignment (not dummy and not impossible)
                if (col < m && cost[row][col] != Integer.MAX_VALUE) {
                    result[row] = col;
                } else {
                    result[row] = -1; // Assigned to dummy or impossible → unassignable
                }
            }
        }

        // Ensure all rows are initialized (safety)
        for (int i = 0; i < n; i++) {
            if (result[i] < 0 || result[i] >= m) {
                result[i] = -1;
            }
        }

        return result;
    }

    /**
     * Computes the total cost of the optimal assignment.
     *
     * @param cost the cost matrix
     * @return the minimum total assignment cost, or Long.MAX_VALUE if not all rows can be assigned
     */
    public static long totalCost(int[][] cost) {
        int[] assignment = solve(cost);
        long total = 0;
        for (int i = 0; i < assignment.length; i++) {
            if (assignment[i] == -1) return Long.MAX_VALUE;
            total += cost[i][assignment[i]];
        }
        return total;
    }
}
