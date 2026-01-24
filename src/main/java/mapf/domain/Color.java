package mapf.domain;

/**
 * Enum representing colors that can be assigned to agents and boxes.
 * Agents can only push/pull boxes of the same color.
 */
public enum Color {
    BLUE,
    RED,
    CYAN,
    PURPLE,
    GREEN,
    ORANGE,
    PINK,
    GREY,
    LIGHTBLUE,
    BROWN;
    
    /**
     * Parses a color from its string representation (case-insensitive).
     * 
     * @param s the color name
     * @return the corresponding Color
     * @throws IllegalArgumentException if the color name is not recognized
     */
    public static Color fromString(String s) {
        return switch (s.trim().toLowerCase()) {
            case "blue" -> BLUE;
            case "red" -> RED;
            case "cyan" -> CYAN;
            case "purple" -> PURPLE;
            case "green" -> GREEN;
            case "orange" -> ORANGE;
            case "pink" -> PINK;
            case "grey", "gray" -> GREY;
            case "lightblue" -> LIGHTBLUE;
            case "brown" -> BROWN;
            default -> throw new IllegalArgumentException("Unknown color: " + s);
        };
    }
}
