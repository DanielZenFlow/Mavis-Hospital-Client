package mapf.testutil;

import mapf.client.LevelParser;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public final class LevelFixtures {
    private LevelFixtures() {}

    public static LevelParser.ParseResult parse(Path path) throws IOException {
        try (BufferedReader reader = Files.newBufferedReader(path)) {
            return new LevelParser().parse(reader);
        }
    }
}
