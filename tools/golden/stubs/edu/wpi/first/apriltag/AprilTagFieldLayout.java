package edu.wpi.first.apriltag;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/** Reads the same JSON as WPILib's AprilTagFieldLayout (only what FieldConstants needs). */
public class AprilTagFieldLayout {
  private final double m_length;
  private final double m_width;
  private final Map<Integer, Pose3d> m_tags = new HashMap<>();

  public AprilTagFieldLayout(Path path) throws IOException {
    JsonNode root = new ObjectMapper().readTree(path.toFile());
    m_length = root.path("field").path("length").asDouble();
    m_width = root.path("field").path("width").asDouble();
    for (JsonNode t : root.path("tags")) {
      JsonNode tr = t.path("pose").path("translation");
      JsonNode q = t.path("pose").path("rotation").path("quaternion");
      m_tags.put(
          t.path("ID").asInt(),
          new Pose3d(
              new Translation3d(tr.path("x").asDouble(), tr.path("y").asDouble(), tr.path("z").asDouble()),
              q.path("W").asDouble(), q.path("X").asDouble(), q.path("Y").asDouble(), q.path("Z").asDouble()));
    }
  }

  public double getFieldLength() {
    return m_length;
  }

  public double getFieldWidth() {
    return m_width;
  }

  @JsonIgnore
  public List<Integer> getTags() {
    return new ArrayList<>(m_tags.keySet());
  }

  public Optional<Pose3d> getTagPose(int id) {
    return Optional.ofNullable(m_tags.get(id));
  }
}
