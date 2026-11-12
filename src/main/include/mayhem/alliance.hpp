#pragma once
// Alliance/side variants of an exported blue-origin routine. C++17, header-only.
#include "mayhem/auto_runner.hpp"
#include <vector>

namespace mayhem {
// None: unchanged. Flip: rotate the field 180 degrees about its center (opposite
// alliance on a rotationally symmetric field; FlipPose). Mirror: reflect across the
// long centerline y = width/2 (left/right variant; MirrorPose). MirrorLength: reflect
// across the midfield line x = length/2 (opposite alliance on a mirror-symmetric field).
enum class FieldTransform { None, Flip, Mirror, MirrorLength };
inline double TransformHeading(double heading, FieldTransform t) {
  constexpr double pi = 3.14159265358979323846;
  switch (t) {
    case FieldTransform::Flip: return Wrap(heading + pi);
    case FieldTransform::Mirror: return Wrap(-heading);
    case FieldTransform::MirrorLength: return Wrap(pi - heading);
    default: return heading;
  }
}
inline void TransformPoint(double& x, double& y, FieldTransform t, double fieldLength, double fieldWidth) {
  if (t == FieldTransform::Flip || t == FieldTransform::MirrorLength) x = fieldLength - x;
  if (t == FieldTransform::Flip || t == FieldTransform::Mirror) y = fieldWidth - y;
}
inline Pose TransformPose(Pose pose, FieldTransform t, double fieldLength, double fieldWidth) {
  TransformPoint(pose.x, pose.y, t, fieldLength, fieldWidth);
  pose.heading = TransformHeading(pose.heading, t);
  return pose;
}
// Every field-frame quantity of a goal: pose, PointAt aim point. Marker headings are
// stored separately (see AllianceRoutine); the markers pointer is left untouched.
inline Goal TransformGoal(Goal goal, FieldTransform t, double fieldLength, double fieldWidth) {
  goal.pose = TransformPose(goal.pose, t, fieldLength, fieldWidth);
  TransformPoint(goal.aimX, goal.aimY, t, fieldLength, fieldWidth);
  return goal;
}
inline PathMarker TransformMarker(PathMarker marker, FieldTransform t) {
  if (marker.kind == MarkerKind::Rotation) marker.heading = TransformHeading(marker.heading, t);
  return marker;
}

// Owns transformed copies of a routine's nodes and path markers and exposes them as a
// RoutineView. Construct once at startup (allocates); call Apply() when the alliance
// is known (e.g. autonomousInit), then Start() the runner. Apply() never allocates and
// always re-derives from the blue source, so repeated calls do not accumulate. Fields
// covered: goal pose x/y/heading, aimX/aimY, rotation-marker headings, start pose.
// Transitions, guards, actions, event markers, tolerances and limits are unchanged.
// Never transform measured odometry; it is already in the blue-origin field frame.
class AllianceRoutine {
 public:
  AllianceRoutine(RoutineView blue, double fieldLength, double fieldWidth,
                  FieldTransform transform = FieldTransform::None, Pose blueStart = {})
      : blue_(blue), start_(blueStart), length_(fieldLength), width_(fieldWidth),
        nodes_(blue.nodes, blue.nodes + blue.nodeCount) {
    std::size_t markers = 0;
    for (std::size_t i = 0; i < blue.nodeCount; ++i) markers += blue.nodes[i].goal.markerCount;
    markers_.resize(markers);
    Apply(transform);
  }
  AllianceRoutine(const RoutineInfo& blue, double fieldLength, double fieldWidth,
                  FieldTransform transform = FieldTransform::None)
      : AllianceRoutine(blue.routine, fieldLength, fieldWidth, transform, blue.startPose) {}
  AllianceRoutine(const AllianceRoutine&) = delete;
  AllianceRoutine& operator=(const AllianceRoutine&) = delete;
  AllianceRoutine(AllianceRoutine&&) = default;  // vector storage moves; views stay valid
  AllianceRoutine& operator=(AllianceRoutine&&) = default;

  // Returns false (and applies None) for a mirror/flip without finite positive field size.
  bool Apply(FieldTransform transform) {
    const bool sized = std::isfinite(length_) && std::isfinite(width_) && length_ > 0 && width_ > 0;
    const bool ok = transform == FieldTransform::None || sized;
    if (!ok) transform = FieldTransform::None;
    transform_ = transform;
    std::size_t m = 0;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      const AutoNode& source = blue_.nodes[i];
      AutoNode& node = nodes_[i];
      node = source;
      node.goal = TransformGoal(source.goal, transform, length_, width_);
      if (source.goal.markerCount) {
        node.goal.markers = markers_.data() + m;
        for (std::size_t j = 0; j < source.goal.markerCount; ++j)
          markers_[m++] = TransformMarker(source.goal.markers[j], transform);
      }
    }
    return ok;
  }
  FieldTransform Transform() const { return transform_; }
  RoutineView View() const {
    RoutineView view = blue_; view.nodes = nodes_.data(); return view;
  }
  // The routine's start pose (when constructed from RoutineInfo or given one), transformed.
  Pose StartPose() const { return Resolve(start_); }
  // Transform any other blue-origin pose (preview points, targets) the same way.
  Pose Resolve(Pose blue) const { return TransformPose(blue, transform_, length_, width_); }
  const RoutineView& Blue() const { return blue_; }

 private:
  RoutineView blue_; Pose start_; double length_, width_;
  FieldTransform transform_ = FieldTransform::None;
  std::vector<AutoNode> nodes_; std::vector<PathMarker> markers_;
};
}  // namespace mayhem
