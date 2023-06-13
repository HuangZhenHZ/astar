#include <bits/stdc++.h>
#include <raylib.h>

const double pi = acos(-1);

double NormalizeAngle(double x) {
  x -= int(x / (2 * pi)) * (2 * pi);
  while (x < -pi) { x += 2 * pi; }
  while (x >= pi) { x -= 2 * pi; }
  return x;
}

double Normalize2022Pi(double x) {
  x -= int(x / (2 * pi)) * (2 * pi);
  while (x < 0) { x += 2 * pi; }
  while (x >= 2 * pi) { x -= 2 * pi; }
  return x;
}

double Sqr(double x) {
  return x * x;
}

struct Vec {
  double x = 0, y = 0;
  Vec() = default;
  Vec(double x, double y) : x(x), y(y) {}
  double Len() const {
    return sqrt(x * x + y * y);
  }
  double DistanceToPoint(const Vec &a) const {
    return sqrt(Sqr(x - a.x) + Sqr(y - a.y));
  }
  Vec operator+ (const Vec &a) const {
    return Vec(x + a.x, y + a.y);
  }
  Vec operator- (const Vec &a) const {
    return Vec(x - a.x, y - a.y);
  }
  void operator+= (const Vec &a) {
    x += a.x;
    y += a.y;
  }
  void operator-= (const Vec &a) {
    x -= a.x;
    y -= a.y;
  }
  Vec operator* (double t) const {
    return Vec(x * t, y * t);
  }
  Vec operator/ (double t) const {
    return Vec(x / t, y / t);
  }
  void operator*= (double t) {
    x *= t;
    y *= t;
  }
  void operator/= (double t) {
    x /= t;
    y /= t;
  }
  double Inner(const Vec &a) const {
    return x * a.x + y * a.y;
  }
  double Cross(const Vec &a) const {
    return x * a.y - y * a.x;
  }
  Vec ComplexMul(const Vec &a) const {
    return Vec(x * a.x - y * a.y, x * a.y + y * a.x);
  }
  Vec ComplexMul(double ax, double ay) const {
    return Vec(x * ax - y * ay, x * ay + y *ax);
  }
};

class Angle {
public:
  explicit Angle(double angle = 0.0) : angle_(angle), unit_vec_(std::cos(angle), std::sin(angle)) {}
  double angle() const { return angle_; }
  Vec unit_vec() const { return unit_vec_; }
  double sin() const { return unit_vec_.y; }
  double cos() const { return unit_vec_.x; }
  Angle operator+ (const Angle &a) const {
    return Angle(angle_ + a.angle_, unit_vec_.ComplexMul(a.unit_vec_));
  }
  Angle operator- (const Angle &a) const {
    return Angle(angle_ - a.angle_, unit_vec_.ComplexMul(a.unit_vec_.x, -a.unit_vec_.y));
  }
  void operator+= (const Angle &a) {
    angle_ += a.angle_;
    unit_vec_ = unit_vec_.ComplexMul(a.unit_vec_);
  }
  void operator-= (const Angle &a) {
    angle_ -= a.angle_;
    unit_vec_ = unit_vec_.ComplexMul(a.unit_vec_.x, -a.unit_vec_.y);
  }
private:
  Angle(double angle, Vec unit_vec) : angle_(angle), unit_vec_(unit_vec) {}
  double angle_;
  Vec unit_vec_;
};

struct Segment {
  Vec a, b;
  Segment(const Vec &a, const Vec &b) : a(a), b(b) {}
  Vec MidPoint() const {
    return (a + b) * 0.5;
  }
};

struct Box {
  Vec center;
  Angle heading;
  double length = 0;
  double width = 0;
  bool HasOverlapWithSegment(const Segment &segment) const {
    Vec transformed_midpoint = (segment.MidPoint() - center).ComplexMul(heading.cos(), -heading.sin());
    Vec transformed_segment_vec = (segment.b - segment.a).ComplexMul(heading.cos(), -heading.sin());
    if (std::abs(transformed_midpoint.x) * 2 > std::abs(transformed_segment_vec.x) + length) {
      return false;
    }
    if (std::abs(transformed_midpoint.y) * 2 > std::abs(transformed_segment_vec.y) + width) {
      return false;
    }
    if (std::abs(transformed_segment_vec.Cross(transformed_midpoint)) * 2 >
      std::abs(transformed_segment_vec.x) * width + std::abs(transformed_segment_vec.y) * length) {
      return false;
    }
    return true;
  }
};

void BoxTest() {
  Segment segment1 = Segment(Vec(0,0), Vec(20, 20));
  Box Box1 = Box{
    .center = Vec(20, 20),
    .heading = Angle(pi * 0.5),
    .length = 20,
    .width = 10,
  };
  assert(Box1.HasOverlapWithSegment(segment1));
}

struct CarPoseTransform {
  Vec movement = Vec(0.0, 0.0);
  Angle heading_diff = Angle(0.0);
  CarPoseTransform() = default;
  CarPoseTransform(const Vec &movement, const Angle &heading_diff) : movement(movement), heading_diff(heading_diff) {}
  
  static CarPoseTransform FromConstCurvature(double signed_curvature, double signed_distance) {
    if (std::abs(signed_curvature) < 1e-6) {
      return CarPoseTransform(Vec(signed_distance, 0.0), Angle(0.0));
    }
    Angle heading_diff(signed_curvature * signed_distance);
    double signed_radius = 1.0 / signed_curvature;
    return CarPoseTransform(Vec(signed_radius * heading_diff.sin(), signed_radius * (1 - heading_diff.cos())), heading_diff);
  }
};

struct CarPose {
  Vec position = Vec(0.0, 0.0);
  Angle heading = Angle(0.0);
  CarPose() = default;
  CarPose(const Vec &position, const Angle &heading) : position(position), heading(heading) {}
  
  void Perform(const CarPoseTransform &transform) {
    position += transform.movement.ComplexMul(heading.unit_vec());
    heading += transform.heading_diff;
  }
};

class CarModel {
public:
  double length() const { return length_; }
  double ra_to_front() const { return ra_to_front_; }
  double ra_to_rear() const { return ra_to_rear_; }
  double ra_to_center() const { return ra_to_center_; }
  double width() const { return width_; }
  Box GetCarBox(const CarPose &car_pose) const {
    return Box{
      .center = car_pose.position + car_pose.heading.unit_vec() * ra_to_center_,
      .heading = car_pose.heading,
      .length = length_,
      .width = width_,
    };
  }
private:
  double length_ = 5.0;
  double ra_to_front_ = 4.0;
  double ra_to_rear_ = 1.0;
  double ra_to_center_ = 1.5;
  double width_ = 2.0;
};

struct Painter {
  double width = 100;
  double height = 100;
  double scale = 15;
  std::vector<std::pair<Segment, Color>> segments_and_colors;
  void AddSegmentWithColor(const Segment &segment, const Color &color) {
    segments_and_colors.emplace_back(segment, color);
  }
  void AddSegmentWithColor(const Vec &a, const Vec &b, const Color &color) {
    segments_and_colors.emplace_back(Segment(a, b), color);
  }
  void Draw() {
    InitWindow(width * scale, height * scale, "window");
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
      BeginDrawing();
      ClearBackground(RAYWHITE);
      for (const auto& [segment, color] : segments_and_colors) {
        DrawLine(segment.a.x * scale, (width - segment.a.y) * scale,
                 segment.b.x * scale, (width - segment.b.y) * scale, color);
      }
      EndDrawing();
    }
    CloseWindow();
  }
};

Segment GetDebugShortSegment(const CarPose &car_pose, double length = 0.5) {
  return {car_pose.position, car_pose.position + car_pose.heading.unit_vec() * length};
}


// ===== solver =====

struct CarState {
  CarPose car_pose = CarPose();
  bool is_reverse = false;
};

class AStarSolver {
public:
  AStarSolver(double xy_grid_size, double theta_grid_size, int curvature_sample_num) :
      xy_grid_size_(xy_grid_size), theta_grid_size_(theta_grid_size), curvature_sample_num_(curvature_sample_num) {}
  
  struct Node {
    Node* from_node = nullptr;
    CarState car_state;
    uint64_t grid_id = 0;
    double g_cost = 0;
    double h_cost = 0;
    double total_cost = 0;
    bool is_closed = false;
  };
  
  struct DataInQueue {
    uint64_t grid_id = 0;
    double total_cost = 0;
    explicit DataInQueue(const Node &node) : grid_id(node.grid_id), total_cost(node.total_cost) {}
    bool operator> (const DataInQueue &a) const {
      return total_cost > a.total_cost;
    }
  };
  
  double ComputeH(const CarState &car_state) const {
    return std::max(car_state.car_pose.position.DistanceToPoint(final_state_.car_pose.position),
      std::abs(NormalizeAngle(car_state.car_pose.heading.angle() - final_state_.car_pose.heading.angle())) * 5.0);
  }
  
  uint64_t ComputeGridId(const CarState &car_state) const {
    uint64_t x_grid = car_state.car_pose.position.x / xy_grid_size_;
    uint64_t y_grid = car_state.car_pose.position.y / xy_grid_size_;
    uint64_t theta_grid = Normalize2022Pi(car_state.car_pose.heading.angle()) / theta_grid_size_;
    constexpr uint64_t kXGridBase = 1e8;
    constexpr uint64_t kYGridBase = 1e4;
    constexpr uint64_t kThetaGridBase = 1;
    return x_grid * kXGridBase + y_grid * kYGridBase + theta_grid * kThetaGridBase;
  }
  
  std::unique_ptr<Node> ConstructNewNode(Node* from_node, const CarState &car_state, double transition_cost) {
    std::unique_ptr<Node> node = std::make_unique<Node>();
    node->from_node = from_node;
    node->car_state = car_state;
    node->grid_id = ComputeGridId(car_state);
    node->g_cost = (from_node ? from_node->g_cost : 0) + transition_cost;
    node->h_cost = ComputeH(car_state);
    node->total_cost = node->h_cost + node->g_cost;
    return node;
  }
  
  void ProcessNewNode(std::unique_ptr<Node>&& node) {
    assert(node != nullptr);
    if (nodes_.count(node->grid_id)) {
      Node* same_grid_node = nodes_[node->grid_id].get();
      if (same_grid_node->is_closed || same_grid_node->total_cost < node->total_cost) {
        return;
      }
    }
    queue_.push(DataInQueue(*node));
    nodes_[node->grid_id] = std::move(node);
  }
  
  void PrintPath(Node* node) {
    if (!node) {
      return;
    }
    Vec last_position = node->car_state.car_pose.position;
    for (; node; node = node->from_node) {
      printf("(%lf, %lf, %lf) %lf\n",
        node->car_state.car_pose.position.x,
        node->car_state.car_pose.position.y,
        node->car_state.car_pose.heading.angle(),
        node->g_cost);
      // painter_.AddSegment(last_position, node->car_state.car_pose.position);
      last_position = node->car_state.car_pose.position;
    }
  }
  
  bool HasCollision(const CarPose &car_pose) const {
    Box car_box = car_model_.GetCarBox(car_pose);
    for (const auto &obstacle_segment : *obstacle_segments_) {
      if (car_box.HasOverlapWithSegment(obstacle_segment)) {
        return true;
      }
    }
    return false;
  }
  
  bool IsGoalReached(const CarState &car_state) const {
    return car_state.car_pose.position.DistanceToPoint(final_state_.car_pose.position) < xy_grid_size_ * 1.5 &&
        std::abs(NormalizeAngle(car_state.car_pose.heading.angle() - final_state_.car_pose.heading.angle())) < theta_grid_size_;
  }
  
  bool Solve(std::vector<CarState> *car_states) {
    while (!queue_.empty()) {
      queue_.pop();
    }
    ProcessNewNode(ConstructNewNode(nullptr, start_state_, 0.0));
    int cnt = 0;
    while (!queue_.empty()) {
      DataInQueue curr_data = queue_.top();
      queue_.pop();
      Node* curr_node = nodes_[curr_data.grid_id].get();
      if (curr_node->is_closed) {
        continue;
      }
      curr_node->is_closed = true;
      cnt++;
      if (cnt > 300000) {
        Painter painter;
        painter.AddSegmentWithColor(GetDebugShortSegment(start_state_.car_pose), BLACK);
        painter.AddSegmentWithColor(GetDebugShortSegment(final_state_.car_pose), BLACK);
        for (const Segment &segment : *obstacle_segments_) {
          painter.AddSegmentWithColor(segment, BLACK);
        }
        painter.Draw();
        return false;
      }
      // if (curr_node->h_cost < 0.5) {
      // if (curr_node->grid_id == ComputeGridId(final_state_)) {
      if (IsGoalReached(curr_node->car_state)) {
        if (car_states != nullptr) {
          car_states->clear();
          for (Node* node = curr_node; node != nullptr; node = node->from_node) {
            car_states->emplace_back(node->car_state);
          }
          std::reverse(car_states->begin(), car_states->end());
        }
        // PrintPath(curr_node);
        printf("total cost = %lf\n", curr_node->total_cost);
        printf("%llu\n", nodes_.size());
        printf("%d\n", cnt);
        return true;
      }
      for (int i = -curvature_sample_num_; i <= curvature_sample_num_; ++i) {
        double signed_curvature = 0.2 / curvature_sample_num_ * i;
        for (double signed_distance = -xy_grid_size_ * 1.6; signed_distance < xy_grid_size_ * 2; signed_distance += xy_grid_size_ * 0.8) {
          if (std::abs(signed_distance) < 0.01) {
            continue;
          }
          CarState next_state = curr_node->car_state;
          next_state.car_pose.Perform(CarPoseTransform::FromConstCurvature(signed_curvature, signed_distance));
          if (HasCollision(next_state.car_pose)) {
            continue;
          }
          next_state.is_reverse = signed_distance < 0;
          double cost = std::abs(signed_distance);
          if (curr_node->car_state.is_reverse != next_state.is_reverse) {
            cost += 1.0;
          }
          ProcessNewNode(ConstructNewNode(curr_node, next_state, cost));
        }
      }
    }
    return false;
  }

  const double xy_grid_size_;
  const double theta_grid_size_;
  const int curvature_sample_num_;
  
  std::priority_queue<DataInQueue, std::vector<DataInQueue>, std::greater<>> queue_;
  std::unordered_map<uint64_t, std::unique_ptr<Node>> nodes_;
  // Painter painter_;
  // Input
  CarModel car_model_;
  CarState start_state_, final_state_;
  std::vector<Segment>* obstacle_segments_;
};

void PrintPath(Painter *painter, const std::vector<CarState>& car_states, const Color &color) {
  for (int i = 0; i + 1 < int(car_states.size()); ++i) {
    painter->AddSegmentWithColor(car_states[i].car_pose.position, car_states[i + 1].car_pose.position, color);
  }
}

void Pursuit(std::vector<Segment>* obstacle_segments,
             const std::vector<CarState>& input_states,
             std::vector<CarState>* output_states,
             int start_index,
             int index_step) {
  output_states->clear();
  output_states->emplace_back(input_states.front());
  int target_state_index = start_index;
  while (target_state_index < int(input_states.size())) {
    AStarSolver astar_solver(0.2, 0.02, 2);
    astar_solver.start_state_ = output_states->back();
    astar_solver.final_state_ = input_states[target_state_index];
    astar_solver.obstacle_segments_ = obstacle_segments;
    std::vector<CarState> car_states = {};
    printf("solver status %d\n",astar_solver.Solve(&car_states));
    int j_max = target_state_index + 1 == int(input_states.size()) ? int(car_states.size()) : (int(car_states.size()) + 1) / 2;
    for (int j = 1; j < j_max; ++j) {
      output_states->emplace_back(car_states[j]);
    }
    if (target_state_index + 1 < int(input_states.size())) {
      target_state_index = std::min(target_state_index + index_step, int(input_states.size()) - 1);
    } else {
      break;
    }
  }
}

double GetApproxLength(const std::vector<CarState>& car_states, const CarState &final_state) {
  double length = 0;
  for (int i = 0; i + 1 < int(car_states.size()); ++i) {
    length += car_states[i].car_pose.position.DistanceToPoint(car_states[i + 1].car_pose.position);
  }
  if (!car_states.empty()) {
    length += car_states.back().car_pose.position.DistanceToPoint(final_state.car_pose.position);
  }
  return length;
}

void GenerateFarAwayCase(AStarSolver *astar_solver) {
  astar_solver->start_state_.car_pose = CarPose(Vec(20, 20), Angle(0));
  astar_solver->final_state_.car_pose = CarPose(Vec(80, 80), Angle(0));
  astar_solver->obstacle_segments_->emplace_back(Vec(30, 40), Vec(40, 30));
}

void GenerateKTurnCase(AStarSolver *astar_solver) {
  astar_solver->start_state_.car_pose = CarPose(Vec(50, 50), Angle(pi * 0.5));
  astar_solver->final_state_.car_pose = CarPose(Vec(45, 50), Angle(-pi * 0.5));
  astar_solver->obstacle_segments_->emplace_back(Vec(42, 0), Vec(42, 100));
  astar_solver->obstacle_segments_->emplace_back(Vec(53, 0), Vec(53, 100));
}

void GenerateNarrowRoadCase(AStarSolver *astar_solver) {
  constexpr double road_width = 2.2;
  astar_solver->start_state_.car_pose = CarPose(Vec(60, 30), Angle(1.0));
  astar_solver->final_state_.car_pose = CarPose(Vec(40, 80), Angle(pi * 0.5));
  astar_solver->obstacle_segments_->emplace_back(Vec(40 - road_width * 0.5, 50), Vec(40 - road_width * 0.5, 100));
  astar_solver->obstacle_segments_->emplace_back(Vec(40 + road_width * 0.5, 50), Vec(40 + road_width * 0.5, 100));
  astar_solver->obstacle_segments_->emplace_back(Vec(40 + road_width * 0.5, 50), Vec(100, 50));
  astar_solver->obstacle_segments_->emplace_back(Vec(0, 50), Vec(40 - road_width * 0.5, 50));
}

int main() {
  BoxTest();
  std::vector<Segment> obstacle_segments = {};
  AStarSolver astar_solver(0.5, 0.05, 2);
  astar_solver.obstacle_segments_ = &obstacle_segments;

  GenerateNarrowRoadCase(&astar_solver);

  Painter painter;
  painter.AddSegmentWithColor(GetDebugShortSegment(astar_solver.start_state_.car_pose), BLACK);
  painter.AddSegmentWithColor(GetDebugShortSegment(astar_solver.final_state_.car_pose), BLACK);
  for (const Segment &segment : obstacle_segments) {
    painter.AddSegmentWithColor(segment, BLACK);
  }
  std::vector<CarState> car_states = {};
  printf("%d\n", astar_solver.Solve(&car_states));
  PrintPath(&painter, car_states, BLACK);
  printf("%lf\n", GetApproxLength(car_states, astar_solver.final_state_));
  std::vector<CarState> smooth_states = {};
  car_states.back() = astar_solver.final_state_;
  Pursuit(&obstacle_segments, car_states, &smooth_states, 5, 5);
  PrintPath(&painter, smooth_states, RED);
  printf("%lf\n", GetApproxLength(smooth_states, astar_solver.final_state_));

  car_states = std::move(smooth_states);
  car_states.back() = astar_solver.final_state_;
  Pursuit(&obstacle_segments, car_states, &smooth_states, 5, 5);
  PrintPath(&painter, smooth_states, GREEN);

  for (int i = 0; i < 20; ++i) {
    car_states = std::move(smooth_states);
    car_states.back() = astar_solver.final_state_;
    Pursuit(&obstacle_segments, car_states, &smooth_states, 5, 5);
    printf("%lf\n", GetApproxLength(smooth_states, astar_solver.final_state_));
  }
  PrintPath(&painter, smooth_states, BLUE);

  painter.Draw();
  return 0;
}

