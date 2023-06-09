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
  std::vector<Segment> segments;
  template <class... Args>
  void EmplaceSegment(Args&&... args) {
    segments.emplace_back(std::forward<Args>(args)...);
  }
  void Draw() {
    InitWindow(width * scale, height * scale, "window");
    SetTargetFPS(60);
    int frame_cnt = 0;
    while (!WindowShouldClose()) {
      frame_cnt += 100;
      if (frame_cnt >= int(segments.size())) {
        frame_cnt = 0;
      }
      BeginDrawing();
      ClearBackground(RAYWHITE);
      // for (int i = 0; i < frame_cnt; ++i) {
      //   const Segment &segment = segments[i];
      for (const Segment &segment : segments) {
        DrawLine(segment.a.x * scale, (width - segment.a.y) * scale,
                 segment.b.x * scale, (width - segment.b.y) * scale, BLACK);
      }
      EndDrawing();
    }
    CloseWindow();
  }
};

// ===== solver =====

struct CarState {
  CarPose car_pose = CarPose();
};

class SearchLayerBase {
public:
  SearchLayerBase() = default;
  virtual ~SearchLayerBase() = default;
  virtual uint64_t ComputeGridId(const CarState &car_state) const = 0;
  virtual std::vector<std::pair<CarState, double>> GetNextStatesWithTransitionCosts(const CarState &car_state) const = 0;
};

class SearchLayerCoarse : public SearchLayerBase {
public:
  SearchLayerCoarse() = default;
  ~SearchLayerCoarse() override = default;

  uint64_t ComputeGridId(const CarState &car_state) const override {
    uint64_t x_grid = car_state.car_pose.position.x / xy_grid_size;
    uint64_t y_grid = car_state.car_pose.position.y / xy_grid_size;
    uint64_t theta_grid = Normalize2022Pi(car_state.car_pose.heading.angle()) / theta_grid_size;
    constexpr uint64_t kXGridBase = 1e8;
    constexpr uint64_t kYGridBase = 1e4;
    constexpr uint64_t kThetaGridBase = 1;
    return x_grid * kXGridBase + y_grid * kYGridBase + theta_grid * kThetaGridBase;
  }

  std::vector<std::pair<CarState, double>> GetNextStatesWithTransitionCosts(const CarState &car_state) const override {
    std::vector<std::pair<CarState, double>> result = {};
    for (int i = -curvature_sample_nums; i <= curvature_sample_nums; ++i) {
      double signed_curvature = 0.2 / curvature_sample_nums * i;
      for (double signed_distance = -xy_grid_size * 1.6; signed_distance < xy_grid_size * 2; signed_distance += xy_grid_size * 0.8) {
        if (std::abs(signed_distance) < 0.01) {
          continue;
        }
        CarState next_state = car_state;
        next_state.car_pose.Perform(CarPoseTransform::FromConstCurvature(signed_curvature, signed_distance));
        if (HasCollision(next_state.car_pose)) {
          continue;
        }
        result.emplace_back(next_state, std::abs(signed_distance) * (1 + std::abs(signed_curvature)));
      }
    }
    return result;
  }

  bool HasCollision(const CarPose &car_pose) const {
    Box car_box = car_model_->GetCarBox(car_pose);
    for (const auto &obstacle_segment : *obstacle_segments_) {
      if (car_box.HasOverlapWithSegment(obstacle_segment)) {
        return true;
      }
    }
    return false;
  }

  double xy_grid_size = 1.0;
  double theta_grid_size = 0.2;
  int curvature_sample_nums = 1;
  std::vector<Segment>* obstacle_segments_;
  CarModel* car_model_;
};

class AStarSolver {
public:
  struct Node {
    Node* from_node = nullptr;
    int search_layer = 0;
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

  uint64_t ComputeGridId(const CarState &car_state, int search_layer) const {
    return search_layers_[search_layer]->ComputeGridId(car_state) * uint64_t(search_layers_.size()) + uint64_t(search_layer);
  }
  
  double ComputePoseDistanceH(const CarPose &pose1, const CarPose &pose2) const {
    return std::max(pose1.position.DistanceToPoint(pose2.position),
        std::abs(NormalizeAngle(pose1.heading.angle() - pose2.heading.angle())) * 5.0);
  }

  double ComputeH(const CarState &car_state, int search_layer) const {
    if ((search_layers_.size() ^ search_layer) & 1) {
      return ComputePoseDistanceH(car_state.car_pose, final_state_.car_pose);
    } else {
      return ComputePoseDistanceH(car_state.car_pose, start_state_.car_pose);
    }
  }

  std::unique_ptr<Node> ConstructNewNode(Node* from_node, int search_layer, const CarState &car_state, double transition_cost) const {
    std::unique_ptr<Node> node = std::make_unique<Node>();
    node->from_node = from_node;
    node->search_layer = search_layer;
    node->car_state = car_state;
    node->grid_id = ComputeGridId(car_state, search_layer);
    node->g_cost = (from_node ? from_node->g_cost : 0) + transition_cost;
    // node->h_cost = ComputeH(car_state);
    // node->total_cost = node->h_cost + node->g_cost;
    return node;
  }
  
  void PushToQueue(std::unique_ptr<Node> node) {
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

  void ProcessNewNode(std::unique_ptr<Node> node) {
    assert(node != nullptr);
    if (node->search_layer == 0) {
      node->h_cost = ComputeH(node->car_state, node->search_layer);
      node->total_cost = node->h_cost + node->g_cost;
      PushToQueue(std::move(node));
      return;
    }
    uint64_t prev_level_grid_id = ComputeGridId(node->car_state, node->search_layer - 1);
    if (nodes_.count(prev_level_grid_id) && nodes_[prev_level_grid_id]->is_closed) {
      Node* ref_node = nodes_[prev_level_grid_id].get();
      for (int i = 0; i < 2; ++i) {
        if (ref_node->from_node) {
          ref_node = ref_node->from_node;
        }
      }
      node->h_cost = ComputePoseDistanceH(node->car_state.car_pose, ref_node->car_state.car_pose) + ref_node->g_cost;
      // node->h_cost = ComputeH(node->car_state, node->search_layer);
      node->total_cost = node->h_cost + node->g_cost * 0.9;
      PushToQueue(std::move(node));
    } else {
      pending_nodes_[prev_level_grid_id].emplace_back(std::move(node));
    }
  }

  void ProcessPendingNodesByNewCloseNode(Node* new_closed_node) {
    if (!pending_nodes_.count(new_closed_node->grid_id)) {
      return;
    }
    Node* ref_node = new_closed_node;
    for (int i = 0; i < 2; ++i) {
      if (ref_node->from_node) {
        ref_node = ref_node->from_node;
      }
    }
    
    std::vector<std::unique_ptr<Node>> pending_nodes = std::move(pending_nodes_[new_closed_node->grid_id]);
    for (auto& node : pending_nodes) {
      node->h_cost = ComputePoseDistanceH(node->car_state.car_pose, ref_node->car_state.car_pose) + ref_node->g_cost;
      // node->h_cost = ComputeH(node->car_state, node->search_layer);
      node->total_cost = node->h_cost + node->g_cost * 0.9;
      PushToQueue(std::move(node));
    }
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
      painter_.EmplaceSegment(last_position, node->car_state.car_pose.position);
      last_position = node->car_state.car_pose.position;
    }
  }
  
  bool HasCollision(const CarPose &car_pose) const {
    Box car_box = car_model_.GetCarBox(car_pose);
    for (const auto &obstacle_segment : obstacle_segments_) {
      if (car_box.HasOverlapWithSegment(obstacle_segment)) {
        return true;
      }
    }
    return false;
  }

  bool Solve() {
    for (const Segment &segment : obstacle_segments_) {
      painter_.EmplaceSegment(segment);
    }
    
    search_layer_0_ = std::make_unique<SearchLayerCoarse>();
    search_layer_0_->obstacle_segments_ = &obstacle_segments_;
    search_layer_0_->car_model_ = &car_model_;
    
    search_layer_1_ = std::make_unique<SearchLayerCoarse>();
    search_layer_1_->obstacle_segments_ = &obstacle_segments_;
    search_layer_1_->car_model_ = &car_model_;
    search_layer_1_->xy_grid_size = 0.4;
    search_layer_1_->theta_grid_size = 0.02;
    search_layer_1_->curvature_sample_nums = 4;
    
    search_layers_.emplace_back(search_layer_0_.get());
    search_layers_.emplace_back(search_layer_1_.get());

    while (!queue_.empty()) {
      queue_.pop();
    }
    // for (int i = 0; i < int(search_layers_.size()); ++i) {
    //   ProcessNewNode(ConstructNewNode(nullptr, i, start_state_, 0.0));
    // }
    ProcessNewNode(ConstructNewNode(nullptr, 0, final_state_, 0.0));
    ProcessNewNode(ConstructNewNode(nullptr, 1, start_state_, 0.0));
    int cnt = 0;
    while (!queue_.empty()) {
      cnt++;
      if (cnt % 100000 == 0) {
        printf("cnt = %d\n", cnt);
      }
      
      DataInQueue curr_data = queue_.top();
      queue_.pop();
      Node* curr_node = nodes_[curr_data.grid_id].get();
      if (curr_node->is_closed) {
        continue;
      }
      curr_node->is_closed = true;
      ProcessPendingNodesByNewCloseNode(curr_node);
      // painter_.EmplaceSegment(curr_node->car_state.car_pose.position, curr_node->car_state.car_pose.position + curr_node->car_state.car_pose.heading.unit_vec() * 0.2);
      // if (curr_node->h_cost < 0.5) {
      if (curr_node->search_layer + 1 == int(search_layers_.size()) &&
          search_layers_.back()->ComputeGridId(curr_node->car_state) == search_layers_.back()->ComputeGridId(final_state_)) {
        PrintPath(curr_node);
        printf("g + h cost = %lf\n", curr_node->g_cost + curr_node->h_cost);
        printf("total cost = %lf\n", curr_node->total_cost);
        printf("%llu\n", nodes_.size());
        printf("%d\n", cnt);
        return true;
      }

      std::vector<std::pair<CarState, double>> next_states_with_transition_costs =
          search_layers_[curr_node->search_layer]->GetNextStatesWithTransitionCosts(curr_node->car_state);
      for (const auto &[next_state, transition_cost] : next_states_with_transition_costs) {
        ProcessNewNode(ConstructNewNode(curr_node, curr_node->search_layer, next_state, transition_cost));
      }
    }
    return false;
  }

// private:
  std::priority_queue<DataInQueue, std::vector<DataInQueue>, std::greater<>> queue_;
  std::unordered_map<uint64_t, std::unique_ptr<Node>> nodes_;
  std::unordered_map<uint64_t, std::vector<std::unique_ptr<Node>>> pending_nodes_;
  Painter painter_;
  // Input
  CarModel car_model_;
  CarState start_state_, final_state_;
  std::vector<Segment> obstacle_segments_;

  std::vector<SearchLayerBase*> search_layers_;
  std::unique_ptr<SearchLayerCoarse> search_layer_0_;
  std::unique_ptr<SearchLayerCoarse> search_layer_1_;
};

int main() {
  BoxTest();
  AStarSolver astar_solver;
  astar_solver.start_state_.car_pose = CarPose(Vec(20, 20), Angle(0));
  astar_solver.final_state_.car_pose = CarPose(Vec(80, 80), Angle(0));
  astar_solver.obstacle_segments_.emplace_back(Vec(30, 40), Vec(40, 30));
  printf("%d\n", astar_solver.Solve());
  astar_solver.painter_.Draw();
  return 0;
}

