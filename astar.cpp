#include <SFML/Graphics.hpp>
#include <queue>
#include "h.hpp"
#include "rs.hpp"

constexpr double kCarWidth = 22.0;
constexpr double kRaToFront = 40.0;
constexpr double kRaToRear = 10.0;
constexpr double kCarLength = kRaToFront + kRaToRear;
constexpr double kRaToCenter = kCarLength * 0.5 - kRaToRear;
constexpr double kMaxCurvature = 1.0 / 50.0;

constexpr double kXyGridSize = 5;
constexpr double kThetaGridSize = 0.15;

constexpr double kEps = 1e-6;
constexpr double kInf = 1e60;

std::vector<Segment> segments_to_draw;

struct DistanceMap {
	int x_grid_num, y_grid_num;
	int goal_x_idx, goal_y_idx;
	std::vector<std::vector<bool>> obstacle_map;
	std::vector<std::vector<double>> distance_map;

	void InitSize(int n, int m) {
		x_grid_num = n;
		y_grid_num = m;
		obstacle_map.clear();
		obstacle_map.resize(x_grid_num);
		for (int i = 0; i < x_grid_num; ++i) {
			obstacle_map[i].resize(y_grid_num);
			for (int j = 0; j < y_grid_num; ++j) {
				obstacle_map[i][j] = false;
			}
		}
	}

	struct State {
		int x, y;
		double dis;
		int from_x, from_y;

		State(int x, int y, double dis, int from_x, int from_y)
			: x(x), y(y), dis(dis), from_x(from_x), from_y(from_y) {}

		bool operator< (const State &that) const {
			return dis > that.dis;
		}
		
		void DrawFrom() const {
			Vec point1((from_x + 0.5) * kXyGridSize, (from_y + 0.5) * kXyGridSize);
			Vec point2((x + 0.5) * kXyGridSize, (y + 0.5) * kXyGridSize);
			segments_to_draw.push_back(Segment(point1, point2));
		}
	};

	static constexpr int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
	static constexpr int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

	void Dijkstra() {
		std::priority_queue<State> q;
		while (!q.empty()) {
			q.pop();
		}
		distance_map.clear();
		distance_map.resize(x_grid_num);
		for (int i = 0; i < x_grid_num; ++i) {
			distance_map[i].resize(y_grid_num);
			for (int j = 0; j < y_grid_num; ++j) {
				distance_map[i][j] = kInf;
			}
		}
		assert(!obstacle_map[goal_x_idx][goal_y_idx]);
		distance_map[goal_x_idx][goal_y_idx] = 0.0;
		q.push(State(goal_x_idx, goal_y_idx, 0.0, goal_x_idx, goal_y_idx));

		while (!q.empty()) {
			State state = q.top();
			q.pop();
			if (state.dis != distance_map[state.x][state.y]) {
				continue;
			}
			// int from_dx = state.x - state.from_x;
			// int from_dy = state.y - state.from_y;
			// double from_angle = (from_dx == 0 && from_dy == 0) ? 0.0 : atan2(from_dy, from_dx);
			
			for (int i = 0; i < 8; ++i) {
				int nx = state.x + dx[i];
				int ny = state.y + dy[i];
				if (nx < 0 || nx >= x_grid_num || ny < 0 || ny >= y_grid_num) {
					continue;
				}
				if (obstacle_map[nx][ny]) {
					continue;
				}
				/*
				double this_angle = atan2(dy[i], dx[i]);
				if (abs(NormalizeAngle(this_angle - from_angle)) < kPI / 4 + kEps) {
					double total_dx = from_dx + dx[i];
					double total_dy = from_dy + dy[i];
					double ndis = distance_map[state.from_x][state.from_y] +
					              sqrt(total_dx * total_dx + total_dy * total_dy);
					if (ndis >= distance_map[nx][ny]) {
						continue;
					}
					distance_map[nx][ny] = ndis;
					State state_to_push(nx, ny, ndis, state.from_x, state.from_y);
					q.push(state_to_push);
					if (abs(total_dx) + abs(total_dy) < 50) {
						state_to_push.DrawFrom();
					}
					continue;
				}
				*/
				double ndis = state.dis + sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
				if (ndis >= distance_map[nx][ny]) {
					continue;
				}
				distance_map[nx][ny] = ndis;
				q.push(State(nx, ny, ndis, state.x, state.y));
			}
		}
	}
};

struct CarState {
	Vec position, direction;
	double heading;
	CarState(): position(0.0, 0.0), direction(0.0, 0.0), heading(0.0) {}

	Box CarBox() const {
		Vec center = position + direction * kRaToCenter;
		return Box(center, heading, kCarLength, kCarWidth);
	}
};

CarState Trans(const CarState &from_state, double signed_curvature, double signed_distance) {
	/*
	double heading_change = signed_curvature * signed_distance;
	next_state.heading = NormalizeAngle(from_state.heading + heading_change);
	next_state.direction = Vec::Direction(next_state.heading);
	next_state.position = from_state.position + Vec::Direction((from_state.heading + next_state.heading) * 0.5) * signed_distance;
	return next_state;
	*/

	CarState next_state;
	double heading_change = signed_curvature * signed_distance;
	next_state.heading = NormalizeAngle(from_state.heading + heading_change);
	next_state.direction = Vec::Direction(next_state.heading);
	if (abs(heading_change) < kEps) {
		next_state.position = from_state.position + from_state.direction * signed_distance;
		return next_state;
	}
	double signed_radius = 1.0 / signed_curvature;
	next_state.position = from_state.position + from_state.direction * signed_radius * sin(heading_change)
	                      + from_state.direction.Rotate90() * signed_radius * (1 - cos(heading_change));
	return next_state;
}

struct AStarState : CarState {
	int x_grid, y_grid, theta_grid;
	double g_cost, h_cost, total_cost;
	int id, from_id;

	AStarState (const CarState &state): CarState(state) {
		x_grid = static_cast<int>(state.position.x / kXyGridSize);
		y_grid = static_cast<int>(state.position.y / kXyGridSize);
		theta_grid = static_cast<int>((state.heading + kPI) / kThetaGridSize);
		g_cost = h_cost = total_cost = 0.0;
	}
};

bool operator> (const AStarState &a, const AStarState &b) {
	return a.total_cost > b.total_cost;
}

void DrawBox(Box box) {
	std::vector<Vec> corners = box.GetCorners();
	for (int i = 0; i < 4; ++i) {
		segments_to_draw.push_back(Segment(corners[i], corners[(i + 1) % 4]));
	}
}

struct Solver {
	std::priority_queue<AStarState, std::vector<AStarState>, std::greater<AStarState>> q_;
	std::vector<AStarState> states_;
	std::vector<Segment> segs_;
	CarState start_state_, final_state_;
	double w_, h_;
	int final_id = -1;
	DistanceMap distance_map_;

	void InitDistanceMap() {
		int x_grid_num = static_cast<int>(w_ / kXyGridSize) + 1;
		int y_grid_num = static_cast<int>(h_ / kXyGridSize) + 1;
		distance_map_.InitSize(x_grid_num, y_grid_num);
		for (int i = 0; i < x_grid_num; ++i) {
			for (int j = 0; j < y_grid_num; ++j) {
				double x = (i + 0.5) * kXyGridSize;
				double y = (j + 0.5) * kXyGridSize;
				Box box(Vec(x, y), 0.0, kXyGridSize * 2, kXyGridSize * 2);
				if (HasCollision(box)) {
					distance_map_.obstacle_map[i][j] = true;
					// DrawBox(box);
				}
			}
		}
		AStarState final_state(final_state_);
		distance_map_.goal_x_idx = final_state.x_grid;
		distance_map_.goal_y_idx = final_state.y_grid;
		distance_map_.Dijkstra();
	}

	bool IsGoalReached(const CarState &state) const {
		return abs(state.position.x - final_state_.position.x) <= kXyGridSize &&
		       abs(state.position.y - final_state_.position.y) <= kXyGridSize &&
		       abs(state.heading - final_state_.heading) <= kThetaGridSize;
	}

	bool IsInRange(const CarState &state) const {
		return 0 < state.position.x && state.position.x < w_ &&
		       0 < state.position.y && state.position.y < h_;
	}

	bool HasCollision(const Box &box) const {
		for (const auto &seg : segs_) {
			if (box.HasOverlapWith(seg)) {
				return true;
			}
		}
		return false;
	}

	bool HasCollision(const CarState &state) const {
		Box carbox = state.CarBox();
		return HasCollision(carbox);
	}

	double ComputeH(const CarState &state) const {
		Vec p = final_state_.position - state.position;
		double x = Dot(state.direction, p);
		double y = Cross(state.direction, p);
		double h1 = rs::GetMinDis(x * kMaxCurvature, y * kMaxCurvature, final_state_.heading - state.heading) / kMaxCurvature;
		
		// return h1 * 1.02;
		
		AStarState astar_state(state);
		double h2 = distance_map_.distance_map[astar_state.x_grid][astar_state.y_grid] * kXyGridSize * cos(kPI / 8) - kXyGridSize;

		return std::max(h1, h2);
		
		/*
		const double dist = (state.position - final_state_.position).Length();
		double heading_change = abs(state.heading - final_state_.heading);
		heading_change = std::min(heading_change, k2PI - heading_change);
		return std::max(dist, heading_change / kMaxCurvature);
		*/
	}

	bool qc_[350][200][70];

	void TryPush(AStarState& state, int from_id) {
		if (qc_[state.x_grid][state.y_grid][state.theta_grid]) {
			return;
		}
		state.id = states_.size();
		state.from_id = from_id;
		qc_[state.x_grid][state.y_grid][state.theta_grid] = true;
		q_.push(state);
		states_.push_back(state);
	}

	bool Solve() {
		InitDistanceMap();
		memset(qc_, 0, sizeof(qc_));
		while (!q_.empty()) {
			q_.pop();
		}
		AStarState start_state(start_state_);
		start_state.h_cost = ComputeH(start_state_);
		start_state.total_cost = start_state.g_cost + start_state.h_cost;
		TryPush(start_state, -1);

		int cnt = 0;

		while (!q_.empty()) {
			AStarState state = q_.top();
			q_.pop();
			/*
			if (IsGoalReached(state)) {
				printf("cnt = %d\n", cnt);
				final_id = state.id;
				return true;
			}
			*/
			cnt++;
			// printf("%d %d %d\n", state.x_grid, state.y_grid, state.theta_grid);
			// segments_to_draw.push_back(Segment(state.position, state.position + state.direction * 5.0));

			constexpr double kStepLength = kXyGridSize * 1.5;

			for (int i = -1; i <= 1; ++i) {
				for (int j = 1; j >= -1; j -= 2) {
					CarState next_car_state = Trans(state, i * kMaxCurvature, j * kStepLength);
					if (!IsInRange(next_car_state)) {
						continue;
					}
					if (HasCollision(next_car_state)) {
						continue;
					}
					AStarState next_astar_state(next_car_state);
					next_astar_state.g_cost = state.g_cost + kStepLength;
					next_astar_state.h_cost = ComputeH(next_car_state);
					next_astar_state.total_cost = next_astar_state.g_cost + next_astar_state.h_cost;
					TryPush(next_astar_state, state.id);

					if (IsGoalReached(next_astar_state)) {
						printf("cnt = %d\n", cnt);
						final_id = next_astar_state.id;
						return true;
					}
				}
			}
		}

		printf("cnt = %d\n", cnt);

		return false;
	}

	void Draw() {
		Vec last_point(100.0, 100.0);
		for (int id = final_id; id >= 0; id = states_[id].from_id) {
			printf("%d %lf %lf\n", id, states_[id].h_cost, distance_map_.distance_map[states_[id].x_grid][states_[id].y_grid]);
			// DrawBox(states_[id].CarBox());
			if (id != final_id) {
				segments_to_draw.push_back(Segment(last_point, states_[id].position));
			}
			last_point = states_[id].position;
		}
	}

} solver;

int main() {
	sf::Clock clock;
	sf::Time elapsed0 = clock.getElapsedTime();
	printf("%f\n", elapsed0.asSeconds());

	solver.w_ = 1600;
	solver.h_ = 900;
	solver.start_state_.position = Vec(200, 600);
	solver.start_state_.heading = 0.0;
	solver.start_state_.direction = Vec::Direction(solver.start_state_.heading);
	solver.final_state_.position = Vec(1200, 200);
	solver.final_state_.heading = 2.0;
	solver.final_state_.direction = Vec::Direction(solver.final_state_.heading);
	// std::swap(solver.start_state_, solver.final_state_);
	// std::swap(solver.start_state_, solver.final_state_);

	 solver.segs_.push_back(Segment(Vec(600, 200), Vec(600, 900)));
	 solver.segs_.push_back(Segment(Vec(800, 0), Vec(800, 700)));

	printf("%d\n", (int)solver.Solve());
	solver.Draw();

	for (const auto seg : solver.segs_) {
		segments_to_draw.push_back(seg);
	}

	sf::Time elapsed1 = clock.getElapsedTime();
	printf("%f\n", elapsed1.asSeconds());

	sf::RenderWindow window(sf::VideoMode(1600, 900), "HZ", sf::Style::Titlebar | sf::Style::Close);
	window.setVerticalSyncEnabled(true);

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
		}

		window.clear();

		for (auto seg : segments_to_draw) {
			sf::Vertex line[] = {
				sf::Vertex(sf::Vector2f(seg.a.x, seg.a.y)),
				sf::Vertex(sf::Vector2f(seg.b.x, seg.b.y))
			};
			window.draw(line, 2, sf::Lines);
		}

		window.display();
	}
	return 0;
}