#include <SFML/Graphics.hpp>
#include "h.hpp"
#include "draw.hpp"

DrawHelper draw_helper;

void DrawArc(Vec position, double heading, double k, double d) {
	constexpr int kSampleNum = 300;
	const double delta_s = d / kSampleNum;
	Vec last_position = position;
	for (int i = 0; i < kSampleNum; ++i) {
		position = position + Vec::Direction(heading) * delta_s;
		heading = NormalizeAngle(heading + k * delta_s);
		draw_helper.Push(Segment(last_position, position));
		last_position = position;
	}
}

void DrawBox(Box box) {
	std::vector<Vec> corners = box.GetCorners();
	for (int i = 0; i < 4; ++i) {
		draw_helper.Push(Segment(corners[i], corners[(i + 1) % 4]));
	}
}

int main() {
	draw_helper.w_ = 1200;
	draw_helper.h_ = 600;
	DrawArc(Vec(300, 300), 0, 0.004, 200);
	DrawArc(Vec(300, 300), 0, 0.000, 200);
	DrawArc(Vec(300, 300), 0, -0.004, 200);
	DrawArc(Vec(300, 300), 0, 0.004, -200);
	DrawArc(Vec(300, 300), 0, 0.000, -200);
	DrawArc(Vec(300, 300), 0, -0.004, -200);
	
	for (int i = -5; i <= 5; ++i) {		
		DrawArc(Vec(900, 300), 0, 0.004 / 5.0 * i, 200);
		DrawArc(Vec(900, 300), 0, 0.004 / 5.0 * i, -200);
	}
	
	DrawBox(Box(Vec(600, 300), 0.0, 1100, 300));
	draw_helper.DrawLoop();
}


