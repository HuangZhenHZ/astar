#pragma once

#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>

constexpr double kPI = 3.1415926535897932384626;
constexpr double k2PI = kPI * 2;
constexpr double kPI2 = kPI / 2;

double NormalizeAngle(double angle) {
	angle -= static_cast<int>(angle / k2PI) * k2PI;
	while (angle < -kPI) {
		angle += k2PI;
	}
	while (angle >= kPI) {
		angle -= k2PI;
	}
	return angle;
}

struct Vec {
	double x, y;
	Vec(double x, double y): x(x), y(y) {}
	static Vec Direction(double heading) {
		return Vec(cos(heading), sin(heading));
	}
	Vec Rotate90() const {
		return Vec(-y, x);
	}
	Vec Rotate270() const {
		return Vec(y, -x);
	}
	double Length() const {
		return sqrt(x * x + y * y);
	}
	double SqrLen() const {
		return x * x + y * y;
	}
	double Dot(const Vec &v) const {
		return x * v.x + y * v.y;
	}
	double Cross(const Vec &v) const {
		return x * v.y - y * v.x;
	}
};

Vec operator+ (const Vec &a, const Vec &b) {
	return Vec(a.x + b.x, a.y + b.y);
}

Vec operator- (const Vec &a, const Vec &b) {
	return Vec(a.x - b.x, a.y - b.y);
}

Vec operator* (const Vec &a, double t) {
	return Vec(a.x * t, a.y * t);
}

Vec operator* (double t, const Vec &a) {
	return Vec(a.x * t, a.y * t);
}

Vec operator/ (const Vec &a, double t) {
	return Vec(a.x / t, a.y / t);
}

Vec operator/ (double t, const Vec &a) {
	return Vec(a.x / t, a.y / t);
}

double Dot(const Vec &a, const Vec &b) {
	return a.x * b.x + a.y * b.y;
}

double Cross(const Vec &a, const Vec &b) {
	return a.x * b.y - a.y * b.x;
}

struct Segment {
	Vec a, b;
	Segment(Vec a, Vec b): a(a), b(b) {}
	Vec MidPoint() const {
		return (a + b) / 2;
	}
};

struct Box {
	Vec center;
	double heading, length, width;

	Box(const Vec &center, double heading, double length, double width)
		: center(center), heading(heading), length(length), width(width) {}

	bool HasOverlapWith(const Segment seg) const {
		Vec direction = Vec::Direction(heading);
		Vec cen_to_mid = seg.MidPoint()  - center;
		Vec vec_of_seg = seg.b - seg.a;
		if (length + abs(Dot(direction, vec_of_seg)) < abs(Dot(direction, cen_to_mid)) * 2) {
			return false;
		}
		if (width + abs(Cross(direction, vec_of_seg)) < abs(Cross(direction, cen_to_mid)) * 2) {
			return false;
		}
		return abs(Cross(vec_of_seg, direction * length)) +
		       abs(Cross(vec_of_seg, direction.Rotate90() * width)) >
		       abs(Cross(vec_of_seg, cen_to_mid)) * 2;
	}

	std::vector<Vec> GetCorners() {
		std::vector<Vec> corners;
		Vec direction = Vec::Direction(heading);
		Vec v1 = direction * (length * 0.5);
		Vec v2 = direction.Rotate90() * (width * 0.5);
		corners.push_back(center + v1 - v2);
		corners.push_back(center + v1 + v2);
		corners.push_back(center - v1 + v2);
		corners.push_back(center - v1 - v2);
		return corners;
	}
};

struct Map {
	int w, h;
	std::vector<Segment> segs;

	void ReadFromFile(const char filename[]) {
		FILE *mapfile = fopen(filename, "r");
		assert(fscanf(mapfile, "%d%d", &w, &h) == 2);
		assert(w > 0 && w < 1920);
		assert(h > 0 && h < 1080);
		int n = 0;
		fscanf(mapfile, "%d", &n);
		printf("Map segments count = %d\n", n);
		for (int i = 0; i < n; ++i) {
			double x1, y1, x2, y2;
			assert(fscanf(mapfile, "%lf%lf%lf%lf", &x1, &y1, &x2, &y2) == 4);
			segs.push_back(Segment(Vec(x1, y1), Vec(x2, y2)));
		}
	}
};

