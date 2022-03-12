#include <cmath>
#include <algorithm>

namespace rs {
	const double pi = acos(-1);
	constexpr double inf = 1e60;

	void R(double x, double y, double *r, double *theta) {
		*r = sqrt(x * x + y * y);
		*theta = atan2(y, x);
	}

	double M(double phi) {
		while (phi < -pi) {
			phi += 2 * pi;
		}
		while (phi > pi) {
			phi -= 2 * pi;
		}
		return phi;
	}

	// LSL
	double path1(double x, double y, double phi) {
		double u, t;
		R(x - sin(phi), y - 1 + cos(phi), &u, &t);
		double s = abs(t) + abs(M(phi - t));
		return u + std::min(s, 2 * pi - s);
	}

	// LSR
	double path2(double x, double y, double phi) {
		double u1, t1;
		R(x + sin(phi), y - 1 - cos(phi), &u1, &t1);
		if (u1 < 2) {
			return inf;
		}
		double u = sqrt(u1 * u1 - 4);
		double a = atan2(2, u);
		double t = M(t1 + a);
		double t2 = M(t1 - a + pi);
		return u + std::min(abs(t) + abs(M(phi - t)), abs(t2) + abs(M(phi - t2)));
	}
	
	// LRL
	double path3(double x, double y, double phi) {
		double u1, theta;
		R(x - sin(phi), y - 1 + cos(phi), &u1, &theta);
		if (u1 > 4) {
			return inf;
		}
		double a = acos(u1 / 4);
		double u = pi - 2 * a;
		double t1 = M(theta + pi / 2 + a);
		double t2 = M(theta + pi / 2 - a);
		double v1 = M(phi - (t1 + u));
		double v2 = M(phi - (t2 - u));
		return u + std::min(abs(t1) + abs(v1), abs(t2) + abs(v2));
	}
	
	double GetMinDis(double x, double y, double phi) {
		phi = M(phi);
		double a1 = std::min(std::min(path1(x, y, phi), path2(x, y, phi)), path3(x, y, phi));
		y = -y;
		phi = -phi;
		double a2 = std::min(std::min(path1(x, y, phi), path2(x, y, phi)), path3(x, y, phi));
		return std::min(a1, a2);
	}
	
	double GetMinDis(double x1, double y1, double phi1, double x2, double y2, double phi2) {
		double s1 = sin(phi1);
		double c1 = cos(phi1);
		double x = c1 * x2 + s1 * y2;
		double y = c1 * y2 - x2 * s1;
		return GetMinDis(x, y, M(phi2 - phi1));
	}

}  // namespace rs
