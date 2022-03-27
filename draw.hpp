#include <SFML/Graphics.hpp>
#include "h.hpp"

struct DrawHelper {
	std::vector<Segment> segments_to_draw;
	
	void Push(const Segment &seg) {
		segments_to_draw.push_back(seg);
	}
	
	void DrawLoop() {
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
	}
};

