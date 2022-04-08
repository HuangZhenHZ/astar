#include <SFML/Graphics.hpp>
#include "h.hpp"

struct DrawHelper {
	int w_ = 1600;
	int h_ = 900;
	
	std::vector<std::pair<Segment, sf::Color>> segments_to_draw;
	
	void Push(const Segment &seg, const sf::Color &color = sf::Color::Black) {
		segments_to_draw.push_back(std::make_pair(seg, color));
	}
	
	void DrawLoop() {
		sf::RenderWindow window(sf::VideoMode(w_, h_), "HZ", sf::Style::Titlebar | sf::Style::Close);
		window.setVerticalSyncEnabled(true);
		
		while (window.isOpen()) {
			sf::Event event;
			while (window.pollEvent(event)) {
				if (event.type == sf::Event::Closed) {
					window.close();
				}
			}
			
			window.clear();
			
			window.draw(sf::RectangleShape(sf::Vector2f(1800, 1000)));
			
			for (auto seg_and_color : segments_to_draw) {
				const auto &seg = seg_and_color.first;
				const auto &color = seg_and_color.second;
				sf::Vertex line[] = {
					sf::Vertex(sf::Vector2f(seg.a.x, seg.a.y), color),
					sf::Vertex(sf::Vector2f(seg.b.x, seg.b.y), color)
				};
				window.draw(line, 2, sf::Lines);
			}
			
			window.display();
		}
	}
};

