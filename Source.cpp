#include <iostream>
#include <SFML/Graphics.hpp>
#include <filesystem>
#include<queue>
#include <SFML/System.hpp>
using namespace std;
using namespace sf;
// Patch Credentials
const int PatchRows = 26;
const int PatchCols = 14;
const int NumberOfPatches = PatchRows * PatchCols;
const int PatchHeight = 60;
const int PatchWidth = 95;

void StartScreen()
{
	RenderWindow startScreen(VideoMode(950, 800), "", sf::Style::None);

	startScreen.setVerticalSyncEnabled(true);

	startScreen.setFramerateLimit(60);
	Texture startTexture;
	if (!startTexture.loadFromFile("Images\\frontPage.png")) {
		cerr << "Error loading start screen image\n";
		return;
	}

	Texture backgroundTexture;
	if (!backgroundTexture.loadFromFile("Images\\Background.jpg")) {
		cerr << "Error loading background image\n";
		return;
	}

	Texture PlayButton;
	if (!PlayButton.loadFromFile("Images\\Play_Button.png")) {
		cerr << "Error loading play button image\n";
		return;
	}

	Sprite backgroundSprite;
	backgroundSprite.setTexture(backgroundTexture);
	backgroundSprite.setScale(
		950.0f / backgroundTexture.getSize().x,
		800.0f / backgroundTexture.getSize().y
	);

	Sprite startSprite(startTexture);
	startSprite.setScale(
		950.0f / startTexture.getSize().x,
		800.0f / startTexture.getSize().y
	);

	startSprite.setPosition(
		(startScreen.getSize().x - startSprite.getGlobalBounds().width) / 2,
		(startScreen.getSize().y - startSprite.getGlobalBounds().height) / 2 + 30
	);

	Sprite playButtonSprite(PlayButton);
	playButtonSprite.setScale(2.3, 1.5);
	playButtonSprite.setPosition(114, 448);

	float height = 1.0;
	float width = 1.0;
	while (startScreen.isOpen())
	{
		Event e;
		while (startScreen.pollEvent(e))
		{
			if (e.type == Event::Closed)
				startScreen.close();
			if (e.type == Event::MouseButtonPressed && e.mouseButton.button == Mouse::Left)
			{
				Vector2f mousePos = startScreen.mapPixelToCoords(Mouse::getPosition(startScreen));
				if (playButtonSprite.getGlobalBounds().contains(mousePos))
				{
					startScreen.close();
				}
			}
		}

		// Handle hover effect
		Vector2f mousePos = startScreen.mapPixelToCoords(Mouse::getPosition(startScreen));
		if (playButtonSprite.getGlobalBounds().contains(mousePos)) {
			playButtonSprite.setScale(2.5f, 1.7f);
			playButtonSprite.setPosition(90, 440);
		}
		else
		{
			playButtonSprite.setScale(2.3f, 1.5f);
			playButtonSprite.setPosition(114, 448);
		}

		startScreen.clear();
		startScreen.draw(backgroundSprite);
		startScreen.draw(startSprite);
		startScreen.draw(playButtonSprite);
		startScreen.display();
	}

}

void showPopupMessage(const string& message) {
	RenderWindow popup(VideoMode(420, 145), "", Style::None);

	Font font;
	if (!font.loadFromFile("Fonts/Super_Adorable.ttf")) {
		cerr << "Failed to load font.\n";
		return;
	}

	Text text;
	text.setFont(font);
	text.setString(message);
	text.setCharacterSize(30);
	text.setFillColor(Color::Black);
	text.setPosition(29, 50);

	RectangleShape background(Vector2f(420, 145));
	// Set light green background color
	background.setFillColor(Color(255, 255, 200)); // Light yellow background

	while (popup.isOpen()) {
		Event e;
		while (popup.pollEvent(e)) {
			if (e.type == Event::Closed || e.type == Event::KeyPressed) {
				popup.close();
			}
		}

		// You can either clear with a matching color or use the rectangle
		popup.clear(Color(255, 255, 200)); // Matches background color
		// popup.draw(background); // optional if you're clearing with same color
		popup.draw(text);
		popup.display();
	}
}


struct Tile {
	int x;
	int y;
	int weight;
	double g = INT_MAX;
	double f = INT_MAX;
	Tile* cameFrom = nullptr;
	bool inSet = false;
};
struct compareHeuristic {
	bool operator()(Tile* first, Tile* second) {
		return first->f > second->f;
	}
};
int heuristic(Tile* cell, Tile* goal) {
	return abs(cell->x - goal->x) + abs(cell->y - goal->y);
}

double movementCost(Tile* from, Tile* to) {
	if (to->weight == -1)
		return INT_MAX; // Obstacle, don't go

	int dx = std::abs(from->x - to->x);
	int dy = std::abs(from->y - to->y);

	// Diagonal move if both dx and dy are 1
	if (dx == 1 && dy == 1)
		return 1.4f * to->weight; // Diagonal cost
	else
		return 1.0f * to->weight; // Straight move
}
vector<Tile*> getNeighbours(Tile* cell, Tile grid[PatchRows][PatchCols]) {
	// Only 6 directions depending on row parity
	pair<int, int> evenDirections[] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1} };
	pair<int, int> oddDirections[] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, 1},  {1, 1} };

	vector<Tile*> neighbours;
	bool isOdd = (cell->x % 2 != 0);
	auto& directions = isOdd ? oddDirections : evenDirections;

	for (auto d : directions) {
		int nx = cell->x + d.first;
		int ny = cell->y + d.second;

		if (nx >= 0 && nx < PatchRows && ny >= 0 && ny < PatchCols && grid[nx][ny].weight != -1) {
			neighbours.push_back(&grid[nx][ny]);
		}
	}
	return neighbours;
}


vector<Tile*> reconstructPath(Tile* goal) {
	vector<Tile*> path;
	Tile* current = goal;
	while (current != nullptr) {
		path.push_back(current);
		current = current->cameFrom;
	}
	reverse(path.begin(), path.end());
	return path;
}
vector<Tile*> AStar(Tile grid[PatchRows][PatchCols], Tile* start, Tile* goal) {
	priority_queue<Tile*, vector<Tile*>, compareHeuristic> openSet;

	// Reset all tiles
	for (int i = 0; i < PatchRows; i++) {
		for (int j = 0; j < PatchCols; j++) {
			grid[i][j].g = INT_MAX;
			grid[i][j].f = INT_MAX;
			grid[i][j].cameFrom = nullptr;
			grid[i][j].inSet = false;
		}
	}

	start->g = 0;
	start->f = heuristic(start, goal);
	start->inSet = true;
	openSet.push(start);

	while (!openSet.empty()) {
		Tile* current = openSet.top();
		openSet.pop();

		if (current == goal)
			return reconstructPath(current);

		for (Tile* neighbour : getNeighbours(current, grid)) {
			double cost = movementCost(current, neighbour);
			double tentative_g = current->g + cost;

			if (tentative_g < neighbour->g) {
				neighbour->cameFrom = current;
				neighbour->g = tentative_g;
				neighbour->f = tentative_g + heuristic(neighbour, goal);
				if (!neighbour->inSet) {
					neighbour->inSet = true;
					openSet.push(neighbour);
				}
			}
		}
	}

	return { nullptr };  // Path not found
}



const int number_of_drones = 3;
bool finaldestination[number_of_drones] = { false };
bool droneDone[number_of_drones] = { false };
bool packageDone[number_of_drones] = { false };
bool houseDone[number_of_drones] = { false };
bool packagePicked[number_of_drones] = { false };
bool countFinished[number_of_drones] = { false };
bool allDone = false;
bool loadingDone = false;
bool spaceBarPressed = false;
// Building Credentials
const int NumberofBuildings = 4;

class Patch
{
public:

	int up[2];
	int down[2];
	int left[2];
	int right[2];
	float xOffset = 0;
	int row, col;
	bool isBuilding = false;
	bool isWindy = false;
	bool isHouse = false;
	bool isDrone = false;
	bool isPackage = false;
	bool isHighlighted = false;
	bool isPicked = false;
	int buildingNo = -1;


	Patch()
	{
		up[0] = 0; up[1] = 0;
		down[0] = 0; down[1] = 0;
		left[0] = 0; left[1] = 0;
		right[0] = 0; right[1] = 0;
		row = -1;
		col = -1;
		isBuilding = false;
		buildingNo = -1;
	}

	Patch(int u[2], int d[2], int l[2], int r[2], int rIndex = -1, int cIndex = -1)
	{
		for (int i = 0;i < 2;i++)
		{
			up[i] = u[i];
			down[i] = d[i];
			left[i] = l[i];
			right[i] = r[i];
		}
		row = rIndex;
		col = cIndex;
	}

	void drawLine(RenderWindow& window)
	{
		Vertex lines[5];
		lines[0] = Vertex(Vector2f(up[0], up[1]), Color::Red);
		lines[1] = Vertex(Vector2f(left[0], left[1]), Color::Red);
		lines[2] = Vertex(Vector2f(down[0], down[1]), Color::Red);
		lines[3] = Vertex(Vector2f(right[0], right[1]), Color::Red);
		lines[4] = Vertex(Vector2f(up[0], up[1]), Color::Red);
		window.draw(lines, 5, LinesStrip);
	}

	void fillBuilding(RenderWindow& window, int BuildingNo)
	{
		Texture texture;
		bool loaded = false;

		if (!loaded)
		{
			if (BuildingNo == 1)
			{
				if (!texture.loadFromFile("Images\\Building1.png")) {
					cerr << "Error loading texture" << endl;
					return;
				}
			}
			else if (BuildingNo == 2)
			{
				if (!texture.loadFromFile("Images\\Building2.png")) {
					cerr << "Error loading texture" << endl;
					return;
				}
			}
			else if (BuildingNo == 3)
			{
				if (!texture.loadFromFile("Images\\Building3.png")) {
					cerr << "Error loading texture" << endl;
					return;
				}
			}
			else if (BuildingNo == 4)
			{
				if (!texture.loadFromFile("Images\\Building4.png")) {
					cerr << "Error loading texture" << endl;
					return;
				}
			}
			loaded = true;
		}
		int scale_y = 30;
		int scale_x = 0;
		Sprite sprite;
		sprite.setTexture(texture);

		sprite.setOrigin(sprite.getLocalBounds().width / 2, sprite.getLocalBounds().height / 2);
		if (BuildingNo == 1)
		{
			sprite.setScale(0.6f, 0.6f);
		}
		else if (BuildingNo == 2)
		{
			scale_y += 3;
			sprite.setScale(0.55f, 0.55f);
		}
		else if (BuildingNo == 3)
		{
			scale_y += 43;
			scale_x -= 4;
			sprite.setScale(0.56f, 0.63f);
		}
		else if (BuildingNo == 4)
		{
			scale_y += 14;
			scale_x += 2;
			sprite.setScale(0.39f, 0.4f);
		}
		sprite.setPosition((up[0] + down[0]) / 2.0f - scale_x, ((up[1] + down[1]) / 2.0f) - scale_y);

		// Move up/down if key is held
		/*if (Keyboard::isKeyPressed(Keyboard::Up))
			xOffset -= 0.1f;
		if (Keyboard::isKeyPressed(Keyboard::Down))
			xOffset += 0.1f;*/



			//cout << "Sprite Position: " << sprite.getPosition().x << ", " << sprite.getPosition().y << endl;


		// Draw shadow first, then actual sprite
		window.draw(sprite);
	}

	bool checkClear()
	{
		if (isBuilding || isWindy || isHouse || isDrone || isPackage)
			return false;
		return true;
	}

	void fillPackage(RenderWindow& window)
	{
		Texture texture;
		bool loaded = false;

		if (!texture.loadFromFile("Images\\package.png")) {
			cerr << "Error loading package" << endl;
		}
		int scale_y = 0;
		int scale_x = 0;
		Sprite sprite;
		sprite.setTexture(texture);
		sprite.setOrigin(sprite.getLocalBounds().width / 2, sprite.getLocalBounds().height / 2);
		sprite.setScale(0.09f, 0.09f);
		sprite.setPosition((up[0] + down[0]) / 2.0f - scale_x, ((up[1] + down[1]) / 2.0f) - scale_y);
		window.draw(sprite);
	}
	void fillDrone(RenderWindow& window, bool picked = false)
	{
		Texture texture;
		bool loaded = false;
		if (picked)
		{
			if (!texture.loadFromFile("Images\\packageDrone.png")) {
				cerr << "Error loading drone" << endl;
			}
		}
		else
		{
			if (!texture.loadFromFile("Images\\Drone2.png")) {
				cerr << "Error loading drone" << endl;
			}
		}
		int scale_y = 15;
		int scale_x = 0;
		Sprite sprite;
		sprite.setTexture(texture);
		sprite.setOrigin(sprite.getLocalBounds().width / 2, sprite.getLocalBounds().height / 2);
		sprite.setScale(0.28f, 0.28f);
		sprite.setPosition((up[0] + down[0]) / 2.0f - scale_x, ((up[1] + down[1]) / 2.0f) - scale_y);
		Sprite shadow = sprite; // Copy original sprite

		// Set shadow color to semi-transparent black
		shadow.setColor(Color(0, 0, 0, 100)); // RGBA: Black with 100 alpha

		// Slightly offset the shadow
		shadow.move(4.f, 12.f); // Shift 5 pixels right and down (like a shadow)
		window.draw(shadow);
		window.draw(sprite);
	}
	void fillHouse(RenderWindow& window)
	{
		Texture texture;
		bool loaded = false;

		if (!texture.loadFromFile("Images\\house.png")) {
			cerr << "Error loading house" << endl;
		}
		int scale_y = 0;
		int scale_x = 0;
		Sprite sprite;
		sprite.setTexture(texture);
		sprite.setOrigin(sprite.getLocalBounds().width / 2, sprite.getLocalBounds().height / 2);
		sprite.setScale(0.2f, 0.2f);
		sprite.setPosition((up[0] + down[0]) / 2.0f - scale_x, ((up[1] + down[1]) / 2.0f) - scale_y);
		window.draw(sprite);
	}
	void fillWind(RenderWindow& window)
	{
		Texture texture;
		bool loaded = false;

		if (!texture.loadFromFile("Images\\wind.png")) {
			cerr << "Error loading wind" << endl;
		}
		int scale_y = 10;
		int scale_x = 0;
		Sprite sprite;
		sprite.setTexture(texture);
		sprite.setOrigin(sprite.getLocalBounds().width / 2, sprite.getLocalBounds().height / 2);
		sprite.setScale(0.6f, 0.6f);
		sprite.setPosition((up[0] + down[0]) / 2.0f - scale_x, ((up[1] + down[1]) / 2.0f) - scale_y);
		Sprite shadow = sprite; // Copy original sprite

		// Set shadow color to semi-transparent black
		shadow.setColor(Color(0, 0, 0, 100)); // RGBA: Black with 100 alpha

		// Slightly offset the shadow
		shadow.move(5.f, 9.f); // Shift 5 pixels right and down (like a shadow)
		window.draw(shadow);
		window.draw(sprite);
	}
	bool containsPoint(int px, int py) const
	{
		float cx = (left[0] + right[0]) / 2.0f;
		float cy = (up[1] + down[1]) / 2.0f;

		float dx = abs(px - cx) / (PatchWidth / 2.0f);
		float dy = abs(py - cy) / (PatchHeight / 2.0f);

		return (dx + dy <= 1.0f);
	}

};

void DrawPatches(RenderWindow& window, Patch patches[], int count)
{
	for (int i = 0; i < count; i++)
	{
		patches[i].drawLine(window);
	}
}

void DrawBuildings(RenderWindow& window, Patch patches[], int count)
{
	for (int i = 0; i < count; i++)
	{
		if (patches[i].isBuilding)
		{
			patches[i].fillBuilding(window, patches[i].buildingNo);
		}
		if (patches[i].isWindy) {
			patches[i].fillWind(window);
		}
		if (patches[i].isDrone)
		{
			if (patches[i].isPicked)
			{
				patches[i].fillDrone(window, true);
			}
			else
			{
				patches[i].fillDrone(window);
			}
		}
		if (patches[i].isPackage) {
			patches[i].fillPackage(window);
		}
		if (patches[i].isHouse) {
			patches[i].fillHouse(window);
		}
	}
}
void highlightPathTiles(const vector<Tile*>& path, Patch c[], vector<int>& patchPath) {
	patchPath.clear();  // don't use preset size


	for (Tile* tile : path)
	{
		int row = tile->x;
		int col = tile->y;
		for (int i = 0; i < NumberOfPatches; i++)
		{
			if (c[i].row == row && c[i].col == col)
			{
				c[i].isHighlighted = true;
				patchPath.push_back(i);
				break;
			}
		}
	}
}



void highlightPatch(RenderWindow& window, Patch c[])
{
	for (int i = 0; i < NumberOfPatches; i++)
	{
		if (c[i].isHighlighted)
		{
			// Use correct offset logic consistent with patch creation
			float xOffset = (c[i].row % 2 == 0) ? 0 : PatchWidth / 2;
			int px = c[i].col * PatchWidth + PatchWidth / 2 + xOffset;
			int py = c[i].row * (PatchHeight / 2) + PatchHeight / 2;

			ConvexShape diamond;
			diamond.setPointCount(4);
			diamond.setPoint(0, Vector2f(px, py - PatchHeight / 2 + 4)); // Top
			diamond.setPoint(1, Vector2f(px + PatchWidth / 2 - 4, py));   // Right
			diamond.setPoint(2, Vector2f(px, py + PatchHeight / 2 - 4)); // Bottom
			diamond.setPoint(3, Vector2f(px - PatchWidth / 2 + 4, py));   // Left

			diamond.setFillColor(Color(0, 0, 255, 100)); // Semi-transparent blue
			diamond.setOutlineThickness(2);
			diamond.setOutlineColor(Color::Blue);

			window.draw(diamond);
		}
	}
}



bool applyAlgo(vector<Tile*>& temp, Patch c[], Tile grid[PatchRows][PatchCols], int initial_x, int initial_y, int dest_x, int dest_y, vector<int>& patchPath)
{
	vector<Tile*> result = AStar(grid, &grid[initial_x][initial_y], &grid[dest_x][dest_y]);
	if (result[0] == nullptr) {
		return false;
	}
	temp = result;
	highlightPathTiles(temp, c, patchPath);
	return true;
}


void moveDrone(vector<Tile*>& temp, Patch c[], Tile grid[PatchRows][PatchCols], vector<int>& patchPath, int& pathNo, int dest_x, int dest_y, RenderWindow& window, float& progress, int droneNo)
{
	if (pathNo + 1 < patchPath.size())
	{
		int current = patchPath[pathNo];
		int next = patchPath[pathNo + 1];
		if (c[next].isBuilding == true) {
			int initial_x = c[current].row;
			int initial_y = c[current].col;
			pathNo = -1;
			int set[3] = { 0 };
			applyAlgo(temp, c, grid, initial_x, initial_y, dest_x, dest_y, patchPath);
			loadingDone = false;
			spaceBarPressed = true;
			progress = 0;
			return;
		}
		if (!c[next].isDrone)
		{
			if (pathNo != 0)
			{
				int previous = patchPath[pathNo - 1];
				c[previous].isDrone = false;
				c[previous].isHighlighted = false;
			}
			c[current].isHighlighted = false;
			c[current].isDrone = false;
			c[next].isDrone = true;
			if (finaldestination[droneNo])
			{
				c[next].isPicked = true;
			}
		}
		else
		{
			pathNo--;
		}
		if (pathNo + 2 >= patchPath.size()) {
			packagePicked[droneNo] = true;
		}
	}
	else
	{
		if (finaldestination[droneNo]) {
			countFinished[droneNo] = true;
		}
		int current = patchPath[pathNo];
		c[current].isPicked = true;
		c[current].isDrone = false;
		c[current].isHighlighted = false;
		if (c[current].isPackage) {
			c[current].isPackage = false;
		}
		int initial_x = c[current].row;
		int initial_y = c[current].col;
		pathNo = -1;
		applyAlgo(temp, c, grid, initial_x, initial_y, dest_x, dest_y, patchPath);
		finaldestination[droneNo] = true;
	}
}

void PatchHovered(Patch c[], RenderWindow& window, int index, bool allowed)
{
	// Highlight the patch when hovered
	// Draw the highlighted patch
	float xOffset = (c[index].row % 2 == 0) ? 0 : PatchWidth / 2;
	int px = c[index].col * PatchWidth + PatchWidth / 2 + xOffset;
	int py = c[index].row * (PatchHeight / 2) + PatchHeight / 2;
	ConvexShape diamond;
	diamond.setPointCount(4);
	diamond.setPoint(0, Vector2f(px, py - PatchHeight / 2 + 4)); // Top
	diamond.setPoint(1, Vector2f(px + PatchWidth / 2 - 4, py));   // Right
	diamond.setPoint(2, Vector2f(px, py + PatchHeight / 2 - 4)); // Bottom
	diamond.setPoint(3, Vector2f(px - PatchWidth / 2 + 4, py));   // Left
	if (allowed)
		diamond.setFillColor(Color(0, 255, 0, 100)); // Semi-transparent green
	else
	{
		diamond.setFillColor(Color(255, 0, 0, 100)); // Semi-transparent red
	}
	diamond.setOutlineThickness(2);
	diamond.setOutlineColor(Color::Green);
	window.draw(diamond);
}

void setClosestPackage(int droneX[], int droneY[], int packageX[], int packageY[], int houseX[], int houseY[], int set[])
{
	for (int i = 0;i < number_of_drones;i++)
	{
		set[i] = 0;
	}

	for (int i = 0;i < number_of_drones;i++)
	{
		int distance = INT_MAX;
		int closestPackageIndex = -1;
		for (int j = 0;j < number_of_drones;j++)
		{
			int dist = abs(droneX[i] - packageX[j]) + abs(droneY[i] - packageY[j]);
			if (dist < distance && set[j] != 1)
			{
				distance = dist;
				closestPackageIndex = j;
			}
		}
		if (closestPackageIndex != -1)
		{
			int tempX = packageX[i];
			int tempY = packageY[i];
			packageX[i] = packageX[closestPackageIndex];
			packageY[i] = packageY[closestPackageIndex];
			packageX[closestPackageIndex] = tempX;
			packageY[closestPackageIndex] = tempY;
			set[i] = 1;
		}
	}

	for (int i = 0;i < number_of_drones;i++)
	{
		set[i] = 0;
	}

	for (int i = 0;i < number_of_drones;i++)
	{
		int distance = INT_MAX;
		int closestHouseIndex = -1;
		for (int j = 0;j < number_of_drones;j++)
		{
			int dist = abs(packageX[i] - houseX[j]) + abs(packageY[i] - houseY[j]);
			if (dist < distance && set[j] != 1)
			{
				distance = dist;
				closestHouseIndex = j;
			}
		}
		if (closestHouseIndex != -1)
		{
			int tempX = houseX[i];
			int tempY = houseY[i];
			houseX[i] = houseX[closestHouseIndex];
			houseY[i] = houseY[closestHouseIndex];
			houseX[closestHouseIndex] = tempX;
			houseY[closestHouseIndex] = tempY;
			set[i] = 1;
		}
	}
}

int main()
{
	StartScreen();
	RenderWindow window(VideoMode(1280, 800), "", Style::None);
	int index = 0;
	Texture backgroundTexture;
	if (!backgroundTexture.loadFromFile("Images\\Background.jpg")) {
		cerr << "Error loading background image\n";
		return -1;
	}
	window.setFramerateLimit(120); // Increase to 120 FPS (default is unlimited)
	Sprite backgroundSprite;
	backgroundSprite.setTexture(backgroundTexture);
	backgroundSprite.setScale(
		1280.0f / backgroundTexture.getSize().x,
		800.0f / backgroundTexture.getSize().y
	);
	Font font;
	if (!font.loadFromFile("Fonts/Super_Adorable.ttf")) {
		cerr << "Failed to load font!" << endl;
		return -1;
	}

	Text text;
	text.setFont(font);
	text.setCharacterSize(45);
	text.setFillColor(sf::Color(72, 61, 139));
	text.setPosition(420, 675);
	text.setString("Click to place the Drone");

	Text loadingText;
	loadingText.setFont(font);
	loadingText.setString("Generating Path...");
	loadingText.setCharacterSize(40);
	loadingText.setFillColor(sf::Color::Black);
	loadingText.setPosition(510, 640);

	// Background bar
	RectangleShape backgroundBar(sf::Vector2f(400, 30));
	backgroundBar.setFillColor(sf::Color(200, 200, 200));
	backgroundBar.setPosition(460, 700);

	// Foreground bar (progress)
	RectangleShape progressBar(sf::Vector2f(0, 30));  // Start at 0
	progressBar.setFillColor(sf::Color(72, 61, 139));
	progressBar.setPosition(460, 700);

	float progress = 0;


	int packagePosx[number_of_drones] = { 0 }, packagePosy[number_of_drones] = { 0 }, HousePosx[number_of_drones] = { 0 }, HousePosy[number_of_drones] = { 0 }, DronePosx[number_of_drones] = { 0 }, DronePosy[number_of_drones] = { 0 }, DronePatch = 0;

	char terrain[PatchRows][PatchCols] = {
		// Rows 0–2: Blocked zone
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'},
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'},
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'},

		// Rows 3–22: Drone zone (columns 2–11 usable)
		{'C','C','C','C','W','C','C','W','C','C','C','C','C','C'},
		{'C','C','W','C','C','C','#','C','C','W','C','C','C','C'},
		{'C','C','C','C','C','W','C','C','#','C','C','C','C','C'},
		{'C','C','C','#','C','C','W','C','C','C','W','C','C','C'},
		{'C','C','C','C','W','C','C','C','C','#','C','C','C','C'},
		{'C','C','W','C','C','C','C','C','W','C','C','C','C','C'},
		{'C','C','C','C','#','C','C','W','C','C','W','C','C','C'},
		{'C','C','W','C','C','W','C','C','C','W','C','#','C','C'},
		{'C','C','C','C','C','C','#','C','C','C','C','C','C','C'},
		{'C','C','#','C','W','C','C','C','W','C','C','W','C','C'},
		{'C','C','C','W','C','C','C','#','C','C','C','C','C','C'},
		{'C','C','W','C','C','C','C','C','C','#','W','C','C','C'},
		{'C','C','C','C','W','C','W','C','C','C','C','C','C','C'},
		{'C','C','C','#','C','C','C','C','W','C','#','C','C','C'},
		{'C','C','W','C','C','C','W','C','C','C','C','W','C','C'},
		{'C','C','C','C','C','W','C','#','C','C','C','C','C','C'},
		{'C','C','#','C','W','C','C','C','C','W','C','C','C','C'},
		{'C','C','C','C','C','W','C','C','#','C','C','C','C','C'},
		{'C','C','W','C','C','#','C','C','W','C','C','#','C','C'},
		{'C','C','C','C','C','C','C','W','C','C','C','C','C','C'},

		// Rows 23–25: Blocked zone
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'},
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'},
		{'C','C','C','C','C','C','C','C','C','C','C','C','C','C'}
	};
	





	Tile grid[PatchRows][PatchCols];

	// Initialize grid with weights and coordinates
	for (int i = 0; i < PatchRows; ++i)
	{
		for (int j = 0; j < PatchCols; ++j)
		{
			grid[i][j].x = i;
			grid[i][j].y = j;

			if (terrain[i][j] == 'C')
				grid[i][j].weight = 1;
			else if (terrain[i][j] == 'W')
				grid[i][j].weight = 5;
			else if (terrain[i][j] == '#')
				grid[i][j].weight = -1;
			else if (terrain[i][j] == 'P')
				grid[i][j].weight = 1;
			else if (terrain[i][j] == 'D')
				grid[i][j].weight = 1;
			else if (terrain[i][j] == 'H')
				grid[i][j].weight = 1;
		}
	}

	// Generate patches
	// Generate patches (aligned version)
	Patch c[NumberOfPatches];
	index = 0;

	float horizontalSpacing = PatchWidth;
	float verticalSpacing = PatchHeight * 0.5f;

	for (int r = 0; r < PatchRows; r++) {
		for (int col = 0; col < PatchCols; col++) {
			float xOffset = (r % 2 == 0) ? 0 : horizontalSpacing / 2;
			float cx = col * horizontalSpacing + horizontalSpacing / 2 + xOffset;
			float cy = r * verticalSpacing + PatchHeight / 2;

			int up[2] = { static_cast<int>(cx), static_cast<int>(cy - PatchHeight / 2) };
			int down[2] = { static_cast<int>(cx), static_cast<int>(cy + PatchHeight / 2) };
			int left[2] = { static_cast<int>(cx - PatchWidth / 2), static_cast<int>(cy) };
			int right[2] = { static_cast<int>(cx + PatchWidth / 2), static_cast<int>(cy) };

			Patch patch(up, down, left, right, r, col);

			// Terrain logic
			char t = terrain[r][col];
			if (t == '#') {
				patch.isBuilding = true;
				patch.buildingNo = rand() % NumberofBuildings + 1;
			}
			if (t == 'W') patch.isWindy = true;
			if (t == 'H') patch.isHouse = true;
			if (t == 'D') {
				patch.isDrone = true;
				DronePatch = index;
			}
			if (t == 'P') patch.isPackage = true;

			c[index++] = patch;
		}
	}
	int clickNo = 0;
	while (window.isOpen() && !allDone)
	{
		window.draw(backgroundSprite);
		window.draw(text);

		Event e;
		if (window.pollEvent(e))
		{
			if (e.type == Event::Closed)
				window.close();
		}
		// Change the color of the patch when mouse is hovered over it
		if (e.type == Event::MouseMoved)
		{
			int mouseX = e.mouseMove.x;
			int mouseY = e.mouseMove.y;
			for (int i = 0; i < NumberOfPatches; i++)
			{
				if (c[i].containsPoint(mouseX, mouseY))
				{
					if (c[i].checkClear())
					{
						PatchHovered(c, window, i, true);
					}
					else
					{
						PatchHovered(c, window, i, false);
					}
				}
			}
		}

		if (e.MouseButtonPressed)
		{
			if (e.mouseButton.button == Mouse::Left)
			{
				int mouseX = e.mouseButton.x;
				int mouseY = e.mouseButton.y;
				for (int i = 0; i < NumberOfPatches; i++)
				{
					if (c[i].containsPoint(mouseX, mouseY))
					{
						if (c[i].checkClear())
						{
							if (!droneDone[number_of_drones - 1] && !houseDone[number_of_drones - 1] && !packageDone[number_of_drones - 1])
							{
								c[i].isDrone = true;
								DronePosx[clickNo] = c[i].row;
								DronePosy[clickNo] = c[i].col;
								droneDone[clickNo] = true;
								terrain[DronePosx[clickNo]][DronePosy[clickNo]] = 'D';
								grid[DronePosx[clickNo]][DronePosy[clickNo]].weight = 1;
								if (droneDone[number_of_drones - 1])
								{
									text.setString("Click to place the Package");
									text.setFillColor(sf::Color(72, 61, 139));
									text.setPosition(405, 675);
									clickNo = -1;
								}
								clickNo++;
							}
							else if (droneDone[number_of_drones - 1] && !houseDone[number_of_drones - 1] && !packageDone[number_of_drones - 1])
							{
								c[i].isPackage = true;
								packagePosx[clickNo] = c[i].row;
								packagePosy[clickNo] = c[i].col;
								packageDone[clickNo] = true;
								terrain[packagePosx[clickNo]][packagePosy[clickNo]] = 'P';
								grid[packagePosx[clickNo]][packagePosy[clickNo]].weight = 1;
								if (packageDone[number_of_drones - 1])
								{
									text.setString("Click to place the Destination");
									text.setFillColor(sf::Color(72, 61, 139));
									text.setPosition(382, 675);
									clickNo = -1;
								}
								clickNo++;
							}
							else if (droneDone[number_of_drones - 1] && !houseDone[number_of_drones - 1] && packageDone[number_of_drones - 1])
							{
								c[i].isHouse = true;
								HousePosx[clickNo] = c[i].row;
								HousePosy[clickNo] = c[i].col;
								houseDone[clickNo] = true;
								terrain[HousePosx[clickNo]][HousePosy[clickNo]] = 'H';
								grid[HousePosx[clickNo]][HousePosy[clickNo]].weight = 1;
								if (houseDone[number_of_drones - 1])
								{
									text.setString("Press Space to start the delivery");
									// set brown color
									text.setFillColor(sf::Color(139, 69, 19));
									text.setPosition(350, 675);
									allDone = true;
								}
								clickNo++;
							}
						}
					}
				}
			}
		}
		else if (e.type == Event::MouseButtonReleased)
		{
			if (e.mouseButton.button == Mouse::Left)
			{
				index++;
			}
		}
		else if (e.type == Event::KeyPressed)
		{
			if (e.key.code == Keyboard::Escape)
			{
				window.close();
			}
		}
		//DrawPatches(window, c, NumberOfPatches);
		DrawBuildings(window, c, NumberOfPatches);
		window.display();
	}
	int set[number_of_drones] = { 0 };
	setClosestPackage(DronePosx, DronePosy, packagePosx, packagePosy, HousePosx, HousePosy, set);
	vector<Tile*> temp[number_of_drones];
	vector<vector<int>> patchPath(number_of_drones, vector<int>(NumberOfPatches, 0));
	for (int i = 0; i < NumberOfPatches; i++)
	{
		c[i].isHighlighted = false;
	}
	for (int i = 0;i < number_of_drones;i++)
	{
		bool check = applyAlgo(temp[i], c, grid, DronePosx[i], DronePosy[i], packagePosx[i], packagePosy[i], patchPath[i]);
		if (!check) {
			showPopupMessage("Path not found.... Terminating program");
			return 0;
		}
	}
	int in[number_of_drones] = { 0 };
	/*for (int i = 0; i < NumberOfPatches; i++)
	{
		if (c[i].isHighlighted)
		{
			cout << c[i].row << ' ' << c[i].col << ' ' << i << ' ' << patchPath[in++] << endl;
		}
	}*/
	bool ismoveDrone = false;
	bool highlights = false;
	sf::Clock clock;
	const float frameDelay = 0.55f;


	while (window.isOpen())
	{
		Event e;
		if (window.pollEvent(e))
		{
			if (e.type == Event::Closed)
				window.close();
		}
		if (clock.getElapsedTime().asSeconds() < frameDelay)
			continue;

		clock.restart();

		window.clear(Color::Black);
		window.draw(backgroundSprite);
		if (!loadingDone)
			DrawBuildings(window, c, NumberOfPatches);
		if (!spaceBarPressed)
			window.draw(text);


		if (!loadingDone && spaceBarPressed)
		{
			window.draw(loadingText);
			window.draw(backgroundBar);
			progress += 100; // Simulate loading progress
			if (progress >= 400)
			{
				progress = 400;
				loadingDone = true;
			}
			progressBar.setSize(sf::Vector2f(progress, 30));
			window.draw(progressBar);

		}
		else if (loadingDone && spaceBarPressed)
		{
			highlightPatch(window, c);
		}
		if (e.KeyPressed)
		{
			if (e.key.code == Keyboard::Space)
			{
				ismoveDrone = true;
				spaceBarPressed = true;
				e.key.code = Keyboard::Unknown;
			}
		}



		// Map Generation
		if (e.MouseButtonPressed)
		{
			if (e.mouseButton.button == Mouse::Left)
			{
				int mouseX = e.mouseButton.x;
				int mouseY = e.mouseButton.y;
				//cout << "Mouse Position: " << x << ", " << y << endl;
				for (int i = 0; i < NumberOfPatches; i++)
				{
					if (c[i].containsPoint(mouseX, mouseY)) {
						if (c[i].checkClear()) {
							cout << "Clicked Patch: row = " << c[i].row + 1 << ", col = " << c[i].col + 1 << ", index = " << i << endl;
							terrain[c[i].row][c[i].col] = '#';
							grid[c[i].row][c[i].col].weight = 5;

							c[i].isBuilding = true;
							c[i].buildingNo = rand() % NumberofBuildings + 1; // Randomly assign a building number
						}
					}
				}
			}
		}

		//DrawPatches(window, c, NumberOfPatches);
		if (loadingDone)
			DrawBuildings(window, c, NumberOfPatches);

		for (int i = 0;i < number_of_drones;i++)
		{
			if (loadingDone && ismoveDrone && packagePicked[i])
			{
				moveDrone(temp[i], c, grid, patchPath[i], in[i], HousePosx[i], HousePosy[i], window, progress, i);
				in[i]++;
			}
			else if (loadingDone && ismoveDrone && !packagePicked[i])
			{
				moveDrone(temp[i], c, grid, patchPath[i], in[i], packagePosx[i], packagePosy[i], window, progress, i);
				in[i]++;
			}
		}
		bool check = false;
		for (int i = 0;i < number_of_drones;i++) {
			if (!countFinished[i]) {
				check = true;
			}
		}
		window.display();
		if (!check) {
			showPopupMessage("ALL PACKAGES DELIVERED!!!");
			return 0;
		}
	}
}