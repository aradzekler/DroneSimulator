import java.awt.*;
import java.util.ArrayList;

public class AutoAlgo1 {

	int map_size = 3000;
	PixelState[][] map;

	Drone drone;
	Point droneStartingPoint;
	ArrayList<Point> points;
	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;
	boolean isSpeedUp = false;
	Graph mGraph = new Graph();
	CPU ai_cpu;
	boolean is_init = true;
	double lastFrontLidarDis = 0;
	boolean isRotateRight = false;
	double changedRight = 0;
	double changedLeft = 0;
	boolean tryToEscape = false;
	int leftOrRight = 1;
	double max_rotation_to_direction = 20;
	boolean is_finish = true;
	boolean isLeftRightRotationEnable = true;
	/*
	
	public void fineEdges(int x,int y) {
		int radius = 6;
		
		for(int i=y-radius;i<y+radius;i++) {
			for(int j=x-radius;j<x+radius;j++) {
				if(Math.abs(y-i) <= 1 && Math.abs(x-j) <= 1) {
					continue;
				}
				if(map[i][j] == PixelState.blocked) {
					blockLine(x,y,j,i);
				}
			}
		}
	}
	*/
	/*
	public void blockLine(int x0,int y0,int x1,int y1) {
		if(x0 > x1) {
			int tempX = x0;
			int tempY = y0;
			x0 = x1;
			y0 = y1;
			x1 = tempX;
			y1 = tempY;
		}
		
	     double deltax = x1 - x0;
	     double deltay = y1 - y0;
	     double deltaerr = Math.abs(deltay / deltax);    // Assume deltax != 0 (line is not vertical),
	     double error = 0.0; // No error at start
	     int y = y0;
	     for (int x=x0;x<x1;x++) {
	    	 setPixel(x,y,PixelState.blocked);
	         error = error + deltaerr;
	         if( 2*error >= deltax ) {
                y = y + 1;
                error=error - deltax;
	        }
	     }
	
	}
	*/
	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double risky_dis = 0;
	int max_angle_risky = 10;
	boolean is_lidars_max = false;
	double save_point_after_seconds = 3;
	double max_distance_between_points = 100;
	boolean start_return_home = false;
	Point init_point;
	int counter = 0;
	double lastGyroRotation = 0;

	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func = new ArrayList<>();
		points = new ArrayList<>();

		drone = new Drone(realMap);
		// adding sensors (straight, right, left)
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);


		initMap();

		isRotating = 0;
		ai_cpu = new CPU(200, "Auto_AI");
		ai_cpu.addFunction(this::update);
	}

	public void initMap() {
		map = new PixelState[map_size][map_size];
		for (int i = 0; i < map_size; i++) {
			for (int j = 0; j < map_size; j++) {
				map[i][j] = PixelState.unexplored;
			}
		}

		droneStartingPoint = new Point(map_size / 2, map_size / 2);
	}

	public void play() {
		drone.play();
		ai_cpu.play();
	}

	public void update(int deltaTime) {
		updateVisited();
		updateMapByLidars();

		ai(deltaTime);


		if (isRotating != 0) {
			updateRotating(deltaTime);
		}
		if (isSpeedUp) {
			drone.speedUp(deltaTime);
		} else {
			drone.slowDown(deltaTime);
		}

	}

	public void speedUp() {
		isSpeedUp = true;
	}

	public void speedDown() {
		isSpeedUp = false;
	}

	public void updateMapByLidars() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);

		for (int i = 0; i < drone.lidars.size(); i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			//rotation = Drone.formatRotation(rotation);
			for (int distanceInCM = 0; distanceInCM < lidar.current_distance; distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x, p.y, PixelState.explored);
			}

			if (lidar.current_distance > 0 && lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x, p.y, PixelState.blocked);
				//fineEdges((int)p.x,(int)p.y);
			}
		}
	}

	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);

		setPixel(fromPoint.x, fromPoint.y, PixelState.visited);

	}

	public void setPixel(double x, double y, PixelState state) {
		int xi = (int) x;
		int yi = (int) y;

		if (state == PixelState.visited) {
			map[xi][yi] = state;
			return;
		}

		if (map[xi][yi] == PixelState.unexplored) {
			map[xi][yi] = state;
		}
	}

	/*
	 * Function for painting parts of the map.
	 */
	public void paintBlindMap(Graphics g) {
		Color c = g.getColor();

		int i = (int) droneStartingPoint.y - (int) drone.startPoint.x;
		int startY = i;
		for (; i < map_size; i++) {
			int j = (int) droneStartingPoint.x - (int) drone.startPoint.y;
			int startX = j;
			for (; j < map_size; j++) {
				if (map[i][j] != PixelState.unexplored) {
					if (map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					} else if (map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					} else if (map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i - startY, j - startX, i - startY, j - startX);
				}
			}
		}
		g.setColor(c);
	}

	public void paintPoints(Graphics g) {
		for (Point p : points) {
			g.drawOval((int) p.x + (int) drone.startPoint.x - 10, (int) p.y + (int) drone.startPoint.y - 10, 20, 20);
		}

	}
	// map painting function
	public void paint(Graphics g) {
		if (SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);

		drone.paint(g);


	}

	/*
		the main AI function.
	 */
	public void ai(int deltaTime) {
		if (!SimulationWindow.toogleAI) {
			return;
		}

		// if drone is started
		if (is_init) {
			speedUp(); // accelerate to a constant speed
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			points.add(dronePoint); // and start to go (maybe to lidar0 pos?)
			mGraph.addVertex(dronePoint);
			is_init = false;
		}

		if (isLeftRightRotationEnable) { // the leftright manuver try.. not implented yet.
			//doLeftRight();
		}


		Point dronePoint = drone.getOpticalSensorLocation();


		if (SimulationWindow.return_home) {

			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
				if (points.size() <= 1 && Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points / 5) {
					speedDown();
				} else {
					removeLastPoint();
				}
			}
		} else {
			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
				points.add(dronePoint);
				mGraph.addVertex(dronePoint);
			}
		}


		if (!is_risky) { // if we are not in a risky situation, check if the distance to wall is smaller then risky.
			Lidar lidar = drone.lidars.get(0);
			if (lidar.current_distance <= max_risky_distance) {
				is_risky = true;
				risky_dis = lidar.current_distance; // if so, turn on risky distance.

			}

			// if we are in risky, check other sensors (left and right presumbly, for 1/3 of risky distance
			Lidar lidar1 = drone.lidars.get(1);
			if (lidar1.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

			Lidar lidar2 = drone.lidars.get(2);
			if (lidar2.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

		} else { // if we are already in risky. try to escape.
			if (!try_to_escape) { // and not initiated escape sequence.
				try_to_escape = true;
				Lidar lidar1 = drone.lidars.get(1);
				double a = lidar1.current_distance;

				Lidar lidar2 = drone.lidars.get(2);
				double b = lidar2.current_distance;


				int spin_by = max_angle_risky;

				// if a and b (our left and right sensors) are  in more then 270cm from object..
				if (a > 270 && b > 270) {
					is_lidars_max = true;
					// we turn around, and go to our last location.
					Point l1 = Tools.getPointByDistance(dronePoint, lidar1.degrees + drone.getGyroRotation(), lidar1.current_distance);
					Point l2 = Tools.getPointByDistance(dronePoint, lidar2.degrees + drone.getGyroRotation(), lidar2.current_distance);
					Point last_point = getAvgLastPoint();
					double dis_to_lidar1 = Tools.getDistanceBetweenPoints(last_point, l1);
					double dis_to_lidar2 = Tools.getDistanceBetweenPoints(last_point, l2);

					if (SimulationWindow.return_home) {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
							removeLastPoint();
						}
					} else {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
							points.add(dronePoint);
							mGraph.addVertex(dronePoint);
						}
					}

					spin_by = 90;
					if (SimulationWindow.return_home) {
						spin_by *= -1;
					}


					if (dis_to_lidar1 < dis_to_lidar2) { // if distance of last spot from left wall larger then last spot of right wall, spin left

						spin_by *= (-1);
					}
				} else { // if there is no need for risky (not close to a wall..)
					Lidar lidar0 = drone.lidars.get(0);
					double lidar0Dist = lidar0.current_distance;
					// CHANGE: added - if lidar0 (farward is beyond range, and far from colliding, continue that path.)
					//TODO: need to add what happens if he gets into a tzomet.
					//TODO: maybe add a conditional that if lidar1 for example is seeing something not painted, then turn.
					//TODO: maybe just check if one of the liadrs is maxed and other is not, if yes then turn.
					//TODO: maybe do a dynamic speed.
					if (lidar0Dist >= WorldParams.lidarLimit - 10 && a > 70 && b > 70) {
						a = 0;
						b = 0;
					}
					else if (a < b) { // if distance from left wall larger then right wall, spin left
						spin_by *= (-1);
					}
				}

				// getting out of risky mode
				spinBy(spin_by, true, new Func() {
					@Override
					public void method() {
						try_to_escape = false;
						is_risky = false;
					}
				});
			}
		}

	}

	// a try to do a left-right manuver?
	public void doLeftRight() {
		if (is_finish) {
			leftOrRight *= -1;
			counter++;
			is_finish = false;

			spinBy(max_rotation_to_direction * leftOrRight, false, new Func() {
				@Override
				public void method() {
					is_finish = true;
				}
			});
		}
	}

	// the main function for updating rotation. lidars get input from buttons or riskymode and updates the rotation
	public void updateRotating(int deltaTime) {

		if (degrees_left.size() == 0) { // if no rotation is needed, do nothing.
			return;
		}

		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if (degrees_left_to_rotate > 0) {
			isLeft = false;
		}

		double curr = drone.getGyroRotation();
		double just_rotated = 0;

		if (isLeft) {

			just_rotated = curr - lastGyroRotation;
			if (just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
		} else {
			just_rotated = curr - lastGyroRotation;
			if (just_rotated < 0) {
				just_rotated = 360 + just_rotated;
			}
		}


		lastGyroRotation = curr;
		degrees_left_to_rotate -= just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0, degrees_left_to_rotate);

		if ((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);

			Func func = degrees_left_func.get(0);
			if (func != null) {
				func.method();
			}
			degrees_left_func.remove(0);


			if (degrees_left.size() == 0) {
				isRotating = 0;
			}
			return;
		}

		int direction = (int) (degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);

	}

	public void spinBy(double degrees, boolean isFirst, Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if (isFirst) {
			degrees_left.add(0, degrees);
			degrees_left_func.add(0, func);


		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}

		isRotating = 1;
	}

	public void spinBy(double degrees, boolean isFirst) {
		lastGyroRotation = drone.getGyroRotation();
		if (isFirst) {
			degrees_left.add(0, degrees);
			degrees_left_func.add(0, null);


		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(null);
		}

		isRotating = 1;
	}

	public void spinBy(double degrees) {
		lastGyroRotation = drone.getGyroRotation();

		degrees_left.add(degrees);
		degrees_left_func.add(null);
		isRotating = 1;
	}

	public Point getLastPoint() {
		if (points.size() == 0) {
			return init_point;
		}

		Point p1 = points.get(points.size() - 1);
		return p1;
	}

	public Point removeLastPoint() {
		if (points.isEmpty()) {
			return init_point;
		}

		return points.remove(points.size() - 1);
	}

	public Point getAvgLastPoint() {
		if (points.size() < 2) {
			return init_point;
		}

		Point p1 = points.get(points.size() - 1);
		Point p2 = points.get(points.size() - 2);
		return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	}


	enum PixelState {blocked, explored, unexplored, visited}


}
