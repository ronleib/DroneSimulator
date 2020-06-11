import graph.algorithms.Graph_Algo;
import graph.dataStructure.node;
import graph.dataStructure.node_data;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.HashMap;

public class AutoAlgo1 {

	int map_size = 3000;

	enum PixelState {blocked, explored, unexplored, visited}

	;
	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;

	ArrayList<Point> points;


	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;

	boolean isSpeedUp = false;

	Graph_Algo grap = new Graph_Algo();

	CPU ai_cpu;

	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func = new ArrayList<>();
		points = new ArrayList<Point>();

		drone = new Drone(realMap);
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
		for (int i = 0; i < points.size(); i++) {
			Point p = points.get(i);
			g.drawOval((int) p.x + (int) drone.startPoint.x - 10, (int) p.y + (int) drone.startPoint.y - 10, 20, 20);
		}
	}

	public void paintPoints2(Graphics g) {
		for (int next : this.important_point.keySet()) {
			node_data p = important_point.get(next);
			g.setColor(Color.red);
			g.drawOval((int) p.getLocation().x() + (int) drone.startPoint.x - 10, (int) p.getLocation().y() + (int) drone.startPoint.y - 10, 20, 20);
			g.setColor(Color.DARK_GRAY);
		}
	}

	public void paint(Graphics g) {
		if (SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);
		paintPoints2(g);
		drone.paint(g);


	}

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


	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double risky_dis = 0;
	int max_angle_risky = 10;

	boolean is_lidars_max = false;

	double save_point_after_seconds = 3;

	double max_distance_between_points = 45;

	boolean start_return_home = false;
	public node_data ran = null;
	private node_data Last_suspicious_point = null;
	private node_data Last_suspicious_point2 = null;
	private node ran2 = null;
	private HashMap<Integer, node_data> important_point = new HashMap<Integer, node_data>();


	String flag = "no";
	String flag2 = "no";


	Point init_point;

	public void ai(int deltaTime) {
		if (!SimulationWindow.toogleAI) {
			return;
		}


		if (is_init) {
			speedUp();
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			points.add(dronePoint);
//			mGraph.addVertex(dronePoint);
			is_init = false;
		}

		if (isLeftRightRotationEnable) {
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
		} else if (SimulationWindow.Suspicious_Point) {
			if (Last_suspicious_point != null) {
				if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
					if (Tools.getDistanceBetweenPoints(Last_suspicious_point.getLocation(), dronePoint) < max_distance_between_points / 5) {
						important_point.remove(Last_suspicious_point.getKey());
						Last_suspicious_point = Last_suspicious_point2;
						SimulationWindow.Suspicious_Point = !SimulationWindow.Suspicious_Point;
					} else {
						removeLastPoint();
					}
				}
			}
		} else {
			if (ran == null) {
				ran = new node(0, 0, 0, 0, 0, 0, System.currentTimeMillis());
				grap.getAlgoGraph().addNode(ran);
			} else if (Tools.getDistanceBetweenPoints(ran.getLocation(), dronePoint) >= 30) {
				addNoed(dronePoint, ran.getSpin(), drone.lidars.get(0), drone.lidars.get(1), drone.lidars.get(2), System.currentTimeMillis() - ran.getTimeOfEdge());
			}
			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
				points.add(dronePoint);
			}
		}


		if (!is_risky) {
			Lidar lidar = drone.lidars.get(0);
			if (lidar.current_distance <= max_risky_distance) {
				is_risky = true;
				risky_dis = lidar.current_distance;

			}


			Lidar lidar1 = drone.lidars.get(1);
			if (lidar1.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

			Lidar lidar2 = drone.lidars.get(2);
			if (lidar2.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

		} else {
			if (!try_to_escape) {
				try_to_escape = true;
				Lidar lidar1 = drone.lidars.get(1);
				double a = lidar1.current_distance;

				Lidar lidar2 = drone.lidars.get(2);
				double b = lidar2.current_distance;


				int spin_by = max_angle_risky;


				if (a > 270 && b > 270) {
					is_lidars_max = true;
					Point l1 = Tools.getPointByDistance(dronePoint, lidar1.degrees + drone.getGyroRotation(), lidar1.current_distance);
					Point l2 = Tools.getPointByDistance(dronePoint, lidar2.degrees + drone.getGyroRotation(), lidar2.current_distance);
					Point last_point = getAvgLastPoint();
					double dis_to_lidar1 = Tools.getDistanceBetweenPoints(last_point, l1);
					double dis_to_lidar2 = Tools.getDistanceBetweenPoints(last_point, l2);

					if (SimulationWindow.return_home) {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
							removeLastPoint();
						}
					} else if (SimulationWindow.Suspicious_Point) {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
							removeLastPoint();
						}
					} else {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
							points.add(dronePoint);
							if (Tools.getDistanceBetweenPoints(ran.getLocation(), dronePoint) >= max_distance_between_points) {
								if (ran == null) {
									ran = new node(0, 0, 0, 0, 0, 0, System.currentTimeMillis());
									grap.getAlgoGraph().addNode(ran);
								} else
									addNoed(dronePoint, ran.getSpin(), drone.lidars.get(0), drone.lidars.get(1), drone.lidars.get(2), System.currentTimeMillis() - ran.getTimeOfEdge());
							}
						}
					}

					spin_by = 90;

					if (SimulationWindow.return_home) {
						spin_by *= -1;
					}


					if (dis_to_lidar1 < dis_to_lidar2) {

						spin_by *= (-1);
					}
				} else {


					if (a < b) {
						spin_by *= (-1);
					}
				}
				ran.setSpin(spin_by);
				spinBy(spin_by, true, new Func() {
					@Override
					public void method() {
						try_to_escape = false;
						is_risky = false;
					}
				});

				if (ran2 != null) {
					if (Math.abs(ran.getRight() - lidar2.current_distance) > 250 &&
							ran.getRight() != 0 && lidar2.current_distance != 0 &&
							lidar2.current_distance > 300) {
						flag = "Rigt";
					} else {
						flag = "no";
					}
					if (Math.abs(ran.getLeft() - lidar1.current_distance) > 250
							&& ran.getLeft() != 0 && lidar1.current_distance != 0 &&
							lidar1.current_distance > 300) {
						flag2 = "Left";
					} else {
						flag2 = "no";
					}
					checkimporNoed();
				}
			}
		}


		if (SimulationWindow.initGraph) {
			grap.init("Grap-skimmer");

//		grap.save("Grap-skimmer");
//		Graph_Algo algo2 = new Graph_Algo();
//		System.out.println(algo2.toString());
//			grap=new Graph_Algo(file_name);

		}

		if (SimulationWindow._SaveGraph) {
			grap.save("Grap-skimmer");
		}
	}


	int counter = 0;

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

	private void checkimporNoed() {
		if (Last_suspicious_point == null) {
			Last_suspicious_point = ran;
			Last_suspicious_point2 = Last_suspicious_point;
		}
		if (Math.abs(Last_suspicious_point.getLocation().distance2D(Last_suspicious_point.getLocation(), ran.getLocation())) > 50) {
			important_point.put(ran.getKey(), ran);
			if (Last_suspicious_point.getKey() != Last_suspicious_point2.getKey()) {
				Last_suspicious_point2 = Last_suspicious_point;
			}
			Last_suspicious_point = ran;
		}
	}

	public synchronized void addNoed(Point dronePoint, int finalSpin_by, Lidar lidar, Lidar lidar1, Lidar lidar2, double v) {
		ran2 = new node(dronePoint.x, dronePoint.y, finalSpin_by, lidar.current_distance, lidar1.current_distance, lidar2.current_distance, v);
		System.out.println("finalSpin_by" + finalSpin_by);
		System.out.println("ran" + ran.toString());
		System.out.println("ran2" + ran2.toString());
		grap.getAlgoGraph().addNode(ran2);
		if (ran.getKey() != 0)
			grap.getAlgoGraph().connect(ran.getKey(), ran2.getKey(), Tools.getDistanceBetweenPoints(ran.getLocation(), ran2.getLocation()));
		ran2.setImportant_pointRigt(flag.toString());
		flag = "no";
		ran2.setImportant_pointleft(flag2.toString());
		flag2 = "no";
		System.out.println("ran" + ran.toString());
		System.out.println("ran2" + ran2.toString());
		ran = ran2;
	}

/*
	public Thread suspicious_point = new Thread() {
		public void run() {
//			System.out.println(ai_cpu.isPlay);
//			System.out.println("ggggg");
//			if (ran2 != null) {
//				List<node_data> arr = grap.shortestPath(Last_suspicious_point.getKey(), ran.getKey());
//				connectedNodesuspicious = arr.iterator();
//				System.out.println(arr.size());
//				ran2 = null;
				ran_return_home = (node_data) connectedNodesuspicious.next();
				while (isRotating == 1) {// imitate to open
				}
				spinBy(finalSpin_by - 180);
				time = System.currentTimeMillis() + 300;
			}
			if (connectedNodesuspicious.hasNext()) {
				if (ran_return_home.getTimeold() <= System.currentTimeMillis() - time) {
					if (Tools.getDistanceBetweenPoints(ran_return_home.getLocation(), drone.getOpticalSensorLocation()) < max_distance_between_points / 5) {
						System.out.println("ID NEW NODE" + ran_return_home.getKey());
						ran_return_home = (node_data) connectedNodesuspicious.next();
						System.out.println("getWeight" + ran.getWeight());
						while (isRotating == 1) {// imitate to open
						}
						spinBy(ran.getWeight());
						time = System.currentTimeMillis();
					}
					if (ran_return_home.getKey() == 1)
						spinBy(ran.getWeight() - 180);
					SimulationWindow.suspicious_point = !SimulationWindow.suspicious_point;
					important_point.remove(ran_return_home.getKey());

				}
			}
		}
	};
	public Thread returnHome = new Thread() {
		public void run() {
			System.out.println(ai_cpu.isPlay);
			System.out.println("ggggg");
			if (!(ai_cpu.isPlay) && ran_return_home.getKey() != 1) {
				drone.play();
			}
			if (ran2 != null) {
				List<node_data> arr = grap.shortestPath(1, ran.getKey());
				connectedNode = arr.iterator();
				System.out.println(arr.size());
				ran2 = null;
				ran_return_home = (node_data) connectedNode.next();
				while (isRotating == 1) {// imitate to open
				}
				spinBy(finalSpin_by - 180);
				time = System.currentTimeMillis() + 300;
			}
			if (connectedNode.hasNext()) {
				if (ran_return_home.getTimeold() <= System.currentTimeMillis() - time) {
					if (Tools.getDistanceBetweenPoints(ran_return_home.getLocation(), drone.getOpticalSensorLocation()) < max_distance_between_points / 5) {
						System.out.println("ID NEW NODE" + ran_return_home.getKey());
						ran_return_home = (node_data) connectedNode.next();
						System.out.println("getWeight" + ran.getWeight());
						while (isRotating == 1) {// imitate to open
						}
						spinBy(ran.getWeight());
						time = System.currentTimeMillis();
					}
					if (ran_return_home.getKey() == 1) drone.stop();
				}
			}
		}
	};
*/

	double lastGyroRotation = 0;

	public void updateRotating(int deltaTime) {

		if (degrees_left.size() == 0) {
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
}
