import graph.algorithms.Graph_Algo;
import graph.dataStructure.DGraph;
import graph.dataStructure.Point3D;
import graph.dataStructure.node;
import graph.dataStructure.node_data;

import javax.tools.Tool;
import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

public class AutoAlgo1 {
	
	int map_size = 3000;
	enum PixelState {blocked,explored,unexplored,visited};
	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;
	
	ArrayList<Point> points;
	
	
	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;
	
	boolean isSpeedUp = false;
	
	//Graph mGraph = new Graph();
	
	CPU ai_cpu;
	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func =  new ArrayList<>();
		points = new ArrayList<Point>();
		
		drone = new Drone(realMap);
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);

		
		initMap();
		
		isRotating = 0;
		ai_cpu = new CPU(200,"Auto_AI");
		ai_cpu.addFunction(this::update);
	}
	
	public void initMap() {
		map = new PixelState[map_size][map_size];
		for(int i=0;i<map_size;i++) {
			for(int j=0;j<map_size;j++) {
				map[i][j] = PixelState.unexplored;
			}
		}
		
		droneStartingPoint = new Point(map_size/2,map_size/2);
	}
	
	public void play() {
		drone.play();
		ai_cpu.play();
		grap= new Graph_Algo();
		}

	
	public void update(int deltaTime) {
		updateVisited();
		updateMapByLidars();
		
		ai(deltaTime);
		
		
		if(isRotating != 0) {
			updateRotating(deltaTime);
		}
		if(isSpeedUp) {
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
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		for(int i=0;i<drone.lidars.size();i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			//rotation = Drone.formatRotation(rotation);
			for(int distanceInCM=0;distanceInCM < lidar.current_distance;distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x,p.y,PixelState.explored);
			}
			
			if(lidar.current_distance > 0 && lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x,p.y,PixelState.blocked);
				//fineEdges((int)p.x,(int)p.y);
			}
		}
	}
	
	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		setPixel(fromPoint.x,fromPoint.y,PixelState.visited);
			
	}
	
	public void setPixel(double x, double y,PixelState state) {
		int xi = (int)x;
		int yi = (int)y;
		
		if(state == PixelState.visited) {
			map[xi][yi] = state; 
			return;
		}
		
		if(map[xi][yi] == PixelState.unexplored) {
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
		
		int i = (int)droneStartingPoint.y - (int)drone.startPoint.x;
		int startY = i;
		for(;i<map_size;i++) {
			int j = (int)droneStartingPoint.x - (int)drone.startPoint.y;
			int startX = j;
			for(;j<map_size;j++) {
				if(map[i][j] != PixelState.unexplored)  {
					if(map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					} 
					else if(map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					}
					else if(map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i-startY, j-startX, i-startY, j-startX);
				}
			}
		}
		g.setColor(c);
	}

	public void paintPoints(Graphics g) {
		for (int next : this.important_point.keySet()){
			node p = important_point.get(next);
//			System.out.println("important_point"+important_point.size());
//			System.out.println("ID NODE:"+next+"--left--"+important_point.get(next).getImportant_pointleft()+"--Rigt--"+important_point.get(next).getImportant_pointRigt());
//			g.setColor(Color.GREEN);
//			g.setColor(Color.red);
			g.drawOval((int) p.getLocation().x() + (int) drone.startPoint.x - 10, (int) p.getLocation().y() + (int) drone.startPoint.y - 10, 20, 20);
		}

	}
	public void paint(Graphics g) {
		if(SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);
		drone.paint(g);
	}
	String flag="no";
	String flag2="no";
	private node Last_suspicious_point =null;
	boolean is_init = true;
	double lastFrontLidarDis = 0;
	boolean isRotateRight = false;
	double Rightchanged = 0;
	double Leftchanged = 0;
	boolean tryToEscape = false;
	int leftOrRight = 1;
	private Graph_Algo grap= new Graph_Algo();
	private node	ran = null;
	private node	ran2=null;
	Iterator connectedNode;
	private HashMap<Integer,node> important_point = new HashMap<Integer,node>();
	double max_rotation_to_direction = 20;
	boolean  is_finish = true;
	boolean isLeftRightRotationEnable = true;
	private node_data ran_return_home= new node();
	private double time;
	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double  risky_dis = 0;
	int max_angle_risky = 10;
	
	boolean is_lidars_max = false;
	
	double save_point_after_seconds = 3;
	
	double max_distance_between_points = 100;
	
	boolean start_return_home = false;
	
	Point init_point;

	public void ai(int deltaTime) {
		Lidar lidar = drone.lidars.get(0);
		Lidar lidar1 = drone.lidars.get(1);
		Lidar lidar2 = drone.lidars.get(2);
		if (!SimulationWindow.toogleAI) {
			return;
		}

		if (is_init) {
			speedUp();
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			is_init = false;
			ran = new node(drone.startPoint.x, drone.startPoint.y,0, 0,0,0,System.currentTimeMillis());
			grap.getAlgoGraph().addNode(ran);
			Last_suspicious_point = ran;
		}

		if (isLeftRightRotationEnable) {
			//doLeftRight();
		}


		Point dronePoint = drone.getOpticalSensorLocation();


		if (SimulationWindow.return_home) {
			if (ran2 != null) {
				List<node_data> arr = grap.shortestPath(1, this.ran.getKey());
				connectedNode = arr.iterator();
//				System.out.println(arr.size());
				ran_return_home = (node_data) connectedNode.next();
				spinBy(ran.getWeight() - 180);
				this.time=System.currentTimeMillis();
			}
//			while (connectedNode.hasNext()){
			if (connectedNode.hasNext()&&isRotating!=1&&
					ran_return_home.getTimeold()<System.currentTimeMillis()-time) {
//				if (ran_return_home.getTimeold() <= System.currentTimeMillis()-time) {
					System.out.println("ID NEW NODE"+ran_return_home.getKey());
					ran_return_home = (node_data) connectedNode.next();
					spinBy(ran.getWeight());
					time=System.currentTimeMillis();
//				}
			}
				if (ran.getKey() == 1 || Tools.getDistanceBetweenPoints(drone.getOpticalSensorLocation(),drone.startPoint)<50)
				drone.stop();
//			}
//			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
//				if (points.size() <= 1 && Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points / 5) {
//					speedDown();
//				} else {
//					removeLastPoint();
//				}
//			}
		}
		else if (SimulationWindow.suspicious_point) {
			///////////////////////////////////
			if (!(ai_cpu.isPlay) && ran_return_home.getKey() != 1) {
				drone.play();
			}
			if (ran2 != null) {
				List<node_data> arr = grap.shortestPath(Last_suspicious_point.getKey(), ran.getKey());
				connectedNode = arr.iterator();
//				System.out.println(arr.size());
				ran_return_home = (node_data) connectedNode.next();
				while (isRotating == 1) {// imitate to open
				}
				spinBy(ran_return_home.getWeight() - 180);
			}
			if (connectedNode.hasNext() && isRotating != 1 &&
					ran_return_home.getTimeold() < System.currentTimeMillis() - time) {
//				if (ran_return_home.getTimeold() <= System.currentTimeMillis()-time) {
				System.out.println("ID NEW NODE" + ran_return_home.getKey());
				ran_return_home = (node_data) connectedNode.next();
				spinBy(ran_return_home.getWeight());
				time = System.currentTimeMillis();
			}
			if (ran_return_home.getKey() == Last_suspicious_point.getID() || Tools.getDistanceBetweenPoints(drone.getOpticalSensorLocation(), drone.startPoint) < 50)
				drone.stop();
		}

		/////////////////////////
		else {
			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
				points.add(dronePoint);
			}
		}



		if (!is_risky) {
//			Lidar lidar = drone.lidars.get(0);
			if (lidar.current_distance <= max_risky_distance) {
				is_risky = true;
				risky_dis = lidar.current_distance;

			}


//			Lidar lidar1 = drone.lidars.get(1);
			if (lidar1.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

//			Lidar lidar2 = drone.lidars.get(2);
			if (lidar2.current_distance <= max_risky_distance / 3) {
				is_risky = true;
			}

		} else {
			if (!try_to_escape) {
				try_to_escape = true;
//				Lidar lidar1 = drone.lidars.get(1);
				double a = lidar1.current_distance;

//				Lidar lidar2 = drone.lidars.get(2);
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
					} else {
						if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
							points.add(dronePoint);
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
				int finalSpin_by = spin_by;
				if (ran2 != null) {

					if ((ran.getRight() < lidar2.current_distance))
//							&& ((drone.startPoint.x-ran.getLocation().x())>(drone.startPoint.y-ran.getLocation().y())))
						flag = "Rigt";
					else
						flag = "no";
					 if ((ran.getLeft() < lidar1.current_distance))
//							 && ((drone.startPoint.x-ran.getLocation().x())>(drone.startPoint.y-ran.getLocation().y())))
						 flag2 = "Left";
					 else
						 flag2 = "no";

					checkimporNoed(finalSpin_by);
				}

				spinBy(spin_by, true, new Func() {
					@Override
					public void method() {
						try_to_escape = false;
						is_risky = false;
					}
				});


				if ((int) ran.getWeight() != finalSpin_by && System.currentTimeMillis()-ran.time>2222 && !SimulationWindow.return_home) {
					System.out.println("addNoed");
					addNoed(dronePoint, finalSpin_by, lidar, lidar1, lidar2,System.currentTimeMillis()-ran.time);
				}
			}
		}
}

	private void checkimporNoed(int finalSpin_by) {
		if (Math.abs(Last_suspicious_point.getLocation().distance2D(Last_suspicious_point.getLocation(), ran.getLocation())) > 35) {
			important_point.put(ran.getKey(), ran);
			Last_suspicious_point = ran;
		}
	}

	private void addNoed(Point dronePoint, int finalSpin_by, Lidar lidar, Lidar lidar1, Lidar lidar2,double v) {
		ran2 = new node(dronePoint.x,dronePoint.y, finalSpin_by, lidar.current_distance, lidar1.current_distance, lidar2.current_distance,v);
//		System.out.println("finalSpin_by"+finalSpin_by);
//		System.out.println("ran"+ran.toString());
//		System.out.println("ran2"+ran2.toString());
		grap.getAlgoGraph().addNode(ran2);
		grap.getAlgoGraph().connect(ran.getKey(), ran2.getKey(), Point3D.distance2D(ran.getLocation(), ran2.getLocation()));
			ran2.setImportant_pointRigt(flag.toString());
			flag = "no";
			ran2.setImportant_pointleft(flag2.toString());
			flag2 = "no";
		ran=ran2;
	}

	int counter = 0;
	
	public void doLeftRight() {
		if(is_finish) {
			leftOrRight *= -1;
			counter++;
			is_finish = false;
			
			spinBy(max_rotation_to_direction*leftOrRight,false,new Func() {
				@Override
				public void method() {
					is_finish = true;
				}
			});
		}
	}
	
	
	double lastGyroRotation = 0;
	public void updateRotating(int deltaTime) {
		
		if(degrees_left.size() == 0) {
			return;
		}
		
		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if(degrees_left_to_rotate > 0) {
			isLeft = false;
		}
		
		double curr =  drone.getGyroRotation();
		double just_rotated = 0;
		
		if(isLeft) {
			
			just_rotated = curr - lastGyroRotation;
			if(just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
		} else {
			just_rotated = curr - lastGyroRotation;
			if(just_rotated < 0) {
				just_rotated = 360 + just_rotated;
			}
		}
		
	
		 
		lastGyroRotation = curr;
		degrees_left_to_rotate-=just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0,degrees_left_to_rotate);
		
		if((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);
			
			Func func = degrees_left_func.get(0);
			if(func != null) {
				func.method();
			}
			degrees_left_func.remove(0);
			
			
			if(degrees_left.size() == 0) {
				isRotating = 0;
			}
			return; 
		}
		
		int direction = (int)(degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);
		
	}
	
	public void spinBy(double degrees,boolean isFirst,Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,func);
		
			
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}
		
		isRotating =1;
	}
	
	public void spinBy(double degrees,boolean isFirst) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,null);
		
			
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(null);
		}
		
		isRotating =1;
	}
	
	public void spinBy(double degrees) {
		lastGyroRotation = drone.getGyroRotation();
		
		degrees_left.add(degrees);
		degrees_left_func.add(null);
		isRotating = 1;
	}
	
	public Point getLastPoint() {
		if(points.size() == 0) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		return p1;
	}
	
	public Point removeLastPoint() {
		if(points.isEmpty()) {
			return init_point;
		}
		
		return points.remove(points.size()-1);
	}
	
	
	public Point getAvgLastPoint() {
		if(points.size() < 2) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		Point p2 = points.get(points.size()-2);
		return new Point((p1.x + p2.x) /2, (p1.y + p2.y) /2);
	}
	
/*
	public  Thread suspicious_point= new Thread() {
		public void run(){
			System.out.println(ai_cpu.isPlay);System.out.println("ggggg");
			if (ran2 != null) {
				List<node_data> arr = grap.shortestPath(Last_suspicious_point.getKey(), ran.getKey());
				connectedNodesuspicious = arr.iterator();
				System.out.println(arr.size());
				ran2 = null;
				ran_return_home = (node_data) connectedNodesuspicious.next();
				while (isRotating==1) {// imitate to open
				}
				spinBy(finalSpin_by - 180);
				time=System.currentTimeMillis()+300;
			}
			if (connectedNodesuspicious.hasNext()) {
				if (ran_return_home.getTimeold() <= System.currentTimeMillis()-time) {
					if (Tools.getDistanceBetweenPoints(ran_return_home.getLocation(),drone.getOpticalSensorLocation())<max_distance_between_points/5){
						System.out.println("ID NEW NODE"+ran_return_home.getKey());
						ran_return_home = (node_data) connectedNodesuspicious.next();
						System.out.println("getWeight"+ran.getWeight());
						while (isRotating==1) {// imitate to open
						}
						spinBy(ran.getWeight());
						time=System.currentTimeMillis();
					}
					if (ran_return_home.getKey() == 1)
						spinBy(ran.getWeight()-180);
					SimulationWindow.suspicious_point=!SimulationWindow.suspicious_point;
					important_point.remove(ran_return_home.getKey());

				}
			}
		}
	};
	 public  Thread returnHome = new Thread() {
		public void run(){
			System.out.println(ai_cpu.isPlay);System.out.println("ggggg");
			if (!(ai_cpu.isPlay) && ran_return_home.getKey() != 1) {drone.play();}
						if (ran2 != null) {
				List<node_data> arr = grap.shortestPath(1, ran.getKey());
				connectedNode = arr.iterator();
				System.out.println(arr.size());
				ran2 = null;
				ran_return_home = (node_data) connectedNode.next();
				while (isRotating==1) {// imitate to open
				}
				spinBy(finalSpin_by - 180);
				time=System.currentTimeMillis()+300;
			}
			if (connectedNode.hasNext()) {
				if (ran_return_home.getTimeold() <= System.currentTimeMillis()-time) {
				if (Tools.getDistanceBetweenPoints(ran_return_home.getLocation(),drone.getOpticalSensorLocation())<max_distance_between_points/5){
					System.out.println("ID NEW NODE"+ran_return_home.getKey());
					ran_return_home = (node_data) connectedNode.next();
					System.out.println("getWeight"+ran.getWeight());
                   while (isRotating==1) {// imitate to open
                   	 }
					spinBy(ran.getWeight());
					time=System.currentTimeMillis();
				}
			if (ran_return_home.getKey() == 1) drone.stop();}}
		}
	};
*/
}
