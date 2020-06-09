package graph.algorithms;

import graph.dataStructure.*;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

import static graph.dataStructure.node.nodeComparator;


/**
 * This  class represents the set of graph-theory algorithms
 * that has a filed of a directional weighted graph.
 * The class has a road-system or communication network in mind - and should support a large number of nodes (over 1,000,000).
 * The implementation is based a TSP algorithm , dijkstra algorithm and CSV reading method to read from files  .
 * @author  Simon Pikalov and Ron Leib
 *
 */

public class Graph_Algo implements graph_algorithms {


	DGraph algoGraph;


	/**
	 * an empty constracto that init the graph (Dgraph ) as an empty graph
	 */
	public Graph_Algo() {
		this.algoGraph = new DGraph();
	}
	public void Graph_Algo() {
		this.algoGraph = new DGraph();
	}

	/**
	 * Init this set of algorithms on the parameter - graph.
	 * @param graph
	 */
	public Graph_Algo(graph graph) {
		if (graph instanceof DGraph && !(graph.equals(null))) {
			algoGraph = (DGraph) graph;
		}
	}

	/**
	 * Init this set of algorithms on the parameter - graph.
	 * @param g
	 */
	@Override
	public void init(graph g) {
		if (g instanceof DGraph && !(g.equals(null))) {
			algoGraph = (DGraph) g;
		}
	}

	public void initJson(String graphString) {
		int nodeID = 0, edgesStart = 0, edgesEnd = 0;
		double weight = 0;
		String point = "";
		try {
			this.Graph_Algo();
			JSONObject graph = new JSONObject(graphString);
			JSONArray nodes = graph.getJSONArray("Nodes");
			JSONArray edges = graph.getJSONArray("Edges");

			for (int i = 0; i < nodes.length(); i++) {
				nodeID = nodes.getJSONObject(i).getInt("id");
				point = nodes.getJSONObject(i).getString("pos");
				Point3D p = new Point3D(point);
				node temp = new node(p.x(), p.y());
				temp.setID(nodeID);
				this.algoGraph.getNodeMap().put(nodeID, temp);
			}

			for (int i = 0; i < edges.length(); i++) {
				edgesStart = edges.getJSONObject(i).getInt("src");
				edgesEnd = edges.getJSONObject(i).getInt("dest");
				weight = edges.getJSONObject(i).getDouble("w");
				this.algoGraph.connect(edgesStart, edgesEnd, weight);
			}
		} catch (Exception exception) {
			///////////
		}
	}

	public String saveJSON() {
		String point, nodeID, start, end, weight;
		JSONObject grap = new JSONObject();
		JSONArray edgeArray = new JSONArray();
		JSONArray nodeArray = new JSONArray();
		try {
			for (int x : this.algoGraph.getNodeMap().keySet()) {
				point = this.algoGraph.getNodeMap().get(x).getLocation().toString();
				nodeID = "" + x;
				JSONObject node = new JSONObject();
				node.put("ID", nodeID);
				node.put("point", point);
				nodeArray.put(node);
			}
			for (int x : this.algoGraph.getNeighbore().keySet()) {
				JSONObject edge = new JSONObject(); //start,end,weight;
				start = "" + x;
				for (int y : this.algoGraph.getNeighbore().get(x).keySet()) {
					end = "" + y;
					weight = "" + this.algoGraph.getNeighbore().get(x).get(y).getWeight();
					edge.put("start", start);
					edge.put("end", end);
					edge.put("weight", weight);
					edgeArray.put(edge);
				}
			}
			grap.put("Node", nodeArray);
			grap.put("Edge", edgeArray);
		} catch (JSONException e) {
			e.printStackTrace();
		}

		return grap.toString();
	}

		/**
	 * Init a graph from file of CSV type
	 * Builds it GRAPH order:"NODE-0"->"EDGE-1"->"MC-2"->"startKey-3"
	 * By the command number: NODE=0 ,EDGE=1,MC=0,startKey=3
	 * look for the Example File in the Ducumantation for the format
	 * @param file_name
	 */
	@Override
	public void init(String file_name) {
		node.setNextFreeID(0);
		File file=new File(file_name);
		DGraph graphCSV= new DGraph();

		String ferstdata="",data="";
		int flag=0,indexX,indexY=0,mc=0,startKey=0;
		double indexW=0;

		try {
			Scanner inputStream=new Scanner(file);
			inputStream.next();
			while (inputStream.hasNext()) {
				data = inputStream.next();
				String[] values = data.split("-");
				if (values.length == 2) {
					flag = Integer.parseInt(values[1]);
					data = inputStream.next();
					values = data.split("-");
				}
				switch (flag) {
					case 0:
						indexX = Integer.parseInt(values[0]);
						indexY = Integer.parseInt(values[1]);
						indexW = Double.parseDouble(values[2]);
						node temp = new node(indexX,indexY, indexW);
						graphCSV.addNode(temp);
						break;
					case 1:
						indexX = Integer.parseInt(values[0]);
						indexY = Integer.parseInt(values[1]);
						indexW = Double.parseDouble(values[2]);
						graphCSV.connect(indexX, indexY, indexW);
						break;
					case 2:
						graphCSV.setMc(Integer.parseInt(values[0]));
						break;
					case 3:
						graphCSV.setEdgeNumber(Integer.parseInt(values[0]));
						break;
				}
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		this.init(graphCSV);
	}

	/**
	 * Creating a new Format CSV
	 * Saves the graph to a file.
	 * Builds it in order:"NODE-0"->"EDGE-1"->"MC-2"->"startKey-3"
	 * And gives a command number NODE=0 ,EDGE=1,MC=0,startKey=3
	 * @param file_name The path in wich the graph goint to be saved
	 */
	@Override
	public void save(String file_name) {
		String indexNode=file_name;
		try {
			PrintWriter pw= new PrintWriter(new File(file_name));
			StringBuilder sb=new StringBuilder();

			sb.append("NODE-0");
			sb.append("\r\n");
			for (int x : algoGraph.getNodeMap().keySet()){
				indexNode=(algoGraph.getNodeMap().get(x).getLocation().x()+"-"+
						algoGraph.getNodeMap().get(x).getLocation().y()+"-"+
						algoGraph.getNodeMap().get(x).getWeight());
				sb.append(indexNode);
				sb.append("\r\n");}

			sb.append("EDGE-1");
			sb.append("\r\n");
			for (int x : algoGraph.getNeighbore().keySet()){
				for (int y : algoGraph.getNeighbore().get(x).keySet()){
					indexNode=x+"-"+y+"-"+algoGraph.getNeighbore().get(x).get(y).getWeight();
					sb.append(indexNode);
					sb.append("\r\n");}
			}

			sb.append("MC-2");
			sb.append("\r\n");
			indexNode=""+algoGraph.getMC();
			sb.append(indexNode);
			sb.append("\r\n");

			sb.append("startKey-3");
			sb.append("\r\n");
			indexNode=""+algoGraph.edgeSize();
			sb.append(indexNode);
			sb.append("\r\n");

			pw.write(sb.toString());
			pw.close();
			System.out.println("finished");


		} catch (Exception e) {
			// TODO: handle exception
		}

	}


	/**
	 *Happens to the function :shortestPath
	 * Now all vertices have a checker value that fear is not equal to Double.MAX_VALUE
	 * Then the graph is isConnected
	 * @return  true if and only if (iff) there is a valid path from EVREY node to each
	 *  * other node. : because  it's directional graph - a valid path (a-->b) does NOT imply a valid path (b-->a) ! .
	 */
	@Override
	public boolean isConnected() {  // check from rand soure the turn it back then check again
		if (this.algoGraph.equals(null))throw new RuntimeException("Graph is not Exists ");
		if(this.algoGraph.nodeSize()<2){return true;}
		this.algoGraph.getNeighbore().keySet();
		int one=0,two=0;
		for (int x : this.algoGraph.getNeighbore().keySet()) {
			if (two==0){
				two=x;}
			if (two!=0&&two!=x){
				one=x;
				break;}
		}
		List<node_data> connected=this.shortestPath(one,two);
		Iterator connectedNode=connected.iterator();
		if (!testNode(connected)) return false;
		DGraph upsideGraph = new DGraph(this.algoGraph.getNodeMap(), Changedirection(this.algoGraph.getNeighbore()));
		Graph_Algo text = new Graph_Algo();
		text.init(upsideGraph);
		return text.testNode(text.shortestPath(one,two));
	}
	private boolean testNode(List<node_data> connected) {
		for (int x : algoGraph.getNodeMap().keySet()) {
			if (algoGraph.getNodeMap().get(x).getWeight()==Integer.MAX_VALUE)
				return false;
		}
		return true;
	}

	/**
	 * A developer Function to Flip all the side's of the current graph
	 * (a-->b) will be changet to  (b-->a)
	 * @param neighbore the neighbore Hasmap
	 * @return the changed Map
	 */
	private HashMap<Integer, HashMap<Integer,edge_data>> Changedirection(HashMap<Integer, HashMap<Integer, edge_data>> neighbore) {
		HashMap<Integer, HashMap<Integer, edge_data>> changneEighbore= new HashMap<Integer, HashMap<Integer, edge_data>>() ;
		for (int x : neighbore.keySet()) {
			for (int y : neighbore.get(x).keySet()){
				if (changneEighbore.containsKey(y))
					changneEighbore.get(y).putAll(neighbore.get(x));
				else changneEighbore.put(y,neighbore.get(x));
			}
		}
		return changneEighbore;
	}
	/**
	 * Function to find the short'st path using dijkstra algorithm and a priority Queue (by calling anothe function)
	 * @param src - start node
	 * @param dest - end (target) node
	 * @return  the length (in Wight not in Nodes )of the shortest path between src to dest
	 */
	@Override
	public double shortestPathDist(int src, int dest) {
		 List<node_data> ranDist=shortestPath( src,  dest);
		return this.algoGraph.getNodeMap().get(dest).getWeight(); // we add the path from srart to first node because start is not part of the list
	}


	/**
	 * Function to find the short'st path using dijkstra algorithm and a priority Queue (by calling anothe function)
	 * @param src - start node
	 * @param dest - end (target) node
	 *   @param reset - if user want's to reset the gtaph by dijastra
	 * @return  the length (in Wight not in Nodes )of the shortest path between src to dest
	 */

	public double shortestPathDist(int src, int dest,boolean reset) {
		if(src==dest) return  0;
		if(reset)  { List<node_data> ranDist=shortestPath( src,  dest);}
		return this.algoGraph.getNodeMap().get(dest).getWeight(); // we add the path from srart to first node because start is not part of the list
	}




	/**
	 * Function to find the short'st path using dijkstra algorithm and a priority Queue
	 * if src and dest is the same  the  function will return  null
	 * src--> n1-->n2-->...dest
	 * @param src - start node
	 * @param dest - end (target) node
	 * @return  the the shortest path between src to dest - as an ordered List of nodes:
	 */
	@Override
	public List<node_data> shortestPath(int src, int dest) {

		if(src==dest) return null;

		if(!(algoGraph.getNodeMap().containsKey(src)&&algoGraph.getNodeMap().containsKey(dest))) {
			throw new RuntimeException("src  or dst is not in the graph");
		}

		PriorityQueue<node_data> nodePriorityQueue = new PriorityQueue<node_data>(nodeComparator); //priority heap
		node_data start = (node)algoGraph.getNodeMap().get(src); // the node of the staty

		for (int x : algoGraph.getNodeMap().keySet()) { // for over all the nodes at the graph and set all the
			algoGraph.getNodeMap().get(x).setWeight(Integer.MAX_VALUE);
		}
		start.setWeight(0);// set the weight of the start node as 0
		nodePriorityQueue.add(start);
		while (!(nodePriorityQueue.isEmpty())) {
		node currNode = (node)nodePriorityQueue.remove();
		if(!algoGraph.getNeighbore().containsKey(currNode.getKey())) continue; // if this node is not connected
		for (int next : algoGraph.getNeighbore().get(currNode.getKey()).keySet()) { // loop all over currnode neigbors
			edge nextEdge = (edge)algoGraph.getNeighbore().get(currNode.getKey()).get(next); // the edge between currnt node and it's neighbore
			node nextNode = (node)this.algoGraph.getNodeMap().get(nextEdge.getDest());
			if((currNode.getWeight()+nextEdge.getWeight())<(nextNode.getWeight())){ //if there is a cheaper path
				nextNode.setWeight((currNode.getWeight()+nextEdge.getWeight())); // update the weight
				nextNode.setHisoty(currNode);
				nodePriorityQueue.add(nextNode);
			}
		}


		}

		Queue<node_data> toReturn = new LinkedList<>();
		node prev =(node) algoGraph.getNodeMap().get(dest);
		toReturn.add(prev);
		if(prev.getHisoty().getKey()==src) {
			return (LinkedList<node_data>) toReturn; // the case it's one node path
		}
		while(prev.getHisoty().getKey()!=src) {
			if (prev.getHisoty().getKey() == Integer.MAX_VALUE) {
				throw new RuntimeException("There is no Vaild path");
			} else {
				toReturn.add(prev.getHisoty());
				prev = prev.getHisoty();
			}
		}
//		Collections.reverse((List<node_data>) toReturn); // flip the order of the list
		return (LinkedList<node_data>) toReturn;

}

	@Override
	public graph copy() {
		HashMap nodeMapCopyGraph = new HashMap<Integer, node>();
		HashMap<Integer, HashMap<Integer, edge_data>> neighboreCopyGraph = new HashMap<Integer, HashMap<Integer, edge_data>>();
		for (int x : algoGraph.getNodeMap().keySet()) {
			node copyNode=new node(algoGraph.getNodeMap().get(x).getLocation().x(),algoGraph.getNodeMap().get(x).getLocation().y());
			nodeMapCopyGraph.put(x,copyNode);
		}
		for (int x : algoGraph.getNeighbore().keySet()){
			for (int y : algoGraph.getNeighbore().get(x).keySet()){
				HashMap edgeCopyGraph = new HashMap<Integer, edge>();
				node copyNode= (node) nodeMapCopyGraph.get(y);
				edge edgeCopy=new edge();  //(node start , node end ,double weight)
				edgeCopyGraph.put(y,copyNode);
				if (neighboreCopyGraph.containsKey(x))
					neighboreCopyGraph.get(x).putAll(edgeCopyGraph);
				else neighboreCopyGraph.put(x,edgeCopyGraph);//.put(y,neighbore.get(x));
			}
		}
		graph graphCopy=new DGraph(nodeMapCopyGraph,neighboreCopyGraph);
		return graphCopy;
	}

		/**
		 *Function to  computes a relatively short path which visit each node in the targets List.
		 *Takes the first point and runs on all points
		 * Who perishes the shortest but continues the same process with her
		 * until it reaches the final point
		 * @param targets
		 * @return  a simple path going over all nodes in the Path throw all List .
		 */
		@Override
		public List<node_data> TSP(List<Integer> targets) {
			Stack<node_data> toTSp = new Stack<node_data>();
			if (targets.size()<1) throw new RuntimeException("is emptey");
			int smul,previous=0,indexNodeStart=0,indexRemove=0;
			Iterator<Integer> ranList=targets.iterator();
			indexNodeStart = ranList.next();
			indexRemove =targets.indexOf(indexNodeStart);
			toTSp.push(this.algoGraph.getNodeMap().get(indexNodeStart));
			targets.remove(indexRemove);
			while (targets.size()>1) {
				ranList=targets.iterator();
				smul = MinIndexWeight(indexNodeStart, ranList);
				toTSp.push(this.algoGraph.getNodeMap().get(smul));
				indexNodeStart=smul;
				indexRemove =targets.indexOf(smul);
				targets.remove(indexRemove);
			}
			toTSp.push(this.algoGraph.getNodeMap().get(targets.get(0)));
	//		Collections.reverse((List<node_data>) toTSp); // flip the order of the list
			return toTSp;
		}

		private int MinIndexWeight(int indexNodeStart, Iterator<Integer> ranList) {
			int indexNodeRan=0,seavIndex=0;
			double indexWeightNew=0,indexWeightSmull=Double.MAX_VALUE;
			boolean flag=false;
			while (ranList.hasNext()){
				indexNodeRan=ranList.next();
				if(!(algoGraph.getNodeMap().containsKey(indexNodeRan))) {
					throw new RuntimeException("src  or dst is not in the graph");
				}
				if (flag==false){this.shortestPath(indexNodeStart,indexNodeRan); flag=true;}

				indexWeightNew=algoGraph.getNodeMap().get(indexNodeRan).getWeight();
				if (indexWeightNew<indexWeightSmull) {
					indexWeightSmull = indexWeightNew;
					seavIndex=indexNodeRan;
				}
			}
			return seavIndex;
		}

	/**
	 *
	 * @return a String represantation of the alggo graph
	 */
	@Override
	public String toString() {
		return "Graph_Algo{" +
				"algoGraph=" + algoGraph.toString()+
				'}';
	}


	/**
	 *
	 * @return the getAlgoGraph
	 */
	public DGraph getAlgoGraph() {
		return algoGraph;
	}

	/**
	 *
	 * @param algoGraph  the getAlgoGraph  to be sted
	 */
	public void setAlgoGraph(DGraph algoGraph) {
		this.algoGraph = algoGraph;
	}




	public static void main(String[] args) {

//
//		Graph_Algo algo = new Graph_Algo();
//		DGraph test = new DGraph();
//		node zero = new node(800,600);
//		node one = new node(900,300);
//		node two = new node(1300,786);
//		node three = new node(252,162);
//		node four = new node(350,100);
//		node five = new node(55,220);
//		test.addtoGraph(zero,one,true,true,30,50);
//		test.addtoGraph(zero,two,true,true,30,50);
//		test.addtoGraph(zero,three,true,true,300,300);
//		test.addtoGraph(one,two,true,true,30,50);
//		test.addtoGraph(three,four,true,true,30,30);
//		test.addtoGraph(two,three,true,true,10,10);
//		test.addtoGraph(zero,five,true,true,100000,100000);
//		test.addtoGraph(four,five,true,true,10,10);
//		algo.init(test);
//		//System.out.println(algo.isConnected());
//		 Queue<node_data> testA =(Queue<node_data>) algo.shortestPath(0,5);
//		for (node_data x :testA
//			 ) {
//			node temp = (node) x;
//			System.out.println(temp.getID());
//		}
//		System.out.println("size of path "+testA.size());
//		algo.save("text1");
//		Graph_Algo algo2 = new Graph_Algo();
//		algo2.init("text1");
//		System.out.println(algo2.toString());
////		Gui temp=new Gui();
////		Gui.drawGraph(algo2.algoGraph);




	}
}
