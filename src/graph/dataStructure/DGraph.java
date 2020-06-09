package graph.dataStructure;

import graph.dataStructure.graph;

import javax.swing.*;
import java.awt.*;
import java.util.Collection;
import java.util.HashMap;
import java.util.Objects;


/**
 * This class a directional weighted graph.
 * The class has a road-system or communication network in mind - and should support a large number of nodes (over 1,000,000).
 * The implementation is based on HashMaps and the graph has two main Hashmaps the first is the Hshmap of nodes ,and the second is the
 * hasmMap of Neighbore that has a Hashmap as value  inside evry node that has edge as value .
 *
 */
public class DGraph extends JPanel implements graph {
    private HashMap<Integer, node_data> nodeMap; // the datastcture of the Nodes
    //private HashMap<Integer[], edge> edgeMap; // the datastcture of the edges key :  [int key src , int key dst, 13(male) means from and 12 (female ) means into  ]
    private HashMap<Integer, HashMap<Integer, edge_data>> neighbore; // the neighbore of ich node the ket of the hashmap is the key of the src and the value key is the dst key and the value of the value is the edge
    private HashMap<Integer, String> MissMap = new HashMap<Integer, String>();
    ;
    private int mc;
    int edgeNumber;
    int startKey;


    /**
     * Create a Dgraph out of two Hashmap component's
     *
     * @param nodeMap   The  Hashmap of nodes
     * @param neighbore The hahmap of neighbor's
     */
    public DGraph(HashMap<Integer, node_data> nodeMap, HashMap<Integer, HashMap<Integer, edge_data>> neighbore) {
        this.nodeMap = nodeMap;
        this.neighbore = neighbore;

    }


    /**
     * an Empty constractor that initiate bouth the HashMaps
     */
    public DGraph() {
        this.nodeMap = new HashMap<Integer, node_data>();
        this.neighbore = new HashMap<Integer, HashMap<Integer, edge_data>>();

    }


    /**
     * this is a package public method for connecting two nodes in one or two direction's
     * red carefully before using this function
     *
     * @param a   first node
     * @param b   second node
     * @param AB  conect a to b
     * @param BA  connect b to a
     * @param WAB the wight of the edge between a to b
     * @param WBA the wight of the edge between b to a
     */
    public void addtoGraph(node_data a, node_data b, boolean AB, boolean BA, double WAB, double WBA) {

        nodeMap.put(a.getKey(), a);
        nodeMap.put(b.getKey(), b);

        if (AB) this.connect(a.getKey(), b.getKey(), WAB);
        if (BA) this.connect(b.getKey(), a.getKey(), WAB);

        repaint();
    }

    /**
     * return the node_data by the node_id,
     *
     * @param key - the node_id
     * @return the node_data by the node_id, null if none.
     */
    @Override
    public node_data getNode(int key) {
        return nodeMap.get(key);
    }

    /**
     * return the data of the edge (src,dest), null if none.
     *
     * @param src
     * @param dest
     * @return
     */
    @Override
    public edge_data getEdge(int src, int dest) {
        if (nodeMap.containsKey(src) && nodeMap.containsKey(dest) && neighbore.containsKey(src) && neighbore.get(src).containsKey(dest)) {
            return neighbore.get(src).get(dest);
        } else {
            return null;  // the case this edge does not exist
        }
    }

    /**
     * add a new node to the graph with the given node_data.
     *
     * @param n
     */
    @Override
    public void addNode(node_data n) {
        if (!(n instanceof node)) {
            throw new RuntimeException("unsuported node data ");
        } else if (nodeMap.containsKey(n.getKey())) {
            throw new RuntimeException("there is already node with than number");
        } else {
            nodeMap.put(n.getKey(), (node) n);
            MissMap.put(((int)n.getLocation().x()), String.valueOf(n.getKey()) + "," + String.valueOf(((int)n.getLocation().y())));
        }
        repaint();

    }

    /**
     * Connect an edge with weight w between node src to node dest  Posiible only if bouth are in the current graph .
     *
     * @param src  - the source of the edge.
     * @param dest - the destination of the edge.
     * @param w    - positive weight representing the cost (aka time, price, etc) between src-->dest.
     */
    @Override
    public void connect(int src, int dest, double w) {

        if (!(nodeMap.containsKey(src) && nodeMap.containsKey(dest))) {
            throw new RuntimeException("src or desrt are not in the graph sec: " + src + "dest : " + dest);
        } else { // if bouth nodes are in the graph
            node_data destNode = nodeMap.get(dest);
            node_data srcNode = nodeMap.get(src);
            edge_data newEdge = new edge(srcNode, destNode, w);
            if (neighbore.containsKey(src)) { // the case the hashmap with the ket src is not empty
                neighbore.get(src).put(dest, newEdge);
                edgeNumber++;
                repaint();
            } else { //if the  hashmap with the ket src is  empty  init it and put the new edge
                HashMap<Integer, edge_data> edgeMap = new HashMap<Integer, edge_data>();
                edgeMap.put(dest, newEdge);
                neighbore.put(src, edgeMap);
                edgeNumber++;
            }

            repaint();

        }

    }


    /**
     * This method return a pointer (shallow copy) for the
     * collection representing all the nodes in the graph.
     *
     * @return Collection<node_data>
     */
    @Override
    public Collection<node_data> getV() {
        if (this.nodeMap.isEmpty()) return null;
        return this.nodeMap.values();
    }


    /**
     * This method return a pointer (shallow copy) for the
     * collection representing all the edges getting out of
     * the given node (all the edges starting (source) at the given node).
     *
     * @return Collection<edge_data>
     */
    @Override
    public Collection<edge_data> getE(int node_id) {
        if (this.neighbore.containsKey(node_id) || this.neighbore.isEmpty() || this.neighbore.get(node_id).isEmpty())
            return null;
        else return this.neighbore.get(node_id).values();
    }


    /**
     * Delete the node (with the given ID) from the graph -
     * and removes all edges which starts or ends at this node.
     * and updats the MC of the Graph
     *
     * @param key
     * @return the data of the removed node (null if none).
     */
    @Override
    public node_data removeNode(int key) {

        if (!nodeMap.containsKey(key)) return null;

        this.neighbore.remove(key);
        for (int x : neighbore.keySet())
            if (neighbore.get(x).containsKey(key)) {
                neighbore.get(x).remove(key);
                this.edgeNumber--;
            }
        this.mc++;
        this.mc++;
        return this.nodeMap.remove(key);

    }

    /**
     * Delete the edge from the graph,
     * and update the MC
     *
     * @param src
     * @param dest
     * @return the data of the removed edge (null if none).
     */
    @Override
    public edge_data removeEdge(int src, int dest) {
        if (!this.neighbore.containsKey(src)) {
            return null;
        } else if (this.neighbore.get(src).containsKey(dest)) {
            mc++;
            return this.neighbore.get(src).remove(dest);
        } else return null;
    }

    /**
     * @return the number of edges
     */
    @Override
    public int nodeSize() {
        return nodeMap.size();
    }

    /**
     * @return the Mode Count - for testing changes in the graph.
     */
    @Override
    public int edgeSize() {
        return edgeNumber;
    }

    /**
     * @return the the int represantation of the changes theat the graph had.
     */
    @Override
    public int getMC() {
        return this.mc;
    }  // return the changes


    /**
     * @return the NodeMap of current graph
     */
    public HashMap<Integer, node_data> getNodeMap() {
        return nodeMap;
    }

    /**
     * @param nodeMap the node map to be seted in the nodeMap field
     */
    public void setNodeMap(HashMap<Integer, node_data> nodeMap) {
        this.nodeMap = nodeMap;
    }

    /**
     * @return the NeighboreMap of current graph
     */
    public HashMap<Integer, HashMap<Integer, edge_data>> getNeighbore() {
        return neighbore;
    }

    /**
     * @param neighbore the  neighboreMap to be seted in the nodeMap field
     */
    public void setNeighbore(HashMap<Integer, HashMap<Integer, edge_data>> neighbore) {
        this.neighbore = neighbore;
    }


    /**
     * @param mc The Mc to be seted
     */
    public void setMc(int mc) {
        this.mc = mc;
    }


    /**
     * Functionto paint a graph using JSwing
     * the function Draw the vertex in Blue Color and in red every Vertex Id
     * if there is a vertex from one node to another there is a point that represent an Arrow of direction and above the point
     * written in black the weight of current edge.
     *
     * @param g the Jframe
     */
    public void paint(Graphics g) {
        setSize(2000, 2000);

        for (int x : nodeMap.keySet()) {
            g.setColor(Color.blue);
            Point3D p = nodeMap.get(x).getLocation();
            g.fillOval((int) p.x(), (int) p.y(), 20, 20);
            String key = Integer.toString(x);
            g.setFont(new Font("Courier", Font.BOLD, 20));
            g.setColor(Color.red);
            g.drawString(key, (int) p.x(), (int) p.y());
            if (!(neighbore.containsKey(x))) continue; // if this node is not connectd
            for (int i : neighbore.get(x).keySet()) {
                if (neighbore.get(x).equals(null) || neighbore.get(x).get(i).equals(null)) continue;
                node_data bro = new node();
                bro = this.nodeMap.get((neighbore.get(x).get(i).getDest()));
                Point3D p2 = new Point3D(bro.getLocation());
                g.setColor(Color.black);
                g.drawLine((int) p.x(), (int) p.y(), (int) p2.x(), (int) p2.y());
                g.setColor(Color.red);
                g.setFont(new Font("Courier", Font.BOLD, 12));
                String weight = new Double(neighbore.get(x).get(i).getWeight()).toString(); // the weight as String
                weight = weight.substring(0, weight.indexOf('.') + 2); //make it 0.44 insted of 0.444444444444
                //g.drawString("*", (int)((p.x()+p2.x())/2),(int)((p.y()+p2.y())/2)); //if i will want to add the weight later
                if ((p.x() == p2.x()) && (p.y() == p2.y())) continue;
                int WX = (int) ((1 * p.x() + 8 * p2.x()) / 9); //section formula ratio 1:9
                int WY = (int) ((1 * p.y() + 8 * p2.y()) / 9); //section formula ratio 1:9
                g.drawString(weight, WX, WY);
                g.setColor(Color.BLACK);
                int inX = (int) ((1 * p.x() + 8 * p2.x()) / 9); //section formula ratio 1:9
                int inY = (int) ((1 * p.y() + 8 * p2.y()) / 9); //section formula ratio 1:9
                g.fillOval(inX, inY, 10, 10);


            }

        }


    }

//    /**
//     * private function for developers to creat a full grapg of 20 nodes
//     */
//    public static void creatRandomGraph() {
//         create a full graph of i nodes
//        HashMap<Integer, edge_data> map = new HashMap<>();
//        DGraph test = new DGraph();
//        node prev = new node(500,500,500);
//        for (int i = 0; i < 20; i++) {
//            int key = i;
//            node point = new node((int) (Math.random() * 1500), (int) (Math.random() * 1000), (int) (Math.random() * 100));
//            map.put(prev.getKey(), new edge(new node(point), new node(prev), Math.random()*100));
//            test.neighbore.put(i, map);
//            test.nodeMap.put(i, point);
//            prev.deepCopy(point);
//
//
//        }


//        JFrame MainFrame = new JFrame();
//        MainFrame.setSize(2000, 2000);
//
//
//        MainFrame.add(test);
//        MainFrame.setVisible(true);


//    }

    /**
     * function to Show the the paint of the graph in a 2000,2000 Frame
     */
    public void Gui() {

        JFrame MainFrame = new JFrame();
        MainFrame.setSize(2000, 2000);
        MainFrame.add(this);
        MainFrame.setVisible(true);

    }


    /**
     * @param o The object(Shiould be a Dgraph) we to compare to , it will check bouth the HashMaps to check if their are Equal.
     * @return The Boolean answer if their are equal or not
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof DGraph)) return false;
        DGraph dGraph = (DGraph) o;
        return
                edgeSize() == dGraph.edgeSize() &&
                        Objects.equals(getNodeMap(), dGraph.getNodeMap()) &&
                        Objects.equals(getNeighbore(), dGraph.getNeighbore());
    }

    /**
     * @return the Hashcode of the graph
     */
    @Override
    public int hashCode() {
        return Objects.hash(getNodeMap(), getNeighbore(), getMC(), edgeSize(), startKey);
    }


    /**
     * @return a String representation of the graph
     */
    @Override
    public String toString() {

        return "DGraph{" +
                "size of nodeMap=" + nodeMap.size() +
                ",size of neighbore=" + neighbore.size() +
                ", mc=" + mc +
                ", edgeNumber=" + edgeNumber +
                ", startKey=" + startKey +
                '}';
    }

    /**
     * this id a develper "Debug function to print th graph"
     *
     * @param neighbore
     * @return
     */
    private String print(HashMap<Integer, HashMap<Integer, edge>> neighbore) {
        for (int x : neighbore.keySet()) {
            for (int y : neighbore.get(x).keySet())
                System.out.print("" + x + "=" + y + "=" + neighbore.get(x).get(x).toString());
        }
        return "";
    }


    /**
     * @param edgeNumber the graph edge number goint to be set
     */
    public void setEdgeNumber(int edgeNumber) {
        this.edgeNumber = edgeNumber;
    }


    /**
     * this id a develper "Debug function to print th graph"
     *
     * @param
     * @return
     */
    public void miss(node s) {
        String HakeY = "";
        for (int i = 0; i < 300; i++) {
            if (MissMap.containsKey(((int)s.getLocation().x()) + i)) {
                for (int j = 0; j < 300; j++) {
                    String[] HAKE = MissMap.get(((int)s.getLocation().x())+ i).split(",");
                    HakeY = String.valueOf(((int)s.getLocation().y()) - j);
                    if (HAKE[HAKE.length - 1].equals(HakeY)) {
                        node_data temp = this.nodeMap.get(Integer.parseInt(HAKE[HAKE.length - 2]));
                        this.connect(temp.getKey(), s.getKey(), Point3D.distance2D(temp.getLocation(), s.getLocation()));
                    } else {
                        HakeY = String.valueOf(((int)s.getLocation().y()) + j);
                        if (HAKE[HAKE.length - 1].equals(HakeY)) {
                            node_data temp = this.nodeMap.get(Integer.parseInt(HAKE[HAKE.length - 2]));
                            this.connect(temp.getKey(), s.getKey(), Point3D.distance2D(temp.getLocation(), s.getLocation()));
                        }
                    }
                }
            } else {
                if (MissMap.containsKey(((int)s.getLocation().x()) - i)) {
                    for (int j = 0; j < 300; j++) {
                        String[] HAKE = MissMap.get(((int)s.getLocation().x()) + i).split(",");
                        HakeY = String.valueOf(((int)s.getLocation().y()) - j);
                        if (HAKE[HAKE.length - 1].equals(HakeY)) {
                            node_data temp = this.nodeMap.get(Integer.parseInt(HAKE[HAKE.length - 2]));
                            this.connect(temp.getKey(), s.getKey(), Point3D.distance2D(temp.getLocation(), s.getLocation()));
                        } else {
                            HakeY = String.valueOf(((int)s.getLocation().y()) + j);
                            if (HAKE[HAKE.length - 1].equals(HakeY)) {
                                node_data temp = this.nodeMap.get(Integer.parseInt(HAKE[HAKE.length - 2]));
                                this.connect(temp.getKey(), s.getKey(), Point3D.distance2D(temp.getLocation(), s.getLocation()));
                            }
                        }
                    }
                }
            }
        }
    }


//        for(int x : neighbore.keySet()){
//            for (int y: neighbore.get(x).keySet())
//                System.out.print(""+x+"="+y+"="+neighbore.get(x).get(x).toString());
//        }
//    }
//
//        if (nodeMap.containsKey(s)
//        } else {
//            return null;  // the case this edge does not exist
//        }
//    }


//    public static void main(String[] args) {

//        creatRandomGraph();

}


//}
