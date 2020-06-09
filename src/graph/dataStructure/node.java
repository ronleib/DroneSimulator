package graph.dataStructure;

//import org.jetbrains.annotations.NotNull;



import java.util.Comparator;


/**
 * This class represents an object of the kind
 * node (vertex) in a (directional) weighted graph.
 * @author simon.pikalov & ronn leib
 *
 */
public class node implements node_data {
    private static int nextFreeID;
    private Point3D point;
    private int ID;
    private double weight;
    private int tag=-1; // the temp value of the node we came from
    private node hisoty;
    private String important_pointleft="no";
    private String important_pointRigt="no";
    private  String info;
    public  double Straight,Left,Right;

    public double getStraight() {
        return Straight;
    }

    public double getLeft() {
        return Left;
    }

    public double getRight() {
        return Right;
    }

    public double getTimeold() {
        return timeold;
    }

    public double time;
    public double timeold;

    /**
     * an empty constructor that set the ID with a static field of ID
     * if you want to creat multiple graph's be careful with this method becuse the field of the node's is'nt
     * going to be zeroed at any point unless you are going to use the zero function.
     * the weight is being set to Integer.ma value by default
     */
    public node() {
        this.ID =nextFreeID;
        nextFreeID++; // move the conter up
        this.point = new Point3D(0, 0, 0);
        this.weight = 0;
    }

    /**
     * an building contractor
     * @param x the x coordinate
     * @param y the y coordinate
     * @param weight the wight of vertex recommended to set to Integer max Value if you dont know  the Shortest Path to a specific sours.
     * @param s the drone.lidars Straight
     * @param l the drone.lidars Left
     * @param r the drone.lidars Right
     */
    public node(double x, double y, double weight,double s,double l ,double r,double v) {
        this.ID = nextFreeID;
        nextFreeID++; // move the conter up
        this.point = new Point3D(x, y);
        this.weight = weight;
        Straight=s;
        Left=l;
        Right=r;
        time=System.currentTimeMillis();
        timeold =v;
    }

    /**
     * an building contractor
     * @param x the x coordinate
     * @param y the y coordinate
     * @param weight the wight of vertex recommended to set to Integer max Value if you dont know  the Shortest Path to a specific sours.
     */
    public node(double x, double y, double weight) {
        this.ID = nextFreeID;
        nextFreeID++; // move the conter up
        this.point = new Point3D(x, y);
        this.weight = weight;

    }

    /**
     * * an building  constructor that set the ID with a static field of ID
     *      * if you want to creat multiple graph's be careful with this method becuse the field of the node's is'nt
     *      * going to be zeroed at any point unless you are going to use the zero function.
     * @param x the x coordinate
     * @param y the y coordinate
     * @param ID  the ID of the vertex
     * @param weight  the wight of vertex recommended to set to Integer max Value if you dont know  the Shortest Path to a specific sours.
     *
     */
    public node(double x, double y,int ID,int weight) {
        this.ID = ID;
        this.point = new Point3D(x, y);
        this.weight=Integer.MAX_VALUE;
        this.weight = weight;

    }

    /**
     *
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public node(double x, double y) {
        this.ID = nextFreeID;
        nextFreeID++; // move the conter up
        this.point = new Point3D(x, y);
        this.weight =0;


    }

    /**
     * copy constactor that copies  the other node ID,weight,Coordinate .
     * @param other the node to be copied
     */
    public node(node_data other) {
        this.point = new Point3D(other.getLocation());
        this.weight = other.getWeight();
        this.ID = other.getKey();
    }

    /**
     * a Method that copies  the other node ID,weight,Coordinate (pakage private mathod ! ).
     * @param other the node to be copied
     */

    void deepCopy(node other) { //
        if (other.equals(null)) return;
        this.ID = other.ID;
        this.point = other.point;
        this.weight = other.weight;
        this.weight = other.weight;



    }


    public void ZeroTheId() {
        nextFreeID=0;
    }

    /**
     * a method to check if two object's are equal (that dosendt check the weight of the nodes)
     * @param o the object to compare
     * @return if the ID and the Cord is the same
     */
    @Override

    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        node node = (node) o;
        return ID == node.ID &&
                point.equals(node.point);

    }


    /**
     *
     * @return the ID of curr node
     */
    @Override
    public int getKey() {
        return this.ID;
    }

    /**
     *
     * @return the Point 3d of node
     */
    @Override
    public Point3D getLocation() {
        return this.point;
    }


    /**
     *
     * @param p - new new location  (position) of this node.
     */
    @Override
    public void setLocation(Point3D p) {
        this.point = p;

    }

    /**
     *
     * @return the weight of curr node
     */
    @Override
    public double getWeight() {
        return weight;
    }


    /**
     *
     * @param w - the new weight
     */
    @Override
    public void setWeight(double w) {
        this.weight = w;
    }

    /**
     *
     * @return the String of info
     */
    @Override
    public String getInfo() {
        return info;}


    /**
     *
      * @param s the info  (String) of this node.
     */
    @Override
    public void setInfo(String s) {
        info=s;
    }

    /**
     *
     * @return the Tag value (int)
     */
    @Override
    public int getTag() {
        return this.tag;
    }


    /**
     *
     * @param t - the new value of the tag
     */
    @Override
    public void setTag(int t) {
        this.tag = t;
    }


    /**
     *
     * @return the next free ID of the curr node
     */
    public static int getNextFreeID() {
        return nextFreeID;
    }

    /**
     *
     * @param nextFreeID the new Next fre index
     */
    public static void setNextFreeID(int nextFreeID) {
        node.nextFreeID = nextFreeID;
    }




    public int getID() {
        return ID;
    }

    public void setID(int ID) {
        this.ID = ID;
    }

     public node getHisoty() {
        return hisoty;
    }

    public void setHisoty(node hisoty) {
        this.hisoty = hisoty;
    }

    public String getImportant_pointRigt() {
        return important_pointRigt;
    }

    public   void setImportant_pointRigt(String important_point) {
        this.important_pointRigt = important_point;
    }

    public String getImportant_pointleft() {
        return important_pointleft;
    }

    public   void setImportant_pointleft(String important_point) {
        this.important_pointleft = important_point;
    }

    @Override
    public String toString() {
        String all="node{" +
                ", ID=" + ID +
                "point="+point.toString()+
                '}';

        return all;
    }

    public static void main(String[] args) {
        DGraph test = new DGraph();
        node_data zero = new node(800,600,0,0);
        node_data one = new node(900,300,1,0);
        node_data two = new node(1300,786,2,0);
        node_data three = new node(252,162,3,0);
        node_data four = new node(350,100,4,0);
        node_data five = new node(55,220,5,0);
        test.addtoGraph(zero,one,true,true,50,50);
        test.addtoGraph(zero,two,true,true,30,50);
        test.addtoGraph(zero,three,true,true,300,300);
        test.addtoGraph(one,two,true,true,30,50);
        test.addtoGraph(three,four,true,true,30,30);
        test.addtoGraph(two,three,true,true,10,10);
        test.addtoGraph(zero,five,true,true,100000,100000);
        test.addtoGraph(four,five,true,true,10,10);
   //     Gui.drawGraph(test);
    }

    /**
     * this is an Comparator of the node priority Queue
     */
    public static Comparator<node_data> nodeComparator = new Comparator<node_data>() {
        @Override
        public int compare(node_data s1, node_data s2) {
            return (int) (s1.getWeight() - s2.getWeight());
        }


        @Override
        public boolean equals(Object obj) {
            return super.equals(obj);
        }
    };



}
