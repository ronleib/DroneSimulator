package graph.dataStructure;

import graph.dataStructure.graph;

import java.util.Objects;


/**
 * This class represents an object of
 * directional edge(src,dest) in a (directional) weighted graph.
 * @author Simon Pikalov & Ron Leib
 *
 */
public class edge implements edge_data {

    private node_data Src;
    private node_data Dst;
    private double weight;
    private int tag;
    private  String info;


    /**
     *
     * @param start node start the node that conecctrd to the end node
     * @param end node End
     * @param weight  the weight of edge must be Possitive
     */
    public edge(node_data start , node_data end ,double weight){

//        if(weight<0) throw  new RuntimeException("weight must be Possitive !") ;

        if (!((start instanceof node)&&(end instanceof node))) {
            throw new RuntimeException("unsuported node data ");
        }
        this.Src=start;
        this.Dst=end;
        this.weight= weight;
    }

    /**
     *
     * @param start node start the node that conecctrd to the end node
     * @param end node End
     */
    public void edge(node_data start , node_data end){

//        if(weight<0) throw  new RuntimeException("weight must be Possitive !") ;

        if (!((start instanceof node)&&(end instanceof node))) {
            throw new RuntimeException("unsuported node data ");
        }
        this.Src=start;
        this.Dst=end;
        this.weight= Point3D.distance2D(start.getLocation(),end.getLocation());
    }

    /**
     * an Empty constractor that set's the weight to 0 by default and creates a
     * two empty nodes , start and end
     */
    public edge() {
        Src = new node();
        Dst= new node();
        weight=0;
        info="";
    }

    /**
     *
     * @return copy an deep copy  of th edge to be copied
     */
    public  edge_data copyEdge(){
        node_data copySrc= new node(this.Src);
        node_data  copyDst=new node(this.Dst);
        double  copyweight=this.getWeight();
        int     copytag;
        edge copyEd=new edge(copySrc,copyDst,copyweight);
        copyEd.setTag(this.getTag());
        return copyEd;
    }


    @Override
    /**
     * check if two object are insteseof the same class and has the same nodes Values (by Node Equals Method)
     * and f they has the same weight
     */
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        edge edge = (edge) o;
        return Double.compare(edge.weight, weight) == 0 &&
                Objects.equals(Src, edge.Src) &&
                Objects.equals(Dst, edge.Dst);
    }



    /**
     *@return The id of the source node of this edge.
     */
    @Override

    public int getSrc() {
        return this.Src.getKey();
    }

    /**
     *
     * @returnThe  id of the destination node of this edge
     */
    @Override
    public int getDest() {return this.Dst.getKey();}

    /**
     *
     * @return the weight of this edge (positive value).
     */
    @Override
    public double getWeight() {
        return this.weight;
    }


    /**
     *
     * @return return the remark (meta data) associated with this edge.
     */
    @Override
    public String getInfo() {
        return info;
    }


    /**
     *
     * @param s allows changing the remark (meta data) associated with this edge.
     */
    @Override
    public void setInfo(String s) {
    info=s;
    }

    /**
     *
     * @return Temporal data (aka color: e,g, white, gray, black)
     *     * which can be used be algorithms
     */
    @Override
    public int getTag() {
        return this.tag;
    }

    /**
     *
     * @param t - /**
     *     * Allow setting the "tag" value for temporal marking an edge - common
     *     * practice for marking by algorithms.
     *     * @param t - the new value of the tag
     *     */

    @Override
    public void setTag(int t) {
        this.tag=t;

    }

    /**
     *A function that returns the length of the edge
     * @return d-destance
     */

    @Override
    public double getDistance() {
        double Dx, Dy, D;
        Dx = (this.Dst.getLocation().x() - this.Src.getLocation().x());
        Dx = Dx * Dx;
        Dy = (this.Dst.getLocation().y() - this.Src.getLocation().y());
        Dy = Dy * Dy;
        D =  Math.sqrt(Dy + Dx);
        return D;
    }


    /**
     *
     * @return String  of edge that contain's Src, Dst,Weight
     */
    @Override
    public String toString() {
        return "edge{" +
                "Src=" + Src.toString()+
                ", Dst=" + Dst.toString()+
                ", weight=" + weight +
                '}';
    }

    /**
     *
     * @param src set the Src node of current edge
     */
    public void setSrc(node src) {
        Src = src;
    }

    /**
     *
     * @return the Dst node of current edge
     */
    public node_data getDstNOde() {
        return Dst;
    }


    /**
     *
     * @param dst set the Dst node with curr node
     */
    public void setDstNOde(node dst) {
        Dst = dst;
    }


    /**
     *
     * @param weight set the weigtt of current edge
     */
    public void setWeight(double weight) {
        this.weight = weight;
    }




}
