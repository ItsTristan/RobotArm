package ca.ualberta;

import lejos.utility.Matrix;

public class Point3D {
	//reference: http://introcs.cs.princeton.edu/java/33design/Point.java.html
	protected final double x;    // x-coordinate
    protected final double y;    // y-coordinate
    protected final double z;	 // z-coordinate
   
    /* 
     * makes a random point
     */
    public Point3D() {
        x = Math.random();
        y = Math.random();
        z = Math.random();
    }

   /** 2D point initialized from parameters
    * 
    * @param x
    * @param y
    */
    public Point3D(double x, double y) {
        this.x = x;
        this.y = y;
        this.z = 0;
    }
    
    /** 
     * point initialized from parameters
     * @param x
     * @param y
     * @param z
     */
    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    /**
     * point initialized from a matrix (3x1 vector)
     * @param p matrix (3x1 vector)
     */
    public Point3D(Matrix p) {
    	this.x = p.get(0,0);
    	this.y = p.get(1,0);
    	this.z = p.get(2,0);
    }

    /** 
     * accessor methods
     */
    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }
    

    /**
     *  Euclidean distance between this point and that point
     */
    public double distance(Point3D that) {
        double dx = this.x - that.x;
        double dy = this.y - that.y;
        double dz = this.z - that.z;
        return Math.sqrt(dx*dx + dy*dy +dz*dz);
    }
    /** 
     * return a string representation of this point
     */
    public String toString() {
    	return String.format("(%.2f, %.2f, %.2f)", x,y,z);
    } 
    /**
     * checks if this point is the same as some object.
     * two points are the same if their distance is approximately 0.
     */
    public boolean equals(Object other){
    	// Make sure the object we're comparing is the right type
    	if (!(other instanceof Point3D))
    		return false;
    	// If it is, then they're the same if their distance
    	// is indistinguishable up to precision.
    	Point3D that = (Point3D) other;
    	return this.distance(that) < Math.pow(10, -13);
    }
    
    /**
     * Subtracts one point from another as vector operations. Used for
     * offsetting a point by an origin.
     */
    public Point3D minus(Point3D that) {
    	return new Point3D(this.x - that.x, this.y - that.y, this.z - that.z);
    }
    
    /**
     * Converts the point into an array of doubles
     */
    public double[] toDouble() {
    	return new double[] {this.x, this.y, this.z};
    }
    
    /**
     * Converts the point into a matrix of homogeneous coordinates.
     */
    public Matrix toHomogenousCoords() {
    	return new Matrix(new double[] {this.x, this.y, this.z, 1}, 4);
    }

	/**
	 * Converts the point into a matrix (vector) of 3 points.
	 */
	public Matrix toMatrix() {
    	return new Matrix(new double[] {this.x, this.y, this.z}, 3);
	}
    
}
