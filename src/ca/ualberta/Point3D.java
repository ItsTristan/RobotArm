package ca.ualberta;

import lejos.utility.Matrix;

public class Point3D {
	//http://introcs.cs.princeton.edu/java/33design/Point.java.html
	protected final double x;    // x-coordinate
    protected final double y;    // y-coordinate
    protected final double z;	 // z-coordinate
   
    // random point
    public Point3D() {
        x = Math.random();
        y = Math.random();
        z = Math.random();
    }

   // point initialized from parameters
    public Point3D(double x, double y) {
        this.x = x;
        this.y = y;
        this.z = 0;
    }
    
    // point initialized from parameters
    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // accessor methods
    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }
    

    // Euclidean distance between this point and that point
    public double distance(Point3D that) {
        double dx = this.x - that.x;
        double dy = this.y - that.y;
        double dz = this.z - that.z;
        return Math.sqrt(dx*dx + dy*dy +dz*dz);
    }
    // return a string representation of this point
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    } 
    public boolean equals(Object other){
    	// Make sure the object we're comparing is the right type
    	if (!(other instanceof Point3D))
    		return false;
    	// If it is, then they're the same if their distance
    	// is indistinguishable up to precision.
    	Point3D that = (Point3D) other;
    	return this.distance(that) < Math.pow(10, -13);
    }
    
    public Point3D minus(Point3D that) {
    	return new Point3D(this.x - that.x, this.y - that.y, this.z - that.z);
    }
    
    public double[] toDouble() {
    	return new double[] {this.x, this.y, this.z};
    }
    
    public Matrix toHomogenousCoords() {
    	return new Matrix(new double[] {this.x, this.y, this.z, 1}, 4);
    }

	public Matrix toMatrix() {
    	return new Matrix(new double[] {this.x, this.y, this.z}, 3);
	}
    
}
