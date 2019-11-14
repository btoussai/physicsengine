package cataclysm.broadphase.staticmeshes;

/**
 * Représente un point défini par 3 coordonnées entières.
 * 
 * @author Briac
 *
 */
class Coord implements Comparable<Coord>{
	
	private int x;
	private int y;
	private int z;
	
	private int hash;
	
	Coord(int x, int y, int z){
		this.x = x;
		this.y = y;
		this.z = z;
		this.hash = 31*(31*(31+x) + y) + z;
	}
	
	public void set(Coord other) {
		this.x = other.x;
		this.y = other.y;
		this.z = other.z;
		this.hash = 31*(31*(31+x) + y) + z;
	}
	
	public void set(int x, int y, int z) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.hash = 31*(31*(31+x) + y) + z;
	}
	
	@Override
	public boolean equals(Object o) {
		if(o == this) {
			return true;
		}
		if(o == null || !(o instanceof Coord)) {
			return false;
		}
		Coord other = (Coord) o;
		return this.x == other.x && this.y == other.y && this.z == other.z;
	}
	
	@Override
	public int hashCode() {
		return this.hash;
	}

	@Override
	public int compareTo(Coord other) {
		return this.hash - other.hash;
	}
	
	@Override
	public String toString() {
		return "Coords[" + x + ", " + y + ", " + z + "]  #" + hash;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public int getZ() {
		return z;
	}

}
