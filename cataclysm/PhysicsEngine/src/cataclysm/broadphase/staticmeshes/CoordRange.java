package cataclysm.broadphase.staticmeshes;

/**
 * Repr�sente un itervalle entre deux coordonn�es enti�res en 3D.
 * 
 * @author Briac
 *
 */
class CoordRange {

	private final Coord start = new Coord(0, 0, 0);
	private final Coord end = new Coord(0, 0, 0);
	private final Coord iterator = new Coord(0, 0, 0);

	private int x;
	private int y;
	private int z;
	private int count;

	public CoordRange() {
		
	}
	
	public void setStart(int x, int y, int z) {
		start.set(x, y, z);
	}
	
	public void setStop(int x, int y, int z) {
		end.set(x, y, z);
	}

	public void startIteration() {
		x = start.getX();
		y = start.getY();
		z = start.getZ();
		count = (end.getX() - x + 1) * (end.getY() - y + 1) * (end.getZ() - z + 1);
	}

	public boolean next() {
		if (count == 0) {
			return false;
		}

		iterator.set(x, y, z);
		count--;

		if (count != 0) {
			z++;
			if (z > end.getZ()) {
				z = start.getZ();
				y++;
				if (y > end.getY()) {
					y = start.getY();
					x++;
				}
			}
		}

		return true;
	}
	
	public Coord getIterator() {
		return iterator;
	}

	@Override
	public String toString() {
		return "Range: " + start.toString() + " --> " + end.toString();
	}

}
