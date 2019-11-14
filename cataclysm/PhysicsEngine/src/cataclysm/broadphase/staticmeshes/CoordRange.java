package cataclysm.broadphase.staticmeshes;

/**
 * Représente un itervalle entre deux coordonnées entières en 3D.
 * 
 * @author Briac
 *
 */
class CoordRange {

	private final Coord start = new Coord(0, 0, 0);
	private final Coord end = new Coord(0, 0, 0);

	private int x;
	private int y;
	private int z;
	private int count;

	public CoordRange(Coord start, Coord end) {
		setFrom(start, end);
	}

	public void setFrom(Coord start, Coord end) {
		this.start.set(start);
		this.end.set(end);
	}

	public void startIteration() {
		x = start.getX();
		y = start.getY();
		z = start.getZ();
		count = (end.getX() - x + 1) * (end.getY() - y + 1) * (end.getZ() - z + 1);
	}

	public boolean next(Coord dest) {
		if (count == 0) {
			return false;
		}

		dest.set(x, y, z);
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

	@Override
	public String toString() {
		return "Range: " + start.toString() + " --> " + end.toString();
	}

}
