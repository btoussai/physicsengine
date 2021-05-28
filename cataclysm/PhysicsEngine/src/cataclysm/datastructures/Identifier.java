package cataclysm.datastructures;

/**
 * All instances of this class can be uniquely identified through their ID.
 * 
 * @author Briac
 *
 */
public abstract class Identifier {

	private final long ID;

	public Identifier(long ID) {
		this.ID = ID;
	}

	public long getID() {
		return ID;
	}

	@Override
	public int hashCode() {
		return (int) ID;

		/*int x = (int) ID;
		x = ((x >>> 16) ^ x) * 0x45d9f3b;
		x = ((x >>> 16) ^ x) * 0x45d9f3b;
		x = (x >>> 16) ^ x;
		return x;
		*/
	}

	@Override
	public boolean equals(Object obj) {
		if (obj instanceof Identifier) {
			return ((Identifier) obj).getID() == ID;
		}
		return false;
	}
	
	@Override
	public String toString() {
		return "ID = " + ID;
	}

}
