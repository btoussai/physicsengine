package cataclysm.record;

/**
 * Represents an object which can be read from/wrote to a RecordFile.
 * Non-abstract classes implementing this interface should be final because any
 * information about the type of the object is not saved.
 * 
 * @author Briac
 *
 */
public interface ReadWriteObject {

	public void read(RecordFile f);

	public void write(RecordFile f);
	
	/**
	 * @return the size in bytes of the object
	 */
	public int size();

}
