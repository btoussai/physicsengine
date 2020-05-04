package cataclysm.record;

/**
 * Represents an object which can be read from/wrote to a RecordFile.
 * Non-abstract classes implementing this interface must be final because any
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
	
	static int arraySize(ReadWriteObject[] array) {
		if(array.length == 0) {
			return 4;
		}
		return 4 + array.length * array[0].size();
	}

}
