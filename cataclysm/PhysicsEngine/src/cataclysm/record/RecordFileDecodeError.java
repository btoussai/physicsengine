package cataclysm.record;

/**
 * Represents an error arising while decoding a record file.
 * 
 * @author Briac
 *
 */
public class RecordFileDecodeError extends RuntimeException {

	public RecordFileDecodeError(String string) {
		super(string);
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

}
