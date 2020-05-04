package cataclysm.record;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel.MapMode;
import java.util.function.Function;

import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Represents a file in which a record is saved
 * 
 * @author Briac
 *
 */
public class RecordFile {
	private final int BUFFER_LENGTH = 1024*1024;
	private final String filePath;
	private ByteBuffer buffer;

	private final RandomAccessFile raf;
	private final BufferedOutputStream out;
	private boolean closed = false;
	private int position = 0;
	private boolean reading;

	/**
	 * Creates a new file in which a record will be saved or opens a record
	 * previously saved on disk
	 * 
	 * @param path    the path of the file
	 * @param reading true if the file should be read from <br>
	 *                false if the file should be written to
	 * @throws IOException
	 */
	public RecordFile(String path, boolean reading) throws IOException {
		this.filePath = path;
		File f = new File(path);
		if (reading) {
			raf = new RandomAccessFile(f, "r");
			if (raf.length() > Integer.MAX_VALUE) {
				throw new IllegalArgumentException("Error the file size is bigger than Integer.MAX_VALUE");
			}else {
				System.out.println("Opening record file of length " + raf.length() + " bytes");
			}

			this.out = null;
			this.buffer = raf.getChannel().map(MapMode.READ_ONLY, position, raf.length());
		} else {
			this.raf = null;
			this.buffer = ByteBuffer.allocate(BUFFER_LENGTH);
			this.out = new BufferedOutputStream(new FileOutputStream(f));
		}
		this.reading = reading;
	}

	public boolean isReading() {
		return reading;
	}

	public boolean isWritting() {
		return !reading;
	}
	
	

	/**
	 * Writes the remaining data contained in the bytebuffer into the file and then
	 * closes it.
	 * @return the total file size
	 */
	public int flushAndClose() {
		if (reading) {
			throw new IllegalStateException(
					"Cannot call flushAndClose() when reading, close() should be called instead.");
		}
		try {
			System.out.println("flushAndClose ! writing " + buffer.position() + " bytes");
			out.write(buffer.array(), 0, buffer.position());
			position += buffer.position();
			buffer.rewind();
			out.close();
			closed = true;
			buffer = null;
		} catch (IOException e) {
			e.printStackTrace();
		}
		return position;
	}

	/**
	 * Closes the file being read from.
	 */
	public void close() {
		if (!reading) {
			throw new IllegalStateException(
					"Cannot call close() when writing, flushAndClose() should be called instead.");
		}
		try {
			raf.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		closed = true;
		buffer = null;
	}

	public boolean isClosed() {
		return closed;
	}

	/**
	 * Flushes the buffer to the file and empties it if there is not enough space in
	 * the buffer to store size bytes.
	 * 
	 * @param size
	 */
	private void flushToFile(int size) {
		int remaining = buffer.remaining();
		if (remaining < size) {
			try {
				System.out.println("flushing to file " + buffer.position() + " bytes");
				out.write(buffer.array(), 0, buffer.position());
				position += buffer.position();
				buffer.rewind();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public void writeByte(byte b) {
		flushToFile(1);
		buffer.put(b);
	}

	public byte readByte() {
		return buffer.get();
	}

	public void writeBool(boolean b) {
		flushToFile(1);
		buffer.put((byte) (b ? 1 : 0));
	}

	public boolean readBool() {
		return buffer.get() != 0;
	}

	public void writeInt(int i) {
		flushToFile(4);
		buffer.putInt(i);
	}

	public int readInt() {
		return buffer.getInt();
	}

	public void writeLong(long l) {
		flushToFile(8);
		buffer.putLong(l);
	}

	public long readLong() {
		return buffer.getLong();
	}

	public void writeFloat(float f) {
		flushToFile(4);
		buffer.putFloat(f);
	}

	public float readFloat() {
		return buffer.getFloat();
	}

	public void writeDouble(double d) {
		flushToFile(8);
		buffer.putDouble(d);
	}

	public double readDouble() {
		return buffer.getDouble();
	}

	public void writeVector3f(Vector3f v) {
		flushToFile(3 * 4);
		buffer.putFloat(v.x);
		buffer.putFloat(v.y);
		buffer.putFloat(v.z);
	}

	public void readVector3f(Vector3f v) {
		v.x = buffer.getFloat();
		v.y = buffer.getFloat();
		v.z = buffer.getFloat();
	}

	public void writeMatrix3f(Matrix3f m) {
		flushToFile(9 * 4);
		buffer.putFloat(m.m00);
		buffer.putFloat(m.m01);
		buffer.putFloat(m.m02);
		buffer.putFloat(m.m10);
		buffer.putFloat(m.m11);
		buffer.putFloat(m.m12);
		buffer.putFloat(m.m20);
		buffer.putFloat(m.m21);
		buffer.putFloat(m.m22);
	}

	public void readMatrix3f(Matrix3f m) {
		m.m00 = buffer.getFloat();
		m.m01 = buffer.getFloat();
		m.m02 = buffer.getFloat();
		m.m10 = buffer.getFloat();
		m.m11 = buffer.getFloat();
		m.m12 = buffer.getFloat();
		m.m20 = buffer.getFloat();
		m.m21 = buffer.getFloat();
		m.m22 = buffer.getFloat();
	}

	public Vector3f[] readVector3fArray() {
		Vector3f[] array = new Vector3f[readInt()];
		for (int i = 0; i < array.length; i++) {
			Vector3f v = new Vector3f();
			readVector3f(v);
			array[i] = v;
		}
		return array;
	}

	public void writeVector3fArray(Vector3f[] array) {
		writeInt(array.length);
		for (int i = 0; i < array.length; i++) {
			writeVector3f(array[i]);
		}
	}

	public ReadWriteObject[] readArray(Function<Integer, ReadWriteObject[]> arrayConstructor, Function<RecordFile, ReadWriteObject> elementConstructor) {
		ReadWriteObject[] array = arrayConstructor.apply(readInt());
		for (int i = 0; i < array.length; i++) {
			array[i] = elementConstructor.apply(this);
		}
		return array;
	}

	public void writeArray(ReadWriteObject[] array) {
		writeInt(array.length);
		for (int i = 0; i < array.length; i++) {
			array[i].write(this);
		}
	}

	/**
	 * Reads an ascii line from the file (each character is encoded as a byte). The
	 * end of line character '\n' is not appended to the stringbuilder
	 * 
	 * @param str
	 */
	public void readLine(StringBuilder str) {
		str.delete(0, str.length());
		byte b;
		while ((b = readByte()) != '\n') {
			str.append((char) b);
		}
	}

	/**
	 * Writes an ascii string in the file and appends an end of line character '\n'.
	 * 
	 * @param str
	 */
	public void writeLine(StringBuilder str) {
		for (int i = 0; i < str.length(); i++) {
			writeByte((byte) str.charAt(i));
		}
		writeByte((byte) '\n');
	}

	/**
	 * Writes an ascii string in the file and appends an end of line character '\n'.
	 * 
	 * @param str
	 */
	public void writeLine(String str) {
		for (int i = 0; i < str.length(); i++) {
			writeByte((byte) str.charAt(i));
		}
		writeByte((byte) '\n');
	}

	/**
	 * @return the position in the file of the next byte which will be written or
	 *         read.
	 */
	public int getPosition() {
		if(!reading) {
			return position + buffer.position();
		}else {
			return buffer.position();
		}
	}

	public String getFilePath() {
		return filePath;
	}

	/**
	 * Reads the next int but does not consume it.
	 * 
	 * @return
	 */
	public int peekInt() {
		int value = buffer.getInt();
		buffer.position(buffer.position() - 4);
		return value;
	}

	/**
	 * Skip n bytes forward or backward when reading, throws an exception if called
	 * when writing
	 * 
	 * @param bytes
	 */
	public void skipBytes(int bytes) {
		if (!reading) {
			throw new IllegalStateException("Cannot skip bytes when writing");
		}
		int new_pos = buffer.position() + bytes;
		if (new_pos < 0 || new_pos > buffer.limit()) {
			throw new IllegalArgumentException("Invalid skip count");
		}
		buffer.position(new_pos);
	}

	public void seek(int position) {
		if (!reading) {
			throw new IllegalStateException("Cannot skip bytes when writing");
		}
		if (position < 0 || position > buffer.limit()) {
			throw new IllegalArgumentException("Invalid position in file");
		}
		buffer.position(position);
	}

}
