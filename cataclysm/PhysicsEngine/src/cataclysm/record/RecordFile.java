package cataclysm.record;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
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
	private final int BUFFER_LENGTH = 1024 * 1024;

	private ByteBuffer buffer = ByteBuffer.allocate(BUFFER_LENGTH);
	private BufferedOutputStream out;
	private BufferedInputStream in;
	private boolean closed = false;

	/**
	 * Creates a new file in which a record will be saved or opens a record
	 * previously saved on disk
	 * 
	 * @param path the path of the file
	 * @param in   true if the file should be read from <br>
	 *             false if the file should be written to
	 * @throws FileNotFoundException if the file doesn't exist or if it couldn't be
	 *                               created
	 */
	public RecordFile(String path, boolean in) throws FileNotFoundException {
		File f = new File(path);
		if (in) {
			this.in = new BufferedInputStream(new FileInputStream(f));
			this.out = null;
		} else {
			this.in = null;
			this.out = new BufferedOutputStream(new FileOutputStream(f));
		}

	}

	/**
	 * Writes the remaining data contained in the bytebuffer into the file and then
	 * closes it.
	 */
	public void flushAndClose() {
		try {
			System.out.println("flushAndClose ! writing " + buffer.position() + " bytes");
			out.write(buffer.array(), 0, buffer.position());
			buffer.rewind();
			out.close();
			closed = true;
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Closes the file being read from.
	 */
	public void close() {
		try {
			in.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		closed = true;
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
				buffer.compact();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Replenishes the buffer from the file if the buffer contains less than size
	 * bytes
	 * 
	 * @param size
	 */
	private void replenishFromFile(int size) {
		int remaining = buffer.remaining();
		if (remaining < size) {
			try {
				buffer.compact();
				int toRead = buffer.limit() - buffer.position();
				System.out.println("replenishing " + toRead + " bytes");
				int count = in.read(buffer.array(), buffer.position(), toRead);
				buffer.rewind();
				System.out.println("read " + count + " bytes");

				if (count == -1) {
					in.close();
					closed = true;
				} else {
					buffer.limit(count + remaining);
				}
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
		replenishFromFile(1);
		return buffer.get();
	}

	public void writeBool(boolean b) {
		flushToFile(1);
		buffer.put((byte) (b ? 1 : 0));
	}

	public boolean readBool() {
		replenishFromFile(1);
		return buffer.get() != 0;
	}

	public void writeInt(int i) {
		flushToFile(4);
		buffer.putInt(i);
	}

	public int readInt() {
		replenishFromFile(4);
		return buffer.getInt();
	}

	public void writeLong(long l) {
		flushToFile(8);
		buffer.putLong(l);
	}

	public long readLong() {
		replenishFromFile(8);
		return buffer.getLong();
	}

	public void writeFloat(float f) {
		flushToFile(4);
		buffer.putFloat(f);
	}

	public float readFloat() {
		replenishFromFile(4);
		return buffer.getFloat();
	}

	public void writeDouble(double d) {
		flushToFile(8);
		buffer.putDouble(d);
	}

	public double readDouble() {
		replenishFromFile(8);
		return buffer.getDouble();
	}

	public void writeVector3f(Vector3f v) {
		flushToFile(3 * 4);
		buffer.putFloat(v.x);
		buffer.putFloat(v.y);
		buffer.putFloat(v.z);
	}

	public void readVector3f(Vector3f v) {
		replenishFromFile(3 * 4);
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
		replenishFromFile(9 * 4);
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

	public ReadWriteObject[] readArray(Function<RecordFile, ReadWriteObject> constructor) {
		ReadWriteObject[] array = new ReadWriteObject[readInt()];
		for (int i = 0; i < array.length; i++) {
			array[i] = constructor.apply(this);
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

}
