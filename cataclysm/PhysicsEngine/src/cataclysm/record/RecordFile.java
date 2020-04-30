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
	private final int BUFFER_LENGTH = 1024;

	private ByteBuffer buffer = ByteBuffer.allocate(BUFFER_LENGTH);
	private BufferedOutputStream out;
	private BufferedInputStream in;

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

	public void flushAndClose() {
		try {
			out.write(buffer.array(), 0, buffer.position());
			buffer.rewind();
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void flushToFile() {
		if (buffer.position() == BUFFER_LENGTH - 1) {
			try {
				out.write(buffer.array());
				buffer.rewind();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private void replenishFromFile() {
		if (buffer.remaining() == 0) {
			try {
				int count = in.read(buffer.array());
				buffer.rewind();

				if (count == -1) {
					in.close();
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public void writeBool(boolean b) {
		flushToFile();
		buffer.put((byte) (b ? 1 : 0));
	}

	public boolean readBool() {
		replenishFromFile();
		return buffer.get() != 0;
	}

	public void writeInt(int i) {
		flushToFile();
		buffer.putInt(i);
	}

	public int readInt() {
		replenishFromFile();
		return buffer.getInt();
	}

	public void writeFloat(float f) {
		flushToFile();
		buffer.putFloat(f);
	}

	public float readFloat() {
		replenishFromFile();
		return buffer.getFloat();
	}

	public void writeVector3f(Vector3f v) {
		writeFloat(v.x);
		writeFloat(v.y);
		writeFloat(v.z);
	}

	public void readVector3f(Vector3f v) {
		v.x = readFloat();
		v.y = readFloat();
		v.z = readFloat();
	}

	public void writeMatrix3f(Matrix3f m) {
		writeFloat(m.m00);
		writeFloat(m.m01);
		writeFloat(m.m02);
		writeFloat(m.m10);
		writeFloat(m.m11);
		writeFloat(m.m12);
		writeFloat(m.m20);
		writeFloat(m.m21);
		writeFloat(m.m22);
	}

	public void readMatrix3f(Matrix3f m) {
		m.m00 = readFloat();
		m.m01 = readFloat();
		m.m02 = readFloat();
		m.m10 = readFloat();
		m.m11 = readFloat();
		m.m12 = readFloat();
		m.m20 = readFloat();
		m.m21 = readFloat();
		m.m22 = readFloat();
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

}
