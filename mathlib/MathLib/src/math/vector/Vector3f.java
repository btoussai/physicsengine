package math.vector;

import java.io.Serializable;
import java.nio.FloatBuffer;

/**
 * Simple class representing a 3-tuple vector, based on LWJGL vector classes
 * 
 * @author Briac
 *
 */
public final class Vector3f implements Serializable {

	private static final long serialVersionUID = 1L;

	public float x, y, z;

	/**
	 * Initialize this vector to zero
	 */
	public Vector3f() {
	}

	/**
	 * Performs a copy of src
	 * 
	 * @param src the source vector
	 */
	public Vector3f(Vector3f src) {
		set(src);
	}

	public Vector3f(float x, float y, float z) {
		set(x, y, z);
	}

	/**
	 * Short-hand for <br>
	 * {@code new Vector3f(value, value, value)}
	 * 
	 * @param value
	 */
	public Vector3f(float value) {
		set(value, value, value);
	}

	public Vector3f set(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
		return this;
	}

	/**
	 * Loads src into this.
	 * 
	 * @param src
	 * @return this
	 */
	public Vector3f set(Vector3f src) {
		x = src.x;
		y = src.y;
		z = src.z;
		return this;
	}

	/**
	 * @return the length of this vector
	 */
	public float length() {
		return (float) Math.sqrt(lengthSquared());
	}

	/**
	 * @param v
	 * @return the length of v
	 */
	public static float length(Vector3f v) {
		return v.length();
	}

	/**
	 * @return the length squared of this vector
	 */
	public float lengthSquared() {
		return x * x + y * y + z * z;
	}

	/**
	 * @param v
	 * @return the length squared of v
	 */
	public static float lengthSquared(Vector3f v) {
		return v.lengthSquared();
	}

	/**
	 * Computes {@code this /= length(this)}
	 * 
	 * @return this
	 * @throws IllegalStateException if the vector is the zero vector
	 */
	public Vector3f normalise() throws IllegalStateException {
		float l = length();
		if (l != 0.0f) {
			return scale(1.0f / l);
		} else
			throw new IllegalStateException("Zero length vector");
	}

	/**
	 * Computes {@code src / length(src)} and puts the result in a new vector
	 * 
	 * @param src
	 * @return the new vector
	 * @throws IllegalStateException if the vector src is the zero vector
	 */
	public static Vector3f normalise(Vector3f src) {
		return new Vector3f(src).normalise();
	}

	/**
	 * Computes {@code dest = src / length(src)}
	 * 
	 * @param src
	 * @param dest
	 * @throws IllegalStateException if the vector src is the zero vector
	 */
	public static void normalise(Vector3f src, Vector3f dest) {
		dest.set(src);
		dest.normalise();
	}

	/**
	 * Translates this vector
	 * 
	 * @param x
	 * @param y
	 * @param z
	 * @return this
	 */
	public Vector3f translate(float x, float y, float z) {
		this.x += x;
		this.y += y;
		this.z += z;
		return this;
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector3f translate(Vector3f v) {
		this.translate(v.x, v.y, v.z);
		return this;
	}

	/**
	 * Computes {@code this += v * scale}
	 * 
	 * @param v
	 * @param scale
	 * @return this
	 */
	public Vector3f translate(Vector3f v, float scale) {
		this.translate(v.x * scale, v.y * scale, v.z * scale);
		return this;
	}

	/**
	 * Computes {@code this = -this}
	 * 
	 * @return this
	 */
	public Vector3f negate() {
		Vector3f.negate(this, this);
		return this;
	}

	/**
	 * Computes {@code -src} and puts the result in a new vector
	 * 
	 * @param src the source vector
	 * @return the new vector
	 */
	public static Vector3f negate(Vector3f src) {
		return new Vector3f(src).negate();
	}

	/**
	 * Computes {@code dest = -src}
	 * 
	 * @param src
	 * @param dest
	 */
	public static void negate(Vector3f src, Vector3f dest) {
		dest.x = -src.x;
		dest.y = -src.y;
		dest.z = -src.z;
	}

	/**
	 * Computes the dot product of left and right
	 * 
	 * @param left
	 * @param right
	 * @return the dot product of left and right
	 */
	public static float dot(Vector3f left, Vector3f right) {
		return left.x * right.x + left.y * right.y + left.z * right.z;
	}

	/**
	 * Computes {@code left x right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector3f cross(Vector3f left, Vector3f right) {
		Vector3f dest = new Vector3f();
		Vector3f.cross(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left x right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void cross(Vector3f left, Vector3f right, Vector3f dest) {
		dest.set(left.y * right.z - left.z * right.y, right.x * left.z - right.z * left.x,
				left.x * right.y - left.y * right.x);
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector3f add(Vector3f v) {
		Vector3f.add(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left + right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector3f add(Vector3f left, Vector3f right) {
		Vector3f dest = new Vector3f();
		Vector3f.add(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left + right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void add(Vector3f left, Vector3f right, Vector3f dest) {
		dest.set(left.x + right.x, left.y + right.y, left.z + right.z);
	}

	/**
	 * Computes {@code this -= v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector3f sub(Vector3f v) {
		Vector3f.sub(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left - right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector3f sub(Vector3f left, Vector3f right) {
		Vector3f dest = new Vector3f();
		Vector3f.sub(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left - right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void sub(Vector3f left, Vector3f right, Vector3f dest) {
		dest.set(left.x - right.x, left.y - right.y, left.z - right.z);
	}

	/**
	 * Computes {@code this *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector3f scale(float scale) {
		Vector3f.scale(this, this, scale);
		return this;
	}

	/**
	 * Computes {@code this.x *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector3f scaleX(float scale) {
		this.x *= scale;
		return this;
	}

	/**
	 * Computes {@code this.y *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector3f scaleY(float scale) {
		this.y *= scale;
		return this;
	}

	/**
	 * Computes {@code this.z *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector3f scaleZ(float scale) {
		this.z *= scale;
		return this;
	}

	/**
	 * Equivalent to {@code this.scaleX(sx).scaleY(sy).scaleZ(sz) }
	 * 
	 * @param sx
	 * @param sy
	 * @param sz
	 * 
	 * @return this
	 */
	public Vector3f scaleXYZ(float sx, float sy, float sz) {
		scaleX(sx);
		scaleY(sy);
		scaleZ(sz);
		return this;
	}

	/**
	 * Computes {@code src * scale} and puts the result in a new vector
	 * 
	 * @param src
	 * @param scale
	 * @return the new vector
	 */
	public static Vector3f scale(Vector3f src, float scale) {
		return new Vector3f(src).scale(scale);
	}

	/**
	 * Computes {@code dest = src*scale}
	 * 
	 * @param src
	 * @param dest
	 * @param scale
	 */
	public static void scale(Vector3f src, Vector3f dest, float scale) {
		dest.x = src.x * scale;
		dest.y = src.y * scale;
		dest.z = src.z * scale;
	}

	/**
	 * Stores this vector in buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector3f store(FloatBuffer buf) {
		buf.put(x);
		buf.put(y);
		buf.put(z);
		return this;
	}

	/**
	 * Loads this vector from buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector3f load(FloatBuffer buf) {
		x = buf.get();
		y = buf.get();
		z = buf.get();
		return this;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder(64);

		sb.append("Vector3f[");
		sb.append(x);
		sb.append(", ");
		sb.append(y);
		sb.append(", ");
		sb.append(z);
		sb.append(']');
		return sb.toString();
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vector3f other = (Vector3f) obj;

		if (x == other.x && y == other.y && z == other.z)
			return true;

		return false;
	}

	@Override
	public int hashCode() {
		return Float.floatToRawIntBits(x) ^ Float.floatToRawIntBits(y) ^ Float.floatToRawIntBits(z);
	}

}
