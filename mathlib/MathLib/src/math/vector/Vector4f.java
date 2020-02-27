package math.vector;

import java.io.Serializable;
import java.nio.FloatBuffer;


/**
 * Simple class representing a 3-tuple vector, based on LWJGL vector classes
 * 
 * @author Briac
 *
 */
public final class Vector4f implements Serializable {

	private static final long serialVersionUID = 1L;

	public float x, y, z, w;

	/**
	 * Initialize this vector to zero
	 */
	public Vector4f() {
	}

	/**
	 * Performs a copy of src
	 * 
	 * @param src the source vector
	 */
	public Vector4f(Vector4f src) {
		set(src);
	}

	public Vector4f(float x, float y, float z, float w) {
		set(x, y, z, w);
	}

	public void set(float x, float y, float z, float w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}

	/**
	 * Loads src into this.
	 * 
	 * @param src
	 * @return this
	 */
	public Vector4f set(Vector4f src) {
		x = src.x;
		y = src.y;
		z = src.z;
		w = src.w;
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
	public static float length(Vector4f v) {
		return v.length();
	}

	/**
	 * @return the length squared of this vector
	 */
	public float lengthSquared() {
		return x * x + y * y + z * z + w * w;
	}

	/**
	 * @param v
	 * @return the length squared of v
	 */
	public static float lengthSquared(Vector4f v) {
		return v.lengthSquared();
	}

	/**
	 * Computes {@code this /= length(this)}
	 * 
	 * @return this
	 * @throws IllegalStateException if the vector is the zero vector
	 */
	public Vector4f normalise() throws IllegalStateException {
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
	public static Vector4f normalise(Vector4f src) {
		return new Vector4f(src).normalise();
	}

	/**
	 * Computes {@code dest = src / length(src)}
	 * 
	 * @param src
	 * @param dest
	 * @throws IllegalStateException if the vector src is the zero vector
	 */
	public static void normalise(Vector4f src, Vector4f dest) {
		dest.set(src);
		dest.normalise();
	}

	/**
	 * Translates this vector
	 * 
	 * @param x
	 * @param y
	 * @param z
	 * @param w 
	 * @return this
	 */
	public Vector4f translate(float x, float y, float z, float w) {
		this.x += x;
		this.y += y;
		this.z += z;
		this.w += w;
		return this;
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector4f translate(Vector4f v) {
		this.translate(v.x, v.y, v.z, v.w);
		return this;
	}

	/**
	 * Computes {@code this += v * scale}
	 * 
	 * @param v
	 * @param scale
	 * @return this
	 */
	public Vector4f translate(Vector4f v, float scale) {
		this.translate(v.x * scale, v.y * scale, v.z * scale, v.w * scale);
		return this;
	}

	/**
	 * Computes {@code this = -this}
	 * 
	 * @return this
	 */
	public Vector4f negate() {
		Vector4f.negate(this, this);
		return this;
	}

	/**
	 * Computes {@code -src} and puts the result in a new vector
	 * 
	 * @param src the source vector
	 * @return the new vector
	 */
	public static Vector4f negate(Vector4f src) {
		return new Vector4f(src).negate();
	}

	/**
	 * Computes {@code dest = -src}
	 * 
	 * @param src
	 * @param dest
	 */
	public static void negate(Vector4f src, Vector4f dest) {
		dest.x = src.x;
		dest.y = src.y;
		dest.z = src.z;
		dest.w = src.w;
	}

	/**
	 * Computes the dot product of left and right
	 * 
	 * @param left
	 * @param right
	 * @return the dot product of left and right
	 */
	public static float dot(Vector4f left, Vector4f right) {
		return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector4f add(Vector4f v) {
		Vector4f.add(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left + right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector4f add(Vector4f left, Vector4f right) {
		Vector4f dest = new Vector4f();
		Vector4f.add(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left + right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void add(Vector4f left, Vector4f right, Vector4f dest) {
		dest.set(left.x + right.x, left.y + right.y, left.z + right.z, left.w + right.w);
	}

	/**
	 * Computes {@code this -= v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector4f sub(Vector4f v) {
		Vector4f.sub(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left - right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector4f sub(Vector4f left, Vector4f right) {
		Vector4f dest = new Vector4f();
		Vector4f.sub(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left - right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void sub(Vector4f left, Vector4f right, Vector4f dest) {
		dest.set(left.x - right.x, left.y - right.y, left.z - right.z, left.w - right.w);
	}

	/**
	 * Computes {@code this *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector4f scale(float scale) {
		Vector4f.scale(this, this, scale);
		return this;
	}

	/**
	 * Computes {@code src * scale} and puts the result in a new vector
	 * 
	 * @param src
	 * @param scale
	 * @return the new vector
	 */
	public static Vector4f scale(Vector4f src, float scale) {
		return new Vector4f(src).scale(scale);
	}

	/**
	 * Computes {@code dest = src*scale}
	 * 
	 * @param src
	 * @param dest
	 * @param scale
	 */
	public static void scale(Vector4f src, Vector4f dest, float scale) {
		dest.x = src.x * scale;
		dest.y = src.y * scale;
		dest.z = src.z * scale;
		dest.w = src.w * scale;
	}

	/**
	 * Stores this vector in buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector4f store(FloatBuffer buf) {
		buf.put(x);
		buf.put(y);
		buf.put(z);
		buf.put(w);
		return this;
	}

	/**
	 * Loads this vector from buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector4f load(FloatBuffer buf) {
		x = buf.get();
		y = buf.get();
		z = buf.get();
		w = buf.get();
		return this;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder(64);

		sb.append("Vector4f[");
		sb.append(x);
		sb.append(", ");
		sb.append(y);
		sb.append(", ");
		sb.append(z);
		sb.append(", ");
		sb.append(w);
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
		Vector4f other = (Vector4f) obj;

		if (x == other.x && y == other.y && z == other.z && w == other.w)
			return true;

		return false;
	}

	@Override
	public int hashCode() {
		return Float.floatToRawIntBits(x) ^ Float.floatToRawIntBits(y) ^ Float.floatToRawIntBits(z) ^ Float.floatToRawIntBits(w);
	}

}
