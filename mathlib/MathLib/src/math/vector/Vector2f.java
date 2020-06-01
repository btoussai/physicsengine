package math.vector;

import java.io.Serializable;
import java.nio.FloatBuffer;

/**
 * Simple class representing a 2-tuple vector, based on LWJGL vector classes
 * 
 * @author Briac
 *
 */
public final class Vector2f implements Serializable {

	private static final long serialVersionUID = 1L;

	public float x, y;

	/**
	 * Initialize this vector to zero
	 */
	public Vector2f() {
	}

	/**
	 * Performs a copy of src
	 * 
	 * @param src the source vector
	 */
	public Vector2f(Vector2f src) {
		set(src);
	}

	public Vector2f(float x, float y) {
		set(x, y);
	}

	/**
	 * 
	 * @param x
	 * @param y
	 * @return this
	 */
	public Vector2f set(float x, float y) {
		this.x = x;
		this.y = y;
		return this;
	}
	
	/**
	 * @param x
	 * @return this
	 */
	public Vector2f setX(float x) {
		this.x = x;
		return this;
	}

	/**
	 * @param y
	 * @return this
	 */
	public Vector2f setY(float y) {
		this.y = y;
		return this;
	}

	/**
	 * Loads src into this.
	 * 
	 * @param src
	 * @return this
	 */
	public Vector2f set(Vector2f src) {
		x = src.x;
		y = src.y;
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
	public static float length(Vector2f v) {
		return v.length();
	}

	/**
	 * @return the length squared of this vector
	 */
	public float lengthSquared() {
		return x * x + y * y;
	}

	/**
	 * @param v
	 * @return the length squared of v
	 */
	public static float lengthSquared(Vector2f v) {
		return v.lengthSquared();
	}

	/**
	 * Computes {@code this /= length(this)}
	 * 
	 * @return this
	 * @throws IllegalStateException if the vector is the zero vector
	 */
	public Vector2f normalise() throws IllegalStateException {
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
	public static Vector2f normalise(Vector2f src) {
		return new Vector2f(src).normalise();
	}

	/**
	 * Computes {@code dest = src / length(src)}
	 * 
	 * @param src
	 * @param dest
	 * @throws IllegalStateException if the vector src is the zero vector
	 */
	public static void normalise(Vector2f src, Vector2f dest) {
		dest.set(src);
		dest.normalise();
	}

	/**
	 * Translates this vector
	 * 
	 * @param x
	 * @param y
	 * @return this
	 */
	public Vector2f translate(float x, float y) {
		this.x += x;
		this.y += y;
		return this;
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector2f translate(Vector2f v) {
		this.translate(v.x, v.y);
		return this;
	}

	/**
	 * Computes {@code this += v * scale}
	 * 
	 * @param v
	 * @param scale
	 * @return this
	 */
	public Vector2f translate(Vector2f v, float scale) {
		this.translate(v.x * scale, v.y * scale);
		return this;
	}

	/**
	 * Computes {@code this = -this}
	 * 
	 * @return this
	 */
	public Vector2f negate() {
		Vector2f.negate(this, this);
		return this;
	}

	/**
	 * Computes {@code -src} and puts the result in a new vector
	 * 
	 * @param src the source vector
	 * @return the new vector
	 */
	public static Vector2f negate(Vector2f src) {
		return new Vector2f(src).negate();
	}

	/**
	 * Computes {@code dest = -src}
	 * 
	 * @param src
	 * @param dest
	 */
	public static void negate(Vector2f src, Vector2f dest) {
		dest.x = -src.x;
		dest.y = -src.y;
	}

	/**
	 * Computes the dot product of left and right
	 * 
	 * @param left
	 * @param right
	 * @return the dot product of left and right
	 */
	public static float dot(Vector2f left, Vector2f right) {
		return left.x * right.x + left.y * right.y;
	}

	/**
	 * Computes {@code this += v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector2f add(Vector2f v) {
		Vector2f.add(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left + right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector2f add(Vector2f left, Vector2f right) {
		Vector2f dest = new Vector2f();
		Vector2f.add(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left + right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void add(Vector2f left, Vector2f right, Vector2f dest) {
		dest.set(left.x + right.x, left.y + right.y);
	}

	/**
	 * Computes {@code this -= v}
	 * 
	 * @param v
	 * @return this
	 */
	public Vector2f sub(Vector2f v) {
		Vector2f.sub(this, v, this);
		return this;
	}

	/**
	 * Computes {@code left - right} and puts the result in a new vector
	 * 
	 * @param left
	 * @param right
	 * @return the new vector
	 */
	public static Vector2f sub(Vector2f left, Vector2f right) {
		Vector2f dest = new Vector2f();
		Vector2f.sub(left, right, dest);
		return dest;
	}

	/**
	 * Computes {@code dest = left - right}
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void sub(Vector2f left, Vector2f right, Vector2f dest) {
		dest.set(left.x - right.x, left.y - right.y);
	}

	/**
	 * Computes {@code this *= scale}
	 * 
	 * @param scale
	 * @return this
	 */
	public Vector2f scale(float scale) {
		Vector2f.scale(this, this, scale);
		return this;
	}

	/**
	 * Computes {@code src * scale} and puts the result in a new vector
	 * 
	 * @param src
	 * @param scale
	 * @return the new vector
	 */
	public static Vector2f scale(Vector2f src, float scale) {
		return new Vector2f(src).scale(scale);
	}

	/**
	 * Computes {@code dest = src*scale}
	 * 
	 * @param src
	 * @param dest
	 * @param scale
	 */
	public static void scale(Vector2f src, Vector2f dest, float scale) {
		dest.x = src.x * scale;
		dest.y = src.y * scale;
	}

	/**
	 * Stores this vector in buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector2f store(FloatBuffer buf) {
		buf.put(x);
		buf.put(y);
		return this;
	}

	/**
	 * Loads this vector from buf
	 * 
	 * @param buf
	 * @return this
	 */
	public Vector2f load(FloatBuffer buf) {
		x = buf.get();
		y = buf.get();
		return this;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder(64);

		sb.append("Vector2f[");
		sb.append(x);
		sb.append(", ");
		sb.append(y);
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
		Vector2f other = (Vector2f) obj;

		if (x == other.x && y == other.y)
			return true;

		return false;
	}
	
	@Override
	public int hashCode() {
		return Float.floatToRawIntBits(x) ^ Float.floatToRawIntBits(y);
	}

}
