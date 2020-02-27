package math;

import math.vector.Vector2f;
import math.vector.Vector3f;
import math.vector.Vector4f;

/**
 * Cette classe contient quelques m�thodes utilitaires pour les vecteurs.
 * @author Briac
 *
 */
public class VectorOps {
	
	public static Vector2f min(Vector2f A, Vector2f B, Vector2f dest) {
		if(dest == null) {
			dest = new Vector2f();
		}
		dest.x = Maths.min(A.x, B.x);
		dest.y = Maths.min(A.y, B.y);
		return dest;
	}
	
	public static Vector2f max(Vector2f A, Vector2f B, Vector2f dest) {
		if(dest == null) {
			dest = new Vector2f();
		}
		dest.x = Maths.max(A.x, B.x);
		dest.y = Maths.max(A.y, B.y);
		return dest;
	}
	
	public static Vector3f min(Vector3f A, Vector3f B, Vector3f dest) {
		if(dest == null) {
			dest = new Vector3f();
		}
		dest.x = Maths.min(A.x, B.x);
		dest.y = Maths.min(A.y, B.y);
		dest.z = Maths.min(A.z, B.z);
		return dest;
	}
	
	public static Vector3f max(Vector3f A, Vector3f B, Vector3f dest) {
		if(dest == null) {
			dest = new Vector3f();
		}
		dest.x = Maths.max(A.x, B.x);
		dest.y = Maths.max(A.y, B.y);
		dest.z = Maths.max(A.z, B.z);
		return dest;
	}
	
	public static Vector4f min(Vector4f A, Vector4f B, Vector4f dest) {
		if(dest == null) {
			dest = new Vector4f();
		}
		dest.x = Maths.min(A.x, B.x);
		dest.y = Maths.min(A.y, B.y);
		dest.z = Maths.min(A.z, B.z);
		dest.w = Maths.min(A.w, B.w);
		return dest;
	}
	
	public static Vector4f max(Vector4f A, Vector4f B, Vector4f dest) {
		if(dest == null) {
			dest = new Vector4f();
		}
		dest.x = Maths.max(A.x, B.x);
		dest.y = Maths.max(A.y, B.y);
		dest.z = Maths.max(A.z, B.z);
		dest.w = Maths.max(A.w, B.w);
		return dest;
	}

}
