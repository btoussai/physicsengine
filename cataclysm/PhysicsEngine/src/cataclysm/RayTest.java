package cataclysm;

import math.vector.Vector3f;

public class RayTest {
	public enum Mode {
		Triangles, Wrappers, ALL;
	}

	private final Vector3f start = new Vector3f();
	private final Vector3f dir = new Vector3f();
	private float maxDistance = 0;
	private boolean backfaceCulling;
	private final Vector3f hitNormal = new Vector3f();
	private float hitDistance = Float.POSITIVE_INFINITY;
	private Mode mode;

	public RayTest(Mode mode) {
		this.mode = mode;
	}

	public void setMode(Mode mode, boolean backfaceCulling) {
		this.mode = mode;
		this.backfaceCulling = backfaceCulling;
	}

	public void reset(float startX, float startY, float startZ, float dirX, float dirY, float dirZ, float maxDistance) {
		start.set(startX, startY, startZ);
		dir.set(dirX, dirY, dirZ);
		this.maxDistance = maxDistance;
		this.hitDistance = Float.POSITIVE_INFINITY;
	}

	public void reset(Vector3f start, Vector3f dir, float maxDistance) {
		start.set(start);
		dir.set(dir);
		this.maxDistance = maxDistance;
		this.hitDistance = Float.POSITIVE_INFINITY;
	}

	public boolean hit() {
		return hitDistance < maxDistance;
	}

	public boolean miss() {
		return hitDistance > maxDistance;
	}

	public Mode getMode() {
		return mode;
	}

	public boolean isBackfaceCulling() {
		return backfaceCulling;
	}

	public float getHitDistance() {
		return hitDistance;
	}

	public Vector3f getHitNormal() {
		return hitNormal;
	}

	public Vector3f getStart() {
		return start;
	}

	public Vector3f getDir() {
		return dir;
	}

	public float getMaxDistance() {
		return maxDistance;
	}
	
	public void setHitDistance(float hitDistance) {
		this.hitDistance = hitDistance;
	}
}
