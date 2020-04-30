package cataclysm.record;

import math.vector.Vector3f;

public final class TriangleRepr implements ReadWriteObject {
	private final Vector3f v1 = new Vector3f();
	private final Vector3f v2 = new Vector3f();
	private final Vector3f v3 = new Vector3f();

	public TriangleRepr(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		f.readVector3f(v1);
		f.readVector3f(v2);
		f.readVector3f(v3);
	}

	@Override
	public void write(RecordFile f) {
		f.writeVector3f(v1);
		f.writeVector3f(v2);
		f.writeVector3f(v3);
	}

}
