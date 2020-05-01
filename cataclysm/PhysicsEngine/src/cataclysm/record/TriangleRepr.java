package cataclysm.record;

import math.vector.Vector3f;

public final class TriangleRepr implements ReadWriteObject {
	public final Vector3f v1 = new Vector3f();
	public final Vector3f v2 = new Vector3f();
	public final Vector3f v3 = new Vector3f();

	public TriangleRepr() {
		
	}
	
	
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


	@Override
	public int size() {
		return 3 * 3 * 4;
	}

}
